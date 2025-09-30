#include "WebSocketServer.hpp"
#include "esp_http_server.h"
#include "esp_log.h"

#include <algorithm>
#include <cstring> // strlen
#include <string>
#include <vector>

#include "Tasks.hpp" // EcgSetNotchHz / EcgSetNotchQ
#include <cstdlib>	 // strtof

static const char *TAG = "WebSocket";
static httpd_handle_t s_server = nullptr;
static std::vector<int> s_clients;

// Symbols created by EMBED_FILES (index.html)
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");

// Serve the embedded index.html at "/"
static esp_err_t root_handler(httpd_req_t *req)
{
	const size_t len = index_html_end - index_html_start;
	httpd_resp_set_type(req, "text/html");
	return httpd_resp_send(
		req, reinterpret_cast<const char *>(index_html_start), len);
}

// WebSocket handler: handshake + (optional) inbound frames
static esp_err_t ws_handler(httpd_req_t *req)
{
	if (req->method == HTTP_GET)
	{
		const int fd = httpd_req_to_sockfd(req);
		if (std::find(s_clients.begin(), s_clients.end(), fd) ==
			s_clients.end())
		{
			s_clients.push_back(fd);
			ESP_LOGI(TAG, "client connected fd=%d (total=%zu)", fd,
					 s_clients.size());
		}
		return ESP_OK;
	}

	// Drain any inbound frame (we ignore content)
	httpd_ws_frame_t frame{};
	esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
	if (ret != ESP_OK)
		return ret;

	if (frame.len)
	{
		std::string buf(frame.len, '\0');
		frame.payload = reinterpret_cast<uint8_t *>(buf.data());
		ret = httpd_ws_recv_frame(req, &frame, frame.len);
		if (ret != ESP_OK)
			return ret;

		if (frame.type == HTTPD_WS_TYPE_TEXT)
		{
			// Expect messages like: "notchHz=50" or "notchQ=30"
			if (buf.rfind("notchHz=", 0) == 0)
			{
				float hz = strtof(buf.c_str() + 8, nullptr);
				if (hz >= 40.0f && hz <= 70.0f)
					EcgSetNotchHz(hz);
			}
			else if (buf.rfind("notchQ=", 0) == 0)
			{
				float q = strtof(buf.c_str() + 7, nullptr);
				if (q >= 5.0f && q <= 80.0f)
					EcgSetNotchQ(q);
			}
		}
	}
	return ESP_OK;
}

// Called by HTTP server when a socket closes
static void on_close(httpd_handle_t, int sockfd)
{
	auto it = std::remove(s_clients.begin(), s_clients.end(), sockfd);
	if (it != s_clients.end())
	{
		s_clients.erase(it, s_clients.end());
		ESP_LOGI(TAG, "client disconnected fd=%d (total=%zu)", sockfd,
				 s_clients.size());
	}
}

// ---- WebSocketServer (class static methods) ----

void WebSocketServer::start()
{
	if (s_server)
		return;

	httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
	cfg.uri_match_fn = httpd_uri_match_wildcard;
	cfg.lru_purge_enable = true;
	cfg.close_fn = on_close; // note: void(*) signature

	ESP_ERROR_CHECK(httpd_start(&s_server, &cfg));

	// HTTP route: "/"
	httpd_uri_t root_uri = {
		.uri = "/",
		.method = HTTP_GET,
		.handler = root_handler,
		.user_ctx = nullptr,
		.is_websocket = false,
		.handle_ws_control_frames = false,
		.supported_subprotocol = nullptr,
	};
	ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &root_uri));

	// WebSocket route: "/ws"
	httpd_uri_t ws_uri = {
		.uri = "/ws",
		.method = HTTP_GET,
		.handler = ws_handler,
		.user_ctx = nullptr,
		.is_websocket = true,
		.handle_ws_control_frames = true,
		.supported_subprotocol = nullptr,
	};
	ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &ws_uri));

	ESP_LOGI(TAG, "WebSocket server started");
}

void WebSocketServer::broadcast(const char *message)
{
	if (!s_server || s_clients.empty() || !message)
		return;

	httpd_ws_frame_t pkt{};
	pkt.type = HTTPD_WS_TYPE_TEXT;
	pkt.payload = (uint8_t *)message;
	pkt.len = std::strlen(message);
	pkt.final = true;

	for (int fd : s_clients)
	{
		esp_err_t r = httpd_ws_send_frame_async(s_server, fd, &pkt);
		if (r != ESP_OK)
			ESP_LOGW(TAG, "ws text send fd=%d err=%d", fd, r);
	}
}

void WebSocketServer::broadcastBinary(const void *data, size_t len)
{
	if (!s_server || s_clients.empty() || !data || len == 0)
		return;

	httpd_ws_frame_t pkt{};
	pkt.type = HTTPD_WS_TYPE_BINARY;
	pkt.payload = (uint8_t *)data;
	pkt.len = len;
	pkt.final = true;

	for (int fd : s_clients)
	{
		esp_err_t r = httpd_ws_send_frame_async(s_server, fd, &pkt);
		if (r != ESP_OK)
			ESP_LOGW(TAG, "ws bin send fd=%d err=%d", fd, r);
	}
}
