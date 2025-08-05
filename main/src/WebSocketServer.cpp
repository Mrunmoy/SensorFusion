#include "WebSocketServer.hpp"
#include "esp_log.h"
#include <string>
#include "esp_http_server.h"

static const char* TAG = "WebSocket";
static httpd_handle_t server = nullptr;
static int client_fd = -1;

// Declare symbols created by `EMBED_FILES`
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

// Handler for WebSocket upgrade and message exchange
static esp_err_t websocket_handler(httpd_req_t* req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket handshake complete");
        client_fd = httpd_req_to_sockfd(req);
        return ESP_OK;
    }
    return ESP_FAIL;
}

// Handler to serve the embedded index.html at "/"
static esp_err_t root_handler(httpd_req_t* req) {
    size_t html_len = index_html_end - index_html_start;

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, reinterpret_cast<const char*>(index_html_start), html_len);
}

// Start both HTTP and WebSocket server
void WebSocketServer::start() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t ws_uri = {
            .uri = "/ws",
            .method = HTTP_GET,
            .handler = websocket_handler,
            .user_ctx = nullptr,
            .is_websocket = true,
            .handle_ws_control_frames = true,
            .supported_subprotocol = nullptr,
        };


        httpd_uri_t root_uri = {
            .uri      = "/",
            .method   = HTTP_GET,
            .handler  = root_handler,
            .user_ctx = nullptr,
            .is_websocket = true,
            .handle_ws_control_frames = true,
            .supported_subprotocol = nullptr,
        };

        httpd_register_uri_handler(server, &ws_uri);
        httpd_register_uri_handler(server, &root_uri);

        ESP_LOGI(TAG, "WebSocket server started");
    } else {
        ESP_LOGE(TAG, "Failed to start WebSocket server");
    }
}

// Send message to the WebSocket client (if connected)
void WebSocketServer::broadcast(const char* message) {
    if (client_fd < 0) {
        ESP_LOGW(TAG, "No active WebSocket client.");
        return;
    }

    httpd_ws_frame_t frame = {};
    frame.type = HTTPD_WS_TYPE_TEXT;
    frame.payload = (uint8_t*)message;
    frame.len = strlen(message);

    esp_err_t ret = httpd_ws_send_frame_async(server, client_fd, &frame);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send WebSocket frame: %s", esp_err_to_name(ret));
        client_fd = -1; // reset if client likely gone
    }
}
