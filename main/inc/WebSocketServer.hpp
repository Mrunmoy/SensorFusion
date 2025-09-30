#pragma once
#include <cstddef>

class WebSocketServer
{
  public:
	// Start HTTP + WebSocket server (idempotent)
	static void start();

	// Optional: send a text message to all clients
	static void broadcast(const char *message);

	// NEW: send binary payload to all clients (e.g., ECG int16 batches)
	static void broadcastBinary(const void *data, size_t len);
};
