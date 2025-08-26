#pragma once

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdint>
#include <functional>
#include <vector>

struct Poller
{
	const char *name;
	uint32_t hz;
	uint32_t max_duration_ms;	// budget for one poll()
	std::function<bool()> poll; // return true on success
	// internal:
	uint32_t next_due_ms{0};
	uint32_t backoff_until_ms{0};
	uint8_t fail_count{0};
};

class PollScheduler
{
  public:
	void add(const Poller &p)
	{
		m_pollers.push_back(p);
	}
	void start(const char *task_name = "PollScheduler", uint32_t stack = 4096,
			   UBaseType_t prio = 5);

  private:
	static void taskEntry(void *arg);
	void run();

  private:
	std::vector<Poller> m_pollers;
};
