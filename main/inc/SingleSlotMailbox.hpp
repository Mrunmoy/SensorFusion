#pragma once

#include <atomic>
#include <mutex>

template <typename T> class SingleSlotMailbox
{
  public:
	SingleSlotMailbox()
		: m_sequence(0)
	{
	}

	void publish(const T &value)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_data = value;
		m_sequence.fetch_add(1, std::memory_order_release);
	}

	void publish(T &&value)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_data = std::move(value);
		m_sequence.fetch_add(1, std::memory_order_release);
	}

	bool tryRead(T &out, uint32_t *seq_out = nullptr) const
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		out = m_data;
		if (seq_out)
		{
			*seq_out = m_sequence.load(std::memory_order_acquire);
		}
		return true;
	}

	T readLatest() const
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		return m_data;
	}

	bool hasNew(uint32_t last_seen) const
	{
		return m_sequence.load(std::memory_order_acquire) != last_seen;
	}

	uint32_t sequence() const
	{
		return m_sequence.load(std::memory_order_acquire);
	}

  private:
	mutable std::atomic<uint32_t> m_sequence;
	mutable std::mutex m_mutex;
	T m_data{};
};