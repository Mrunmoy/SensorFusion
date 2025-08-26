#pragma once

#include "MPU6050Driver.hpp"
#include "SensorTypes.hpp"
#include "SingleSlotMailbox.hpp"
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class ImuPollTask
{
  public:
	ImuPollTask(MPU6050Driver &driver, SingleSlotMailbox<ImuSample> &mailbox,
				uint32_t hz = 100) // default 100 Hz
		: m_driver(driver), m_mailbox(mailbox), m_pollingFrequency(hz)
	{
	}

	bool start(const char *taskName = "ImuPollTask", uint32_t stack = 4096,
			   UBaseType_t prio = 5)
	{
		if (m_running.load())
			return true;
		m_running.store(true);
		return xTaskCreate(&ImuPollTask::taskEntry, taskName, stack, this, prio,
						   &m_task) == pdPASS;
	}

	void stop()
	{
		if (!m_running.load())
			return;
		m_running.store(false);
		// Wait for task to exit cleanly
		while (m_task != nullptr)
		{
			vTaskDelay(pdMS_TO_TICKS(10));
		}
	}

  private:
	static void taskEntry(void *arg)
	{
		static_cast<ImuPollTask *>(arg)->run();
	}

	void run()
	{
		const TickType_t period =
			pdMS_TO_TICKS(1000 / (m_pollingFrequency ? m_pollingFrequency : 1));
		TickType_t lastWake = xTaskGetTickCount();

		ImuSample sample{};
		float ax, ay, az, gx, gy, gz, tempC;

		while (m_running.load())
		{
			bool okA = m_driver.readAcceleration(ax, ay, az);
			bool okG = m_driver.readGyroscope(gx, gy, gz);
			bool okT = m_driver.readTemperature(tempC);

			if (okA && okG && okT)
			{
				// Driver already returns m/s^2 and deg/s; convert gyro to rad/s
				// here
				sample.acceleration.x = ax;
				sample.acceleration.y = ay;
				sample.acceleration.z = az;

				constexpr float deg2rad = 0.017453292519943295f;
				sample.rotation.x = gx * deg2rad;
				sample.rotation.y = gy * deg2rad;
				sample.rotation.z = gz * deg2rad;

				sample.temperature_c = tempC;

				// Monotonic ms timestamp from FreeRTOS tick count
				sample.timestamp_ms = static_cast<uint32_t>(
					(xTaskGetTickCount() * portTICK_PERIOD_MS));

				m_mailbox.publish(sample);
			}
			// else: skip publish this cycle (keeps last good sample available)

			vTaskDelayUntil(&lastWake, period);
		}

		m_task = nullptr; // signal stop() we are done
		vTaskDelete(nullptr);
	}

  private:
	MPU6050Driver &m_driver;
	SingleSlotMailbox<ImuSample> &m_mailbox;

	TaskHandle_t m_task{nullptr};
	std::atomic_bool m_running{false};
	uint32_t m_pollingFrequency;
};
