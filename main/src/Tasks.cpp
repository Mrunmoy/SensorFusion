#include "Tasks.hpp"
#include "Ad8232Driver.hpp"
#include "MicroPacer.hpp"
#include "SensorAdcBus.hpp"
#include "WebSocketServer.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <atomic>
#include <cmath>

// Live-tunable notch params (default AU: 50 Hz, Q=30)
static std::atomic<float> g_notchHz{50.0f};
static std::atomic<float> g_notchQ{30.0f};

extern "C" void EcgSetNotchHz(float hz)
{
	g_notchHz.store(hz);
}
extern "C" void EcgSetNotchQ(float q)
{
	g_notchQ.store(q);
}

// RBJ-style biquad notch (band-stop)
struct Notch50Hz
{
	// coefficients
	float b0 = 1, b1 = 0, b2 = 1, a1 = 0, a2 = 0;
	// state
	float z1 = 0, z2 = 0;

	// fs: sample rate (e.g., 250), f0: notch freq (50 or 60), Q: quality factor
	// (~20-40)
	void configure(float fs, float f0 = 50.0f, float Q = 30.0f)
	{
		const float w0 = 2.0f * static_cast<float>(M_PI) * (f0 / fs);
		const float cw = std::cos(w0);
		const float sw = std::sin(w0);
		const float alpha = sw / (2.0f * Q);

		// un-normalized
		float bb0 = 1.0f;
		float bb1 = -2.0f * cw;
		float bb2 = 1.0f;
		float aa0 = 1.0f + alpha;
		float aa1 = -2.0f * cw;
		float aa2 = 1.0f - alpha;

		// normalize by a0
		b0 = bb0 / aa0;
		b1 = bb1 / aa0;
		b2 = bb2 / aa0;
		a1 = aa1 / aa0;
		a2 = aa2 / aa0;

		// reset state on reconfig
		z1 = z2 = 0.0f;
	}

	// process one sample (in mV). returns filtered mV (float -> int)
	inline int apply(int x)
	{
		// Direct Form II (RBJ)
		float y = b0 * x + z1;
		z1 = b1 * x + z2 - a1 * y;
		z2 = b2 * x - a2 * y;
		return static_cast<int>(y);
	}
};

struct DcBlock
{
	float alpha = 0.99f;
	int prev_in = 0;
	float prev_out = 0.0f;
	int apply(int mv)
	{
		float y = float(mv) - float(prev_in) + alpha * prev_out;
		prev_in = mv;
		prev_out = y;
		return int(y);
	}
};

extern "C" void EcgTask(void *pv)
{
	static const char *TAG = "EcgTask";
	auto *ctx = static_cast<EcgCtx *>(pv);
	if (!ctx)
	{
		vTaskDelete(nullptr);
		return;
	}

	static SensorAdcBus adcBus;
	static Ad8232Driver ecg(adcBus);

	Ad8232Driver::Config cfg;
	cfg.unit = ADC_UNIT_1;
	cfg.channel = ADC_CHANNEL_0;				  // GPIO1 on ESP32-S3
	cfg.atten = Ad8232Driver::Attenuation::DB_11; // maps to DB_12 internally
	cfg.calibrate = true;

	if (!ecg.init(cfg))
	{
		ESP_LOGE(TAG, "ECG init failed");
		vTaskDelete(nullptr);
		return;
	}

	auto pacer = make_pacer(ctx->hz);
	DcBlock dc;

	// 50 Hz notch (use 60.0f if you’re on 60 Hz mains)
	// Live-tunable notch
	Notch50Hz notch;
	float currHz = g_notchHz.load();
	float currQ = g_notchQ.load();
	notch.configure(ctx->hz, currHz, currQ);

	constexpr int BATCH = 25; // 100ms @ 250 Hz
	int16_t batch[BATCH];
	int bi = 0;
	int lastGood = 0;

	uint32_t printCount = 0;
	ESP_LOGI(TAG, "start @ %.1f Hz", ctx->hz);

	for (;;)
	{
		sleep_until(pacer);

		// Hot-reconfigure if sliders changed
		float hz = g_notchHz.load();
		float q = g_notchQ.load();
		if (hz != currHz || q != currQ)
		{
			currHz = hz;
			currQ = q;
			notch.configure(ctx->hz, currHz, currQ);
		}

		int mv = -1;
		if (!ecg.sample(mv))
			continue;

		if (mv != -1)
			lastGood = mv;
		int hp = dc.apply(mv == -1 ? lastGood : mv);

		// apply notch
		int hpNotched = notch.apply(hp);
		// hpNotched = notch2.apply(hpNotched); // if you enabled 100 Hz notch

		// clip and store
		if (hpNotched > 32767)
			hpNotched = 32767;
		if (hpNotched < -32768)
			hpNotched = -32768;
		batch[bi++] = static_cast<int16_t>(hpNotched);

		// logging (unchanged)
		if ((++printCount % (uint32_t)(ctx->hz / 2.0f)) == 0)
			ESP_LOGI(TAG, "ECG hp=%d mV notch=%d mV", hp, hpNotched);

		if (bi == BATCH)
		{
			WebSocketServer::broadcastBinary(batch, sizeof(batch));
			bi = 0;
		}
	}
}
