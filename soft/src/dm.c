#include "dm.h"
#include "adc.h"
#include "debug.h"
#include "main.h"
#include "mpu6050.h"
#include "ws2812.h"
#include <math.h>

#include "phys_engine.h"

extern RNG_HandleTypeDef hrng;

DM_SHOW_T show = DM_IDLE, show_next = DM_IDLE;
uint32_t to = 0;

float param[DM_COUNT] = {0};

DM_SHOW_T dm_get_mode(void) { return show; }

#define PERIODIC_DECIMATION_CNT(value, diff, ...) \
	{                                             \
		static uint32_t cnt = 0;                  \
		cnt += diff;                              \
		if(cnt >= (value))                        \
		{                                         \
			cnt = 0;                              \
			__VA_ARGS__;                          \
		}                                         \
	}

void dm_switch(DM_SHOW_T mode)
{
	show = mode;
	debug("DM switch: %d\n", show);
}

void dm_switch_parameter(void)
{
	float *p = &param[show];
	switch(show)
	{
	default: break;

	case DM_RANDOM_COLOR:
	case DM_RAINBOW_ROTATE:
	case DM_ROTATING_STRIPES:
	case DM_PENDULUM:
		*p += 30;
		if(*p < 100 || *p > 240) *p = 100;
		break;

	case DM_STRIPPER_FIXED:
		*p *= 1.6f;
		if(*p < 120 || *p > 800) *p = 120;
		break;

	case DM_RIDE0:
		*p += 20;
		if(*p < 80 || *p > 240) *p = 80;
		break;
	}

	debug("\tparam: %.3f\n", *p);
}

void dm_init(void)
{
	for(int i = 0; i < DM_COUNT; i++)
	{
		show = (DM_SHOW_T)i;
		dm_switch_parameter();
	}
	show = DM_IDLE;
}

void dm_switch_mode_vbat(void)
{
	to = 2000;
	show_next = show;
	dm_switch(DM_BAT);
}

void dm_switch_mode_sparkle(void)
{
	to = 1200;
	show_next = show;
	dm_switch(DM_SPARKLE);
	param[show] = HAL_RNG_GetRandomNumber(&hrng) % 6;
}

void dm_switch_mode_next(void)
{
	show++;
	if(show >= DM_COUNT || show < DM_IDLE) show = DM_IDLE;

	dm_switch(show);

	switch(show)
	{
	default: break;
	case DM_PENDULUM: phys_engine_reset(); break;
	}
	ws2812_clear();
}

void dm_poll(uint32_t diff_ms)
{
	if(to)
	{
		if(to <= diff_ms)
		{
			to = 0;
			show = show_next;
			debug("DM TO switch: %d\n", show);
			ws2812_clear();
		}
		else
			to -= diff_ms;
	}

	switch(show)
	{
	default:
	case DM_IDLE: break;

	case DM_BAT:
	{
		uint32_t light = get_percent_battery() * LED_COUNT;
		color_t color_light = {140, 0, 0};
		for(uint32_t i = 0; i < light; i++)
		{
			ws2812_set_led(i, &color_light);
		}
		for(uint32_t i = light; i < LED_COUNT; i++)
		{
			ws2812_set_led(i, &black);
		}
	}
	break;

	case DM_RANDOM_COLOR:
	{
		static uint32_t trans_timer = 0;
		static float hue_tgt = 0, hue_now = 0, hue_incr;

		if(trans_timer == 0)
		{
			trans_timer = 1000 + (HAL_RNG_GetRandomNumber(&hrng) % 8000);

			do
			{
				hue_tgt = HAL_RNG_GetRandomNumber(&hrng) % 360;
			} while(fabsf(hue_tgt - hue_now) < 20 || fabsf(hue_tgt - hue_now) > 340);

			if(fabsf(hue_tgt - hue_now) < 360 - fabsf(hue_tgt - hue_now))
				hue_incr = (hue_tgt - hue_now) / (float)trans_timer;
			else
				hue_incr = (360 - hue_tgt - hue_now) / (float)trans_timer;
			debug("hue %.2f => %.2f, incr: %.4f, time: %d\n", hue_now, hue_tgt, hue_incr, trans_timer);
		}
		else
		{
			trans_timer = trans_timer < diff_ms ? 0 : trans_timer - diff_ms;
			hue_now += hue_incr * (float)diff_ms;
			if(hue_now > 360.0f) hue_now -= 360.0f;
			if(hue_now < 0.0f) hue_now += 360.0f;
		}

		color_t light_color = hsv2rgb(hue_now, 1.0f, param[show]);
		ws2812_set_led_all(&light_color);
	}
	break;

	case DM_ROTATING_STRIPES:
	{
#define LED_PER_STRIPE 8
#define NUM_STRIPES LED_COUNT / LED_PER_STRIPE /* per stripe */
		static struct
		{
			float hue;
			float hue_vel;
			uint32_t change_to;
		} stripe[NUM_STRIPES / 2] = {0};

		static float offset = 0;
		offset += 0.003f * (float)diff_ms;
		if(offset >= LED_COUNT) offset -= LED_COUNT;
		ws2812_clear();
		for(uint32_t i = 0; i < sizeof(stripe) / sizeof(stripe[0]); i++)
		{
			if(stripe[i].change_to < diff_ms)
			{
				stripe[i].change_to = 1000 + (HAL_RNG_GetRandomNumber(&hrng) % 4000);
				stripe[i].hue_vel = 0.001 + (float)(HAL_RNG_GetRandomNumber(&hrng) % 100) * 0.00025f;
				stripe[i].hue_vel *= (HAL_RNG_GetRandomNumber(&hrng) % 2) ? 1.0f : -1.0f;
			}
			else
			{
				stripe[i].change_to -= diff_ms;
			}
			stripe[i].hue += stripe[i].hue_vel * diff_ms;
			normalize_hue(&stripe[i].hue);
			color_t light_color;
			for(int o = 0; o < LED_PER_STRIPE + 1; o++)
			{
				float br = sin((ceil(offset) - offset + o) / 3.14159256f);
				light_color = hsv2rgb(stripe[i].hue, 1.0f, br * param[show]);
				ws2812_set_led_recursive(LED_COUNT - (offset + i * 2 * LED_PER_STRIPE + o), &light_color);
			}
		}
	}
	break;

#define BRB_BRD_0 4
#define BRB_BRD_1 59
#define BRB_BRD_2 94
#define BRB_BRD_3 149

	case DM_STRIPPER_FIXED:
	{
		static color_t color;
		static uint32_t timer = 0;
		timer += diff_ms;
		if(timer > param[show])
		{
			color = hsv2rgb(HAL_RNG_GetRandomNumber(&hrng) % 360, 1.0f, 240);
			timer = 0;
		}
		if(timer < 0.4 * param[show])
		{
			for(uint32_t i = 0; i < BRB_BRD_0; i++)
				ws2812_set_led(i, &color);
			for(uint32_t i = BRB_BRD_1; i < BRB_BRD_2; i++)
				ws2812_set_led(i, &color);
			for(uint32_t i = BRB_BRD_3; i < LED_COUNT; i++)
				ws2812_set_led(i, &color);
		}
		else
		{
			ws2812_clear();
		}
	}
	break;

	case DM_RAINBOW_ROTATE:
	{
		static uint32_t offset = 0;
		offset += diff_ms;
		uint32_t offset_scaled = offset / 30;
		if(offset_scaled == LED_COUNT) offset = offset_scaled = 0;

		for(uint32_t i = 0; i < LED_COUNT; i++)
		{
			int32_t hue = (int32_t)(offset_scaled + i) * 360 / LED_COUNT;
			hue %= 360;
			color_t light_color = hsv2rgb(hue, 1.0f, param[show]);
			ws2812_set_led(i, &light_color);
		}
	}
	break;

	case DM_PENDULUM:
	{
		float ts = (float)diff_ms * 0.001f;

		// static float angle = 1; //3.14159256/2;
		float angle = -0.42f +
					  approx_atan2((float)accel_filt[1] * 0.00024420024 /*1/4095*/,
								   (float)accel_filt[2] * 0.00024420024); // 3.14159256/2;

		phys_engine_poll(ts, angle);

		ws2812_set_angle(phys_engine_get_angle(), phys_engine_get_w() * 20.0f, param[show], 25 /* led count */);
	}
	break;

	case DM_POLICE:
	{
		enum
		{
			PHASE_RED_0,
			PHASE_RED_1,
			PHASE_RED_OFF,
			PHASE_BLU_0,
			PHASE_BLU_1,
			PHASE_BLU_OFF
		};

#define FLASH_CNT 30
#define FLASH_COLOR_LEN 35
#define FLASH_DELAY 200
#define POLICE_OFFSET 12

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
		static uint32_t phase = PHASE_BLU_OFF;
		if(phase == PHASE_BLU_OFF || phase == PHASE_RED_OFF)
		{
			ws2812_clear();
			PERIODIC_DECIMATION_CNT(FLASH_DELAY, diff_ms, { phase = phase == PHASE_BLU_OFF ? PHASE_RED_0 : PHASE_BLU_0; });
		}
		else if(phase == PHASE_RED_0 || phase == PHASE_RED_1)
		{
			PERIODIC_DECIMATION_CNT(FLASH_COLOR_LEN, diff_ms, {
				if(phase == PHASE_RED_0)
				{
					phase = PHASE_RED_1;

					color_t c = {250, 0, 0};
					for(uint32_t i = 0; i < LED_COUNT / 2; i++)
					{
						ws2812_set_led_recursive(i - POLICE_OFFSET, &c);
					}
				}
				else
				{
					phase = PHASE_RED_0;
					ws2812_clear();
				}
				PERIODIC_DECIMATION_CNT(FLASH_CNT, diff_ms, { phase = PHASE_RED_OFF; });
			});
		}
		else if(phase == PHASE_BLU_0 || phase == PHASE_BLU_1)
		{
			PERIODIC_DECIMATION_CNT(FLASH_COLOR_LEN, diff_ms, {
				if(phase == PHASE_BLU_0)
				{
					phase = PHASE_BLU_1;

					color_t c = {0, 0, 220};
					for(uint32_t i = LED_COUNT / 2; i < LED_COUNT; i++)
					{
						ws2812_set_led_recursive(i - POLICE_OFFSET, &c);
					}
				}
				else
				{
					phase = PHASE_BLU_0;
					ws2812_clear();
				}
				PERIODIC_DECIMATION_CNT(FLASH_CNT, diff_ms, { phase = PHASE_BLU_OFF; });
			});
		}

#pragma GCC diagnostic pop
	}
	break;

	case DM_SPARKLE:
	{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
		static bool is_lit = true;
		PERIODIC_DECIMATION_CNT(FLASH_COLOR_LEN, diff_ms, {
			if(is_lit)
			{
				is_lit = false;

				color_t c[] = {
					{230, 0, 0},
					{230, 210, 0},
					{0, 210, 0},
					{0, 210, 210},
					{0, 0, 210},
					{230, 0, 210},
				};

				for(uint32_t i = 0; i < LED_COUNT; i += 5)
				{
					ws2812_set_led(i++, &c[(int)param[show]]);
					ws2812_set_led(i++, &c[(int)param[show]]);
				}
			}
			else
			{
				ws2812_clear();
				is_lit = true;
			}
		});
#pragma GCC diagnostic pop
	}
	break;

	case DM_RIDE0:
	{
		if(accel_filt[0] > 3000 && fabsf(accel_filt[1] < 2500) && fabsf(accel_filt[2]) < 2500)
			dm_switch_mode_sparkle();
		else
		{
			static float hue = 0;
			hue += (float)diff_ms / 30.0f;
			if(hue >= 360.0f) hue = 0;

			color_t c = hsv2rgb(hue, 1.0, param[show]);

			if(accel_filt[1] < -500)
			{
				ws2812_clear();
				for(uint32_t i = 0; i < 15; i++)
					ws2812_set_led(i, &c);
				for(uint32_t i = 125; i < LED_COUNT; i++)
					ws2812_set_led(i, &c);

				int offsetp = map(accel_filt[1], -3000, -500, 15, 45);
				int offsetn = map(accel_filt[1], -3000, -500, 125, 95);

				for(int32_t i = 15; i < offsetp; i++)
					ws2812_set_led(i, &c);

				for(int32_t i = 125; i > offsetn; i--)
					ws2812_set_led(i, &c);
			}
			else if(accel_filt[1] > 500)
			{
				ws2812_clear();
				for(uint32_t i = 45; i < 95; i++)
					ws2812_set_led(i, &c);

				int offsetp = map(accel_filt[1], 500, 3000, 15, 45);
				int offsetn = map(accel_filt[1], 500, 3000, 125, 95);

				for(int32_t i = 95; i < offsetn; i++)
					ws2812_set_led(i, &c);

				for(int32_t i = 45; i > offsetp; i--)
					ws2812_set_led(i, &c);
			}
			else
			{
				for(uint32_t i = 0; i < LED_COUNT; i++)
					ws2812_set_led(i, &c);
			}
		}
	}
	break;

	case DM_RIDE_SPARKLE:
	{
		if(accel_filt[0] > 2000 && fabsf(accel_filt[1] < 2500) && fabsf(accel_filt[2]) < 2500)
			dm_switch_mode_sparkle();
		color_t x = {100, 0, 0};
		ws2812_set_led(LED_COUNT / 2, &x);
	}
	break;
	}
}