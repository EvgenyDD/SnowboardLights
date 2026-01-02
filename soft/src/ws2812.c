#include "ws2812.h"
#include "main.h"
#include "math_const.h"
#include <math.h>
#include <stdbool.h>

extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_tim1_ch1;

const color_t black = {0, 0, 0};
const color_t red = {255, 0, 0};
const color_t green = {0, 255, 0};
const color_t blue = {0, 0, 255};
const color_t white = {255, 255, 255};

static bool transfer_enabled = false;

#define DELAY_LEN 64
#define PREFIX 64
#define ARRAY_LEN PREFIX + LED_COUNT * 24 + DELAY_LEN

#define TIM_COMPARE_HIGH 22
#define TIM_COMPARE_LOW 8

uint16_t buffer_dma[ARRAY_LEN];
color_t leds[LED_COUNT];

static const uint8_t gamma8[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2,
	2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5,
	5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10,
	10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
	17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
	25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
	37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
	51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
	69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
	90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
	115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
	144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
	177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
	215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255, 0, 0, 0};

void blend(const uint8_t *in_a, const uint8_t *in_b, uint8_t *out, float amount)
{
	float r, g, b;

	r = ((float)in_b[0] * amount) + ((float)in_a[0] * (1.0 - amount));
	g = ((float)in_b[1] * amount) + ((float)in_a[1] * (1.0 - amount));
	b = ((float)in_b[2] * amount) + ((float)in_a[2] * (1.0 - amount));

	out[0] = (r > 255.0) ? 255.0 : (r < 0.0) ? 0.0
											 : r;
	out[1] = (g > 255.0) ? 255.0 : (g < 0.0) ? 0.0
											 : g;
	out[2] = (b > 255.0) ? 255.0 : (b < 0.0) ? 0.0
											 : b;
}

/**
 * @brief Interval hit linear function
 *          /\
 *         /  \
 * ......./    \...... output
 * 
 * @param value positive or zero value
 * @param middle positive value
 * @param half_sector positive value
 * @param range positive value
 * @return float 
 */
float interval_hit_dim(int32_t value, int32_t middle, int32_t half_sector, int32_t range)
{
	const int32_t max = middle + half_sector;
	const int32_t min = middle - half_sector;

	if(max >= range)
	{
		if(value >= min && value < range)
			return 1.0f - fabsf((float)(value - middle) / (float)half_sector);
		else if(value + range < max)
			return 1.0f - fabsf((float)(value - middle + range) / (float)half_sector);
		else
			return 0;
	}
	else if(min < 0)
	{
		if(value >= 0 && value < max)
			return 1.0f - fabsf((float)(value - middle) / (float)half_sector);
		else if(value >= min + range)
			return 1.0f - fabsf((float)(value - middle - range) / (float)half_sector);
		else
			return 0;
	}
	else
	{
		if(value >= min && value < max)
			return 1.0f - fabsf((float)(value - middle) / (float)half_sector);
		else
			return 0;
	}
}

bool interval_hit_bool(int32_t value, int32_t middle, int32_t half_sector, int32_t range)
{
	const int32_t max = middle + half_sector;
	const int32_t min = middle - half_sector;

	if(max >= range)
	{
		if(value >= min && value < range)
			return true;
		else if(value + range < max)
			return true;
		else
			return false;
	}
	else if(min < 0)
	{
		if(value >= 0 && value < max)
			return true;
		else if(value >= min + range)
			return true;
		else
			return false;
	}
	else
	{
		if(value >= min && value < max)
			return true;
		else
			return false;
	}
}

void ws2812_init(void)
{
	memset(buffer_dma, 0, sizeof(buffer_dma));
	memset(leds, 0, sizeof(leds));
}

void ws2812_push(void)
{
	if(transfer_enabled) return;

	transfer_enabled = true;

	bool is_on = false;
	uint32_t iterator = PREFIX;
	for(uint32_t led = 0; led < LED_COUNT; led++)
	{
		if(leds[led].r ||
		   leds[led].g ||
		   leds[led].b) is_on = true;

		for(uint32_t i = 0; i < 8; i++)
		{
			buffer_dma[iterator] = gamma8[leds[led].r] & (1U << (7 - i)) ? TIM_COMPARE_HIGH : TIM_COMPARE_LOW;
			iterator++;
		}

		for(uint32_t i = 0; i < 8; i++)
		{
			buffer_dma[iterator] = gamma8[leds[led].g] & (1U << (7 - i)) ? TIM_COMPARE_HIGH : TIM_COMPARE_LOW;
			iterator++;
		}

		for(uint32_t i = 0; i < 8; i++)
		{
			buffer_dma[iterator] = gamma8[leds[led].b] & (1U << (7 - i)) ? TIM_COMPARE_HIGH : TIM_COMPARE_LOW;
			iterator++;
		}
	}
	HAL_GPIO_WritePin(WS2812_ENABLE_GPIO_Port, WS2812_ENABLE_Pin, is_on);
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)&buffer_dma, ARRAY_LEN);
}

HAL_StatusTypeDef _HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	/* Check the parameters */
	assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));

	switch(Channel)
	{
	case TIM_CHANNEL_1:
	{
		/* Disable the TIM Capture/Compare 1 DMA request */
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
		(void)HAL_DMA_Abort_IT(htim->hdma[TIM_DMA_ID_CC1]);
		break;
	}

	case TIM_CHANNEL_2:
	{
		/* Disable the TIM Capture/Compare 2 DMA request */
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
		(void)HAL_DMA_Abort_IT(htim->hdma[TIM_DMA_ID_CC2]);
		break;
	}

	case TIM_CHANNEL_3:
	{
		/* Disable the TIM Capture/Compare 3 DMA request */
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
		(void)HAL_DMA_Abort_IT(htim->hdma[TIM_DMA_ID_CC3]);
		break;
	}

	case TIM_CHANNEL_4:
	{
		/* Disable the TIM Capture/Compare 4 interrupt */
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
		(void)HAL_DMA_Abort_IT(htim->hdma[TIM_DMA_ID_CC4]);
		break;
	}

	default:
		break;
	}

	/* Disable the Capture compare channel */
	//   TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_DISABLE);

	if(IS_TIM_BREAK_INSTANCE(htim->Instance) != RESET)
	{
		/* Disable the Main Output */
		__HAL_TIM_MOE_DISABLE(htim);
	}

	/* Disable the Peripheral */
	//   __HAL_TIM_DISABLE(htim);

	/* Change the htim state */
	htim->State = HAL_TIM_STATE_READY;

	/* Return function status */
	return HAL_OK;
}

void ws2812_terminate(void)
{
	_HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	// TIM1->CCR1 = 0;
	// __HAL_TIM_DISABLE(&htim1);
	transfer_enabled = false;
}

void ws2812_clear(void) { memset(leds, 0, sizeof(leds)); }

void ws2812_set_led(uint16_t id, const color_t *color)
{
	if(id >= LED_COUNT) return;

	memcpy(&leds[id], color, sizeof(color_t));
}

void ws2812_set_led_all(const color_t *color)
{
	for(uint32_t i = 0; i < LED_COUNT; i++)
		memcpy(&leds[i], color, sizeof(color_t));
}

void ws2812_set_led_recursive(int16_t id, const color_t *color)
{
	id %= LED_COUNT;
	if(id < 0) id += LED_COUNT;

	memcpy(&leds[id], color, sizeof(color_t));
}

/**
 * h [0;360]
 * s [0;1]
 * v [0;255]
 */
color_t hsv2rgb(float h, float s, float v)
{
	float p, q, t, ff;
	color_t out;

	float hh = h;
	if(hh >= 360.0f || hh < 0) hh = 0;
	hh /= 60.0f;
	int i = (int)hh;
	ff = hh - i;
	p = v * (1.0f - s);
	q = v * (1.0f - (s * ff));
	t = v * (1.0f - (s * (1.0f - ff)));

	switch(i)
	{
	case 0:
		out.r = v;
		out.g = t;
		out.b = p;
		break;
	case 1:
		out.r = q;
		out.g = v;
		out.b = p;
		break;
	case 2:
		out.r = p;
		out.g = v;
		out.b = t;
		break;

	case 3:
		out.r = p;
		out.g = q;
		out.b = v;
		break;
	case 4:
		out.r = t;
		out.g = p;
		out.b = v;
		break;
	case 5:
	default:
		out.r = v;
		out.g = p;
		out.b = q;
		break;
	}
	return out;
}

void ws2812_set_angle(float angle, float w, uint8_t brightness, uint8_t led_count)
{
	const float offset = 0;

	float normed = angle + offset;
	normed = angle_norm(normed);

	// static float prev_start = -999;
	int start = (float)LED_COUNT * normed * ONE_DIV_2PI;

	// if(prev_start != start)
	{
		float w_abs = fabsf(w);
		float w_trunc = w_abs > 120.0f ? 120.0f : w_abs;
		color_t light_color = hsv2rgb(240 + w_trunc, 1.0f, brightness);

		for(uint32_t i = 0; i < LED_COUNT; i++)
		{
			ws2812_set_led(i, interval_hit_bool((int32_t)i, start, led_count / 2, LED_COUNT)
			                      ? &light_color
			                      : &black);
		}
		// prev_start = start;
	}
}

color_t color_dim(const color_t *c, float dim)
{
	if(dim > 1.0f) dim = 1.0f;
	if(dim < 0) dim = 0;
	color_t out;
	out.r = c->r * dim;
	out.g = c->g * dim;
	out.b = c->b * dim;
	return out;
}

void normalize_hue(float *hue)
{
	*hue = fmodf(*hue, 360.0f);
	if(*hue < 0) *hue += 360.0f;
}