#include "adc.h"
#include "debounce.h"
#include "debug.h"
#include "dm.h"
#include "fatfs.h"
#include "file_operations.h"
#include "gps.h"
#include "main.h"
#include "mpu6050.h"
#include "phys_engine.h"
#include "ws2812.h"
#include <stdio.h>

extern UART_HandleTypeDef huart3;
extern IWDG_HandleTypeDef hiwdg;
extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;

static bool logging_enabled = false;
static button_ctrl_t btn[4];

void init(void)
{
	LED0_GPIO_Port->ODR |= LED0_Pin;
	LED1_GPIO_Port->ODR |= LED1_Pin;
	LED2_GPIO_Port->ODR |= LED2_Pin;
	LED3_GPIO_Port->ODR |= LED3_Pin;

	HAL_IWDG_Refresh(&hiwdg);

	__HAL_UART_ENABLE(&huart3);
	__HAL_I2C_ENABLE(&hi2c1);

	adc_init();
	ws2812_init();
	ws2812_push();

	debounce_init(&btn[0], 100);
	debounce_init(&btn[1], 100);
	debounce_init(&btn[2], 100);
	debounce_init(&btn[3], 100);
	btn[0].pressed = true;

	PWR_EN_GPIO_Port->ODR |= PWR_EN_Pin;

	while(HAL_GetTick() < 800)
		HAL_IWDG_Refresh(&hiwdg);

	mpu6050_init();
	file_op_init();
	dm_init();

	LED0_GPIO_Port->ODR &= (uint32_t)~LED0_Pin;
	LED1_GPIO_Port->ODR &= (uint32_t)~LED1_Pin;
	LED2_GPIO_Port->ODR &= (uint32_t)~LED2_Pin;
	LED3_GPIO_Port->ODR &= (uint32_t)~LED3_Pin;

	HAL_PWR_EnableBkUpAccess();
	uint32_t data = HAL_RTCEx_BKUPRead(&hrtc, 0);
	debug("BKUP reg #0: %d\n", data);
#define SYNC_VAR 2
	if(data != SYNC_VAR)
	{
		RTC_TimeTypeDef sTime = {0};
		RTC_DateTypeDef sDate = {0};

		sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		sTime.StoreOperation = RTC_STOREOPERATION_RESET;
		sDate.Year = 20;
		sDate.Month = RTC_MONTH_FEBRUARY;
		sDate.Date = 12;
		sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
		sTime.Hours = 0;
		sTime.Minutes = 10;
		sTime.Seconds = 0;
		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	}

	gps_init();

	HAL_IWDG_Refresh(&hiwdg);

	HAL_RTCEx_BKUPWrite(&hrtc, 0, SYNC_VAR);
	HAL_PWR_DisableBkUpAccess();
}

void loop(void)
{
	// time diff
	static uint32_t time_ms_prev = 0;
	uint32_t time_ms_now = HAL_GetTick();
	uint32_t diff_ms = time_ms_now < time_ms_prev
						   ? 0xFFFFFFFF + time_ms_now - time_ms_prev
						   : time_ms_now - time_ms_prev;
	time_ms_prev = time_ms_now;

	dm_poll(diff_ms);

	static uint32_t prev_tick = 0, prev_tick2 = 0;
	if(prev_tick < HAL_GetTick())
	{
		prev_tick = HAL_GetTick() + 1000;

		debug("A: %.0f %.0f %.0f | G: %0.f %0.f %0.f | Vbat: %0.2f | Temp: %0.1f\n",
			  accel_filt[0], accel_filt[1], accel_filt[2],
			  gyro_filt[0], gyro_filt[1], gyro_filt[2],
			  adc_get_v_bat(), adc_get_temp());
	}
	if(prev_tick2 < HAL_GetTick())
	{
		prev_tick2 = HAL_GetTick() + 5000;
		RTC_TimeTypeDef time = {0};
		RTC_DateTypeDef date = {0};
		HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

		debug("Date 20%02d / %02d / %02d Time %02d:%02d:%02d\n",
			  date.Year, date.Month, date.Date, time.Hours, time.Minutes, time.Seconds);
	}

	uint32_t led_period = pvt.fixType == 3 ? 1500 : 400;

	if(diff_ms > 0)
	{
		static uint32_t led0_cnt;
		led0_cnt += diff_ms;
		if(led0_cnt > led_period) led0_cnt = 0;

		bool lit = logging_enabled ? 0 : led0_cnt < 5;
		LED3_GPIO_Port->BSRR = LED3_Pin << (lit ? 0 : 16);

		lit = logging_enabled ? led0_cnt < 5 : 0;
		LED2_GPIO_Port->BSRR = LED2_Pin << (lit ? 0 : 16);

		bool litb = get_percent_battery() < 0.3 ? led0_cnt < 100 : 0;
		if(adc_get_v_bat() < 8.0) litb = true;
		LED0_GPIO_Port->BSRR = LED0_Pin << (litb ? 0 : 16);
	}

	if(diff_ms > 0)
	{
		if(mpu6050_read_acc_gyro(diff_ms))
		{
			if(logging_enabled)
			{
				// uint8_t buf[128] = "";

				// snprintf(buf, sizeof(buf), "%d %d %d %d %d %d %d\r\n",
				// 		 (int)HAL_GetTick(),
				// 		 accel_raw[0],
				// 		 accel_raw[1],
				// 		 accel_raw[2],
				// 		 gyro_raw[0],
				// 		 gyro_raw[1],
				// 		 gyro_raw[2]);

				uint8_t arr[256];
				uint32_t i = 0;
				uint32_t t = HAL_GetTick();
				struct
				{
					int16_t accel_raw[3], gyro_raw[3];
				} ag;
				ag.accel_raw[0] = accel_raw[0];
				ag.accel_raw[1] = accel_raw[1];
				ag.accel_raw[2] = accel_raw[2];
				ag.gyro_raw[0] = gyro_raw[0];
				ag.gyro_raw[1] = gyro_raw[1];
				ag.gyro_raw[2] = gyro_raw[2];

				memcpy(&arr[i], &t, sizeof(t));
				i += sizeof(t);

				memcpy(&arr[i], &ag, sizeof(ag));
				i += sizeof(ag);

				memcpy(&arr[i], &pvt, sizeof(pvt));
				i += sizeof(pvt);

				// size_t len = strlen(buf);

				// if(file_op_log(buf, strlen(buf)) == 0) logging_enabled = false;
				if(file_op_log(arr, i) == 0) logging_enabled = false;
			}
		}
	}

	ws2812_push();

	gps_poll();

	static bool prev_btn[4] = {1, 1, 1, 1};

	debounce_cb(&btn[0], BTN0_GPIO_Port->IDR & BTN0_Pin, diff_ms);
	debounce_cb(&btn[1], !(BTN1_GPIO_Port->IDR & BTN1_Pin), diff_ms);
	debounce_cb(&btn[2], !(BTN2_GPIO_Port->IDR & BTN2_Pin), diff_ms);
	debounce_cb(&btn[3], !(BTN3_GPIO_Port->IDR & BTN3_Pin), diff_ms);

	static bool must_turn_off = false;

	if(adc_get_v_bat() < 7.0f)
	{
		debug("[!] OFF, Vbat: %.3f V\n", adc_get_v_bat());
		PWR_EN_GPIO_Port->ODR &= (uint32_t)(~PWR_EN_Pin);
	}

	// button processor
	{
		static bool shift_selection = false;
		if(shift_selection == false)
		{
			if(btn[0].pressed_shot)
			{
				if(dm_get_mode() == DM_BAT)
				{
					file_op_log_disable();
					must_turn_off = true;
				}

				dm_switch_mode_vbat();
			}
			if(btn[0].unpressed_shot && must_turn_off)
			{
				PWR_EN_GPIO_Port->ODR &= (uint32_t)(~PWR_EN_Pin);
			}

			if(btn[2].pressed_shot) dm_switch_parameter();
			if(btn[3].pressed_shot) dm_switch_mode_next();
		}
		else
		{
			if(btn[0].pressed_shot)
			{
				shift_selection = false;

				if(logging_enabled)
				{
					file_op_log_disable();
					logging_enabled = false;
				}
				else if(file_op_log_enable())
				{
					logging_enabled = true;
				}
			}
		}

		if(btn[1].pressed_shot)
		{
			shift_selection = !shift_selection;
			if(shift_selection)
			{
				LED1_GPIO_Port->ODR |= LED1_Pin;
				LED2_GPIO_Port->ODR |= LED2_Pin;
			}
		}

		if(shift_selection == false)
		{
			LED1_GPIO_Port->ODR &= (uint32_t)~LED1_Pin;
			LED2_GPIO_Port->ODR &= (uint32_t)~LED2_Pin;
		}
	}

	for(uint8_t i = 0; i < 4; i++)
	{
		prev_btn[i] = btn[i].pressed;
	}

	HAL_IWDG_Refresh(&hiwdg);
}