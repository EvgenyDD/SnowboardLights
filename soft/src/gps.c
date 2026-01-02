#include "gps.h"
#include "main.h"
#include <stdbool.h>
#include <string.h>

// UBLOX NEO-M8N

extern UART_HandleTypeDef huart1;

NAV_PVT_t pvt;

static volatile uint8_t buf_rx[256];
static volatile uint32_t cnt_rx = 0;
static volatile uint16_t pkt_rx_len = 0;
static volatile bool pkt_rxed = false;

static uint8_t pkt_tx[64] = {SYNC_CHAR_1, SYNC_CHAR_2};

static void checksum(volatile uint8_t *data, uint32_t len, uint8_t *ck_a, uint8_t *ck_b)
{
	uint8_t CK_A = 0, CK_B = 0;
	for(uint32_t i = 0; i < len; i++)
	{
		CK_A = CK_A + data[i];
		CK_B = CK_B + CK_A;
	}
	*ck_a = CK_A;
	*ck_b = CK_B;
}

static uint8_t checksum_check(volatile uint8_t *data, uint32_t len)
{
	uint8_t CK_A = 0, CK_B = 0;
	for(uint32_t i = 2; i < len - 2; i++)
	{
		CK_A = CK_A + data[i];
		CK_B = CK_B + CK_A;
	}
	return ((CK_A == data[len - 2]) && (CK_B == data[len - 1]));
}

static void send_pkt(uint8_t CLSS, uint8_t ID, const uint8_t *pl, uint16_t size)
{
	uint32_t i = 2;
	pkt_tx[i++] = CLSS;
	pkt_tx[i++] = ID;
	memcpy(&pkt_tx[i], &size, 2);
	i += 2;
	memcpy(&pkt_tx[i], pl, size);
	i += size;
	checksum(&pkt_tx[2], i - 2, &pkt_tx[i], &pkt_tx[i + 1]);
	i += 2;
	HAL_UART_Transmit(&huart1, pkt_tx, i, 500);
}

void gps_init(void)
{
	__HAL_UART_ENABLE(&huart1);

	uint8_t dummy = 0;
	HAL_UART_Transmit(&huart1, &dummy, 1, 10);
	HAL_Delay(1);

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

	if(0) // first time config
	{
		// UART CFG
		const uint32_t baudrate = 115200;
		uint8_t uart_cfg[20] = {0x01, 0, 0, 0, 0xD0, 0x08, 0, 0, 0, 0, 0, 0, 0x01, 0, 0x01, 0, 0, 0, 0, 0};
		memcpy(&uart_cfg[8], &baudrate, 4);
		send_pkt(CLSS_CFG, ID_PRT, uart_cfg, sizeof(uart_cfg));
		HAL_Delay(150);

		// !!! switch UART to 115200
		huart1.Init.BaudRate = 115200;
		if(HAL_UART_Init(&huart1) != HAL_OK)
		{
			Error_Handler();
		}

		uint16_t meas_rate = 200; // ms
		uint8_t nav_meas_rate[6] = {0, 0, 0x01, 0x00, 0x01, 0};
		memcpy(&nav_meas_rate[0], &meas_rate, 2);
		send_pkt(CLSS_CFG, ID_RATE, nav_meas_rate, sizeof(nav_meas_rate));
		HAL_Delay(150);

		const uint8_t clr_sav_ld[13] = {
			0, 0, 0, 0,
			0xFF, 0xFF, 0, 0,
			0, 0, 0, 0,
			0x17};
		send_pkt(CLSS_CFG, ID_CFG, clr_sav_ld, sizeof(clr_sav_ld));
		HAL_Delay(150);

		send_pkt(CLSS_CFG, ID_MSG, (uint8_t[]){MSG_CLSS_NAV, MSG_ID_NAV_POSLLH, 0, 0, 0, 0, 0, 0}, 8); // Set Message Rate(s)
		HAL_Delay(150);

		send_pkt(CLSS_CFG, ID_MSG, (uint8_t[]){MSG_CLSS_NAV, MSG_ID_NAV_PVT, 0, 1, 0, 0, 0, 0}, 8); // Set Message Rate(s)
		HAL_Delay(150);
	}
	else // generic OP
	{
		// !!! switch UART to 115200
		huart1.Init.BaudRate = 115200;
		if(HAL_UART_Init(&huart1) != HAL_OK)
		{
			Error_Handler();
		}

		send_pkt(CLSS_CFG, ID_MSG, (uint8_t[]){MSG_CLSS_NAV, MSG_ID_NAV_POSLLH, 0, 0, 0, 0, 0, 0}, 8); // Set Message Rate(s)
		HAL_Delay(150);

		send_pkt(CLSS_CFG, ID_MSG, (uint8_t[]){MSG_CLSS_NAV, MSG_ID_NAV_PVT, 0, 1, 0, 0, 0, 0}, 8); // Set Message Rate(s)
		HAL_Delay(150);
	}
}

static void gps_parse(char c)
{
	switch(cnt_rx)
	{
	case 0:
		if(c == SYNC_CHAR_1)
		{
			buf_rx[cnt_rx] = c;
			cnt_rx++;
		}
		break;

	case 1:
		if(c != SYNC_CHAR_2)
		{
			cnt_rx = 0;
			break;
		}
		buf_rx[cnt_rx] = c;
		cnt_rx++;
		break;

	default:
		buf_rx[cnt_rx] = c;
		cnt_rx++;
		if(cnt_rx >= sizeof(buf_rx)) cnt_rx = 0;
		if(cnt_rx == 6)
		{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
			memcpy(&pkt_rx_len, &buf_rx[4], 2);
#pragma GCC diagnostic pop
		}
		if(cnt_rx == pkt_rx_len + 8UL)
		{
			pkt_rxed = true;
			cnt_rx = 0;
		}
		break;
	}
}

void gps_poll(void)
{
	if(pkt_rxed)
	{
		pkt_rxed = false;
		if(checksum_check(buf_rx, pkt_rx_len + 8))
		{
			const uint8_t clss = buf_rx[2];
			const uint8_t id = buf_rx[3];
			if(pkt_rx_len == sizeof(pvt) && clss == 1 && id == 7)
			{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
				memcpy(&pvt, &buf_rx[6], sizeof(pvt));
#pragma GCC diagnostic pop
			}
		}
	}
}

void USART1_IRQHandler(void)
{
	// HAL_UART_IRQHandler(&huart1);
	if(USART1->SR & USART_SR_RXNE)
	{
		volatile uint8_t ch = USART1->DR & (uint16_t)0x00FF;
		gps_parse(ch);
		USART1->SR = ~(USART_SR_RXNE);
	}
	NVIC_ClearPendingIRQ(USART1_IRQn);
}