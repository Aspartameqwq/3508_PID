/**
  ******************************************************************************
  * @file    DR16.c
  * @brief   DR16 DBUS receiver implementation
  *          - DMA1_Stream1 double-buffer circular reception on USART3
  *          - IDLE interrupt-based frame boundary detection
  *          - 18-byte DBUS frame parsing
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "DR16.h"
#include "usart.h"    /* huart3 */
#include "dma.h"      /* MX_DMA_Init */
#include "debug.h"    /* DR16 debug variables */

/* Private variables ---------------------------------------------------------*/
static volatile uint8_t sbus_rx_buffer[2][RC_FRAME_LENGTH];
RC_Ctl_t RC_CtrlData;

/**
  * @brief  Configure DMA for USART3 RX double-buffer circular mode
  *         and enable IDLE interrupt detection.
  *
  *         CubeMX generates basic USART3 + DMA (normal mode) init.
  *         This function upgrades DMA to double-buffer circular mode
  *         and links it to the USART3 IDLE interrupt for frame detection.
  */
void DR16_Init(void)
{
    DMA_Stream_TypeDef *dma_stream = huart3.hdmarx->Instance;

    /* ---------- Disable DMA stream before reconfiguration ---------- */
    CLEAR_BIT(dma_stream->CR, DMA_SxCR_EN);
    while ((dma_stream->CR & DMA_SxCR_EN) != 0U)
    {
        /* Wait until DMA is fully disabled */
    }

    /* ---------- Clear pending TC/HT/TE/DME/FE flags for DMA1 Stream 1 ---------- */
    DMA1->LIFCR = 0x07C0U;

    /* ---------- Set peripheral address (not set by CubeMX MSP init) ---------- */
    dma_stream->PAR = (uint32_t)&USART3->DR;

    /* ---------- Enable circular and double-buffer mode ---------- */
    SET_BIT(dma_stream->CR, DMA_SxCR_CIRC);  /* Circular mode */
    SET_BIT(dma_stream->CR, DMA_SxCR_DBM);   /* Double-buffer mode */

    /* ---------- Set both memory addresses ---------- */
    dma_stream->M0AR = (uint32_t)&sbus_rx_buffer[0][0];  /* Memory 0 */
    dma_stream->M1AR = (uint32_t)&sbus_rx_buffer[1][0];  /* Memory 1 */

    /* ---------- Start with Memory 0 (CT = 0) ---------- */
    CLEAR_BIT(dma_stream->CR, DMA_SxCR_CT);

    /* ---------- Set transfer count and enable ---------- */
    dma_stream->NDTR = RC_FRAME_LENGTH;
    SET_BIT(dma_stream->CR, DMA_SxCR_EN);

    /* ---------- Enable USART3 DMA RX request ---------- */
    SET_BIT(USART3->CR3, USART_CR3_DMAR);

    /* ---------- Enable USART3 IDLE interrupt ---------- */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    /* ---------- 初始化 RC_CtrlData 通道为中位值，防止上电未收到遥控器信号时电机暴走 ---------- */
    RC_CtrlData.rc.ch0 = RC_CH_VALUE_OFFSET;   /* 1024 */
    RC_CtrlData.rc.ch1 = RC_CH_VALUE_OFFSET;
    RC_CtrlData.rc.ch2 = RC_CH_VALUE_OFFSET;
    RC_CtrlData.rc.ch3 = RC_CH_VALUE_OFFSET;
    RC_CtrlData.rc.s1  = RC_SW_MID;            /* 3 = MID */
    RC_CtrlData.rc.s2  = RC_SW_MID;
}

/**
  * @brief  Handle USART3 IDLE interrupt for DR16 frame reception.
  *
  *         USART3 IDLE fires when RX line stays high for 1 byte time
  *         after the last stop bit — this IS the DBUS frame boundary.
  *
  *         DMA Double-Buffer behaviour in CIRC+DBM mode:
  *           - NDTR=0 → hardware auto-reloads NDTR & auto-toggles CT
  *           - CT points to the buffer DMA is CURRENTLY targeting
  *           - The OTHER buffer holds the just-completed frame
  *
  *         Key insight (the bug that was fixed):
  *           CT=0 → DMA just switched TO M0 → M1 has the complete frame
  *           CT=1 → DMA just switched TO M1 → M0 has the complete frame
  *
  *         No DMA register manipulation is needed in the handler:
  *         circular mode auto-reloads NDTR and auto-toggles CT —
  *         DMA just keeps running for the next frame.
  */
void DR16_USART3_IDLE_IRQHandler(void)
{
    DMA_Stream_TypeDef *dma_stream = huart3.hdmarx->Instance;

    /* --- Debug: snapshot state before any modification --- */
    debug_dr16_idle_count++;
    debug_dr16_uart_sr = USART3->SR;
    uint8_t  ct  = (dma_stream->CR & DMA_SxCR_CT) ? 1U : 0U;
    uint16_t ndtr = (uint16_t)dma_stream->NDTR;
    debug_dr16_ct   = ct;
    debug_dr16_ndtr = ndtr;

    /*
     * CT points to the buffer DMA is CURRENTLY targeting (next frame).
     * The OTHER buffer was just completed — that's the one to process.
     *   CT=0 → DMA targeting M0 → M1 was just filled → process M1
     *   CT=1 → DMA targeting M1 → M0 was just filled → process M0
     */
    if (ct == 0U)
    {
        for (uint8_t i = 0U; i < RC_FRAME_LENGTH; i++)
            debug_dr16_raw[i] = sbus_rx_buffer[1][i];

        RemoteDataProcess((uint8_t *)sbus_rx_buffer[1]);
    }
    else
    {
        for (uint8_t i = 0U; i < RC_FRAME_LENGTH; i++)
            debug_dr16_raw[i] = sbus_rx_buffer[0][i];

        RemoteDataProcess((uint8_t *)sbus_rx_buffer[0]);
    }

    /* DMA circular double-buffer auto-handles NDTR reload & CT toggle.
     * No need to disable/re-enable — let DMA keep running. */
}

/**
  * @brief  Parse 18-byte DBUS raw frame into RC_CtrlData.
  *
  *         DBUS frame bit layout (DJI DR16 receiver, 144 bits = 18 bytes):
  *           Field       Offset(bit)  Len(bit)   Byte mapping
  *           ─────────   ───────────  ────────   ─────────────────────────────
  *           ch0             0          11       Byte[0][7:0] | Byte[1][2:0]<<8
  *           ch1            11          11       Byte[1][7:3] | Byte[2][5:0]<<5
  *           ch2            22          11       Byte[2][7:6] | Byte[3][7:0]<<2 | Byte[4][0]<<10
  *           ch3            33          11       Byte[4][7:1] | Byte[5][3:0]<<7
  *           S1             44           2       Byte[5][5:4]
  *           S2             46           2       Byte[5][7:6]
  *           Mouse X        48          16       Byte[6] | Byte[7]<<8  (int16)
  *           Mouse Y        64          16       Byte[8] | Byte[9]<<8  (int16)
  *           Mouse Z        80          16       Byte[10] | Byte[11]<<8 (int16)
  *           Mouse Left     96           8       Byte[12]
  *           Mouse Right   104           8       Byte[13]
  *           Keys          112          16       Byte[14] | Byte[15]<<8 (uint16)
  *           Reserved      128          16       Byte[16] | Byte[17]<<8
  *
  * @param  pData: pointer to 18-byte raw buffer
  */
void RemoteDataProcess(uint8_t *pData)
{
    if (pData == NULL)
    {
        return;
    }

    debug_dr16_frame_count++;

    /* 更新最后一次收到有效帧的时间戳，供 main.c 判断信号是否丢失 */
    debug_dr16_last_frame_tick = HAL_GetTick();

    /* --- RC channels: 11-bit values (0 ~ 2047) ---
     *
     * 位域布局（DBUS 18 字节帧，字节序 LSB-first）：
     *   ch0: Byte[0][7:0]  | Byte[1][2:0] << 8  → frame bit  0 ~ 10
     *   ch1: Byte[1][7:3]  | Byte[2][5:0] << 5  → frame bit 11 ~ 21
     *   ch2: Byte[2][7:6]  | Byte[3][7:0] << 2 | Byte[4][0] << 10
     *                                             → frame bit 22 ~ 32
     *   ch3: Byte[4][7:1]  | Byte[5][3:0] << 7  → frame bit 33 ~ 43
     *
     * 掩码 0x07FF 截取低 11 位，消除相邻通道的残留位。 */
    RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
    RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
                          ((int16_t)pData[4] << 10)) & 0x07FF;
    RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) & 0x07FF;

    /* --- RC switches: 2-bit values ---
     *   S1: Byte[5][5:4]  → frame bit 44 ~ 45
     *   S2: Byte[5][7:6]  → frame bit 46 ~ 47 */
    RC_CtrlData.rc.s1 = (pData[5] >> 4) & 0x03U;
    RC_CtrlData.rc.s2 = (pData[5] >> 6) & 0x03U;

    /* --- Mouse values --- */
    RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
    RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);
    RC_CtrlData.mouse.press_l = pData[12];
    RC_CtrlData.mouse.press_r = pData[13];

    /* --- Keyboard keys (16-bit, Byte[14] Byte[15]) --- */
    RC_CtrlData.key.v = ((uint16_t)pData[14] | ((uint16_t)pData[15] << 8));
}
