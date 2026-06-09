/**
  ******************************************************************************
  * @file    DR16.h
  * @brief   DR16 receiver via DBUS protocol
  *          - USART3, 100Kbps, 9-bit (8 data + even parity), 1 stop bit
  *          - DMA double-buffer circular reception
  *          - IDLE interrupt for frame boundary detection
  ******************************************************************************
  */

#ifndef __DR16_H__
#define __DR16_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ----------------------- DBUS Frame Length --------------------------------- */
#define RC_FRAME_LENGTH 18U

/* ----------------------- RC Channel Value Range ---------------------------- */
#define RC_CH_VALUE_MIN    ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX    ((uint16_t)1684)

/* ----------------------- RC Switch Positions ------------------------------- */
#define RC_SW_UP   ((uint8_t)1)
#define RC_SW_MID  ((uint8_t)3)
#define RC_SW_DOWN ((uint8_t)2)

/* ----------------------- PC Keyboard Key Mapping --------------------------- */
#define KEY_PRESSED_OFFSET_W     ((uint16_t)(0x01 << 0))
#define KEY_PRESSED_OFFSET_S     ((uint16_t)(0x01 << 1))
#define KEY_PRESSED_OFFSET_A     ((uint16_t)(0x01 << 2))
#define KEY_PRESSED_OFFSET_D     ((uint16_t)(0x01 << 3))
#define KEY_PRESSED_OFFSET_Q     ((uint16_t)(0x01 << 4))
#define KEY_PRESSED_OFFSET_E     ((uint16_t)(0x01 << 5))
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)(0x01 << 6))
#define KEY_PRESSED_OFFSET_CTRL  ((uint16_t)(0x01 << 7))

/* ----------------------- Data Structure ------------------------------------ */
#pragma pack(1)
typedef struct
{
    struct
    {
        uint16_t ch0;    /* RC channel 0 (right horizontal, 364~1684) */
        uint16_t ch1;    /* RC channel 1 (right vertical,   364~1684) */
        uint16_t ch2;    /* RC channel 2 (left  vertical,   364~1684) */
        uint16_t ch3;    /* RC channel 3 (left  horizontal, 364~1684) */
        uint8_t  s1;     /* Switch 1: 1=UP, 3=MID, 2=DOWN */
        uint8_t  s2;     /* Switch 2: 1=UP, 3=MID, 2=DOWN */
    } rc;
    struct
    {
        int16_t  x;      /* Mouse X-axis velocity */
        int16_t  y;      /* Mouse Y-axis velocity */
        int16_t  z;      /* Mouse Z-axis (scroll wheel) */
        uint8_t  press_l; /* Left button press state */
        uint8_t  press_r; /* Right button press state */
    } mouse;
    struct
    {
        uint16_t v;      /* Keyboard key bitmask */
    } key;
} RC_Ctl_t;
#pragma pack()

/* ----------------------- External Variable --------------------------------- */
extern RC_Ctl_t RC_CtrlData;

/* ----------------------- API Functions ------------------------------------- */

/**
  * @brief  Initialize DR16 receiver
  *         - Reconfigures DMA1_Stream1 for double-buffer circular mode
  *         - Enables USART3 IDLE interrupt
  *         - Starts DMA reception
  * @note   Must be called after MX_USART3_UART_Init() and MX_DMA_Init()
  */
void DR16_Init(void);

/**
  * @brief  USART3 IDLE interrupt handler for DR16
  *         - Detects DMA double-buffer target
  *         - Switches buffer and processes received frame
  * @note   Called from USART3_IRQHandler in stm32f4xx_it.c
  */
void DR16_USART3_IDLE_IRQHandler(void);

/**
  * @brief  Parse DBUS raw frame data into RC_CtrlData
  * @param  pData: pointer to 18-byte raw frame buffer
  * @note   See DBUS protocol specification for bit layout
  */
void RemoteDataProcess(uint8_t *pData);

/**
  * @brief  Get pointer to parsed RC control data
  * @retval Pointer to global RC_CtrlData structure
  */
static inline RC_Ctl_t* DR16_GetData(void)
{
    return &RC_CtrlData;
}

#ifdef __cplusplus
}
#endif

#endif /* __DR16_H__ */
