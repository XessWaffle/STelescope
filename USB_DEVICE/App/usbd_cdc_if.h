/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.h
  * @version        : v2.0_Cube
  * @brief          : Header for usbd_cdc_if.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief For Usb device.
  * @{
  */

/** @defgroup USBD_CDC_IF USBD_CDC_IF
  * @brief Usb VCP device module
  * @{
  */

/** @defgroup USBD_CDC_IF_Exported_Defines USBD_CDC_IF_Exported_Defines
  * @brief Defines.
  * @{
  */
/* Define size for the receive and transmit buffer over CDC */
#define APP_RX_DATA_SIZE  1024
#define APP_TX_DATA_SIZE  1024
/* USER CODE BEGIN EXPORTED_DEFINES */
#define PACKET_METADATA_SIZE 4

#define ORIENTATION_PACKET_SIZE 56
#define ORIENTATION_PACKET_ID 0xDEAD

#define CAMERA_PACKET_METADATA_SIZE 4 
#define CAMERA_PACKET_ID 0xBEEF
#define CAMERA_PIPE_BUFFER_LENGTH APP_TX_DATA_SIZE - CAMERA_PACKET_METADATA_SIZE - PACKET_METADATA_SIZE
/* USER CODE END EXPORTED_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Types USBD_CDC_IF_Exported_Types
  * @brief Types.
  * @{
  */

/* USER CODE BEGIN EXPORTED_TYPES */

typedef enum
{
  CMD_SET_R_RATE,
  CMD_SET_Y_RATE,
  CMD_SET_P_RATE,
  CMD_SET_R_POS,
  CMD_SET_Y_POS,
  CMD_SET_P_POS,
  CMD_SET_STATE,
  CMD_RESET_POS,
  CMD_RESET_DES_POS,
  CMD_HOME, /* Deprecated */
  CMD_MICST_MODE, /* Change the microstep mode (mode given in info)*/
  CMD_CAPTURE,
  CMD_EM_STOP,
  CMD_STOP,
  CMD_ACK,
  CMD_INVALID = 0xFF
} telescope_command_e;

typedef struct
{
  telescope_command_e cmd: 8;
  uint32_t info:24;
} telescope_command_s;

typedef struct
{
  union
  {
    struct
    {
      /* Packet Metadata */
      uint16_t packet_id;
      uint16_t size;

      /* Packed Accelerometer Data */
      int16_t out_x_g_raw;
      int16_t out_y_g_raw;
      int16_t out_z_g_raw;
      int16_t out_x_a_raw;
      int16_t out_y_a_raw;
      int16_t out_z_a_raw;

      /* Packed Compass Data*/
      int16_t out_x_raw_offboard;
      int16_t out_y_raw_offboard;
      int16_t out_z_raw_offboard;
      int16_t out_x_raw_onboard;
      int16_t out_y_raw_onboard;
      int16_t out_z_raw_onboard;
      uint32_t microstep_mode: 8;
      uint32_t _res1:24;

      /* Packed stepper data */
      uint32_t yaw_position;
      uint32_t roll_position;
      uint32_t pitch_position;
      int32_t yaw_rate;
      int32_t roll_rate;
      int32_t pitch_rate;
    };
    uint8_t raw_data[ORIENTATION_PACKET_SIZE];
  };
} orientation_packet_s;

typedef struct
{
  union
  {
    struct
    {
      /* Packet Metadata */
      uint16_t packet_id;
      uint16_t size;

      /* Camera Packet Metadata*/
      uint32_t last_packet;

      /* Image Buffer Data */
      uint8_t buff[CAMERA_PIPE_BUFFER_LENGTH];
    };
    uint8_t raw_data[PACKET_METADATA_SIZE + CAMERA_PACKET_METADATA_SIZE + CAMERA_PIPE_BUFFER_LENGTH];
  };
} camera_data_packet_s;
/* USER CODE END EXPORTED_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Macros USBD_CDC_IF_Exported_Macros
  * @brief Aliases.
  * @{
  */

/* USER CODE BEGIN EXPORTED_MACRO */

/* USER CODE END EXPORTED_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

/** CDC Interface callback. */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_FunctionsPrototype USBD_CDC_IF_Exported_FunctionsPrototype
  * @brief Public functions declaration.
  * @{
  */

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

/* USER CODE BEGIN EXPORTED_FUNCTIONS */
telescope_command_s* get_next_command();
/* USER CODE END EXPORTED_FUNCTIONS */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */

