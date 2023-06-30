/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
*
* Defines peripherals available for use on CYW943012BTEVK-01
*
*/

#pragma once

/** \addtogroup Platform config - Peripherals pin configuration
*   \ingroup HardwareDrivers
*/
/*! @{ */

/******************************************************
 *                   Enumerations
 ******************************************************/

#include "wiced_bt_types.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_printf.h"

typedef enum
{
    WICED_PLATFORM_PTU_SEL_USB         = 0x01,
    WICED_PLATFORM_PTU_SEL_UART        = 0x02,
    WICED_PLATFORM_PTU_SEL_I2C_SLAVE   = 0x04,
    WICED_PLATFORM_PTU_SEL_SPIFFY      = 0x20,
    WICED_PLATFORM_PTU_SEL_UNKNOWN     = 0xff
} wiced_platform_ptu_sel_t;


/**
 * wiced_platform_transport_rx_data_handler
 *
 * Callback registered by the application to receive the incoming HCI UART data.
 *
 * @param[in] op_code   : operation code for the incoming HCI data (refer to hci_control_api.h)
 * @param[in] p_data    : Pointer to the received data for the op_code
 * @param[in] data_len  : length of the data pointed to by p_data in bytes
 */
typedef void (wiced_platform_transport_rx_data_handler)(uint16_t op_code, uint8_t *p_data, uint32_t data_len);

//! Possible interrupt configuration for platform buttons
typedef enum
{
    WICED_PLATFORM_BUTTON_BOTH_EDGE = WICED_GPIO_EN_INT_BOTH_EDGE,   //!< indicates that app. should receive interrupt on both edges
    WICED_PLATFORM_BUTTON_RISING_EDGE = WICED_GPIO_EN_INT_RISING_EDGE, //!< indicates that app. should receive interrupt only for rising edge
    WICED_PLATFORM_BUTTON_FALLING_EDGE = WICED_GPIO_EN_INT_FALLING_EDGE,//!< indicates that app. should receive interrupt only for falling edge
} wiced_platform_button_interrupt_edge_t;

typedef struct
{
    wiced_bt_gpio_numbers_t* gpio; /**< WICED GPIO pin */
    uint32_t config; /**< configuration like GPIO_PULL_DOWN,GPIO_PULL_UP etc., */
    uint32_t default_state; /**< GPIO_PIN_OUTPUT_HIGH/GPIO_PIN_OUTPUT_LOW */
}
wiced_platform_led_config_t;

typedef enum
{
    WICED_PLATFORM_LED_1,  //!< LED 1 (optional, WICED_P27)
    WICED_PLATFORM_LED_2,  //!< LED 2 (default, WICED_P26)
    WICED_PLATFORM_LED_MAX //!< Max LED for error check
} wiced_platform_led_number_t;



#if USE_DESIGN_MODUS

#include "wiced.h"
#include "wiced_hal_gpio.h"

//! Number of LEDs available on the platform.

//! Number of Buttons available on the platform.
typedef enum
{
    WICED_PLATFORM_BUTTON_1,  //!< BUTTON 1
    WICED_PLATFORM_BUTTON_2,  //!< BUTTON 1
    WICED_PLATFORM_BUTTON_3,  //!< BUTTON 1
    WICED_PLATFORM_BUTTON_4,  //!< BUTTON 1
    WICED_PLATFORM_BUTTON_MAX //!< Max button for error check
} wiced_platform_button_number_t;

#define WICED_PLATFORM_BUTTON_MAX_DEF     4   //define for platform_gpio_buttons

//! Number of GPIO available on the platform.
// Configurable via SuperMux
typedef enum
{
    WICED_PLATFORM_GPIO_1,  //!< GPIO 1
    WICED_PLATFORM_GPIO_2,  //!< GPIO 2
    WICED_PLATFORM_GPIO_3,  //!< GPIO 3
    WICED_PLATFORM_GPIO_4,  //!< GPIO 4
    WICED_PLATFORM_GPIO_5,  //!< GPIO 5
    WICED_PLATFORM_GPIO_6,  //!< GPIO 6
    WICED_PLATFORM_GPIO_7,  //!< GPIO 7
    WICED_PLATFORM_GPIO_8,  //!< GPIO 8
    WICED_PLATFORM_GPIO_9,  //!< GPIO 9
    WICED_PLATFORM_GPIO_10,  //!< GPIO 10
    WICED_PLATFORM_GPIO_11,  //!< GPIO 11
    WICED_PLATFORM_GPIO_12,  //!< GPIO 12
    WICED_PLATFORM_GPIO_13,  //!< GPIO 13
    WICED_PLATFORM_GPIO_14,  //!< GPIO 14
    WICED_PLATFORM_GPIO_15,  //!< GPIO 15
    WICED_PLATFORM_GPIO_16,  //!< GPIO 16
    WICED_PLATFORM_GPIO_17,  //!< GPIO 17
    WICED_PLATFORM_GPIO_18,  //!< GPIO 18
    WICED_PLATFORM_GPIO_19,  //!< GPIO 19
    WICED_PLATFORM_GPIO_20,  //!< GPIO 20
    WICED_PLATFORM_GPIO_21,  //!< GPIO 21
    WICED_PLATFORM_GPIO_22,  //!< GPIO 22
    WICED_PLATFORM_GPIO_23,  //!< GPIO 23
    WICED_PLATFORM_GPIO_24,  //!< GPIO 24
    WICED_PLATFORM_GPIO_25,  //!< GPIO 25
    WICED_PLATFORM_GPIO_MAX //!< Max GPIO for error check
} wiced_platform_gpio_number_t;


//! List of pins available on the platform
enum wiced_platform_pins
{
    PLATFORM_GPIO_0,
    PLATFORM_GPIO_1,
    PLATFORM_GPIO_2,
    PLATFORM_GPIO_3,
    PLATFORM_GPIO_4,
    PLATFORM_GPIO_5,
    PLATFORM_GPIO_6,
    PLATFORM_GPIO_7,
    PLATFORM_GPIO_8,
    PLATFORM_GPIO_9,
    PLATFORM_GPIO_10,
    PLATFORM_GPIO_11,
    PLATFORM_GPIO_12,
    PLATFORM_GPIO_13,
    PLATFORM_GPIO_14,
    PLATFORM_GPIO_15,
    PLATFORM_GPIO_16,
    PLATFORM_GPIO_17,
    PLATFORM_GPIO_18,
    PLATFORM_GPIO_19,
    PLATFORM_GPIO_20,
    PLATFORM_GPIO_21,
    PLATFORM_GPIO_22,
    PLATFORM_GPIO_23,
    PLATFORM_GPIO_24,
    PLATFORM_GPIO_25,
    PLATFORM_GPIO_MAX_PINS
};

/**
 * configuration for the platform GPIOs
 */
typedef struct
{
    wiced_bt_gpio_numbers_t gpio_pin; /**< WICED GPIO pin */
    wiced_bt_gpio_function_t functionality; /**< chosen functionality for the pin */
}
wiced_platform_gpio_t;

/**
 * Configuration for platform LEDs
 */

/**
 * Configuration for platform Buttons
 */
typedef struct
{
    wiced_bt_gpio_numbers_t* gpio; /**< WICED GPIO pin */
    uint32_t config; /**< configuration like GPIO_PULL_DOWN,GPIO_PULL_UP etc., interrupt is configured through wiced_platform_register_button_callback(...) */
    uint32_t default_state; /**< GPIO_PIN_OUTPUT_HIGH/GPIO_PIN_OUTPUT_LOW */
    uint32_t button_pressed_value; /**< Button pressed value */
}
wiced_platform_button_config_t;

/**
 * Configuration for platform GPIOs
 */
typedef struct
{
    wiced_bt_gpio_numbers_t* gpio; /**< WICED GPIO pin */
    uint32_t config; /**< configuration like GPIO_PULL_DOWN,GPIO_PULL_UP etc., interrupt is configured through wiced_platform_register_button_callback(...) */
    uint32_t default_state; /**< GPIO_PIN_OUTPUT_HIGH/GPIO_PIN_OUTPUT_LOW */
}
wiced_platform_gpio_config_t;


/*! pin for button 1 */
#define WICED_GPIO_PIN_BUTTON_1       WICED_GPIO_10
#define WICED_GPIO_PIN_BUTTON         WICED_GPIO_PIN_BUTTON_1

/*! configuration settings for button, x can be GPIO_EN_INT_RISING_EDGE or GPIO_EN_INT_FALLING_EDGE or GPIO_EN_INT_BOTH_EDGE */
#define WICED_GPIO_BUTTON_SETTINGS(x)                       ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | x )

/*! pin for LED 1, only available with R67 at position B */
#define WICED_GPIO_PIN_LED_1     WICED_P08
/*! pin for LED 2, only available with R66 at position B */
#define WICED_GPIO_PIN_LED_2     WICED_P07

#define WICED_PUART_TXD          WICED_GPIO_06  // BT_I2S_CLK
#define WICED_PUART_RXD          WICED_GPIO_07  // BT_I2S_WS

/** Pin state for the LED on. */
#ifndef LED_STATE_ON
#define LED_STATE_ON          (GPIO_PIN_OUTPUT_LOW)
#endif
/** Pin state for the LED off. */
#ifndef LED_STATE_OFF
#define LED_STATE_OFF         (GPIO_PIN_OUTPUT_HIGH)
#endif

/** Pin state for when a button is pressed. */
#ifndef BTN_PRESSED
#define BTN_PRESSED           (GPIO_PIN_OUTPUT_LOW)
#endif
/** Pin state for when a button is released. */
#ifndef BTN_OFF
#define BTN_OFF               (GPIO_PIN_OUTPUT_HIGH)
#endif

#define WICED_RESET_PIN     WICED_P34


#else // USE_DESIGN_MODUS

//! Number of Buttons available on the platform.
typedef enum
{
    WICED_PLATFORM_BUTTON_1,  //!< BUTTON 1
    WICED_PLATFORM_BUTTON_2,  //!< BUTTON 2
    WICED_PLATFORM_BUTTON_3,  //!< BUTTON 3
    WICED_PLATFORM_BUTTON_4,  //!< BUTTON 4
    WICED_PLATFORM_BUTTON_MAX //!< Max button for error check
} wiced_platform_button_number_t;

#define WICED_PLATFORM_BUTTON_MAX_DEF     4   //define for platform_gpio_buttons

#define HCI_UART_DEFAULT_BAUD   3000000   /* default baud rate is 3M, that is max supported baud rate on Mac OS */

/* In fact, in cyw9b5_audio library, these settings are just for compile, no actual functionality */
#define SPI_CS      WICED_P00
#define SPI_CLK     WICED_P11
#define SPI_MOSI    WICED_P01
#define SPI_MISO    WICED_P09
#define I2S_DI      WICED_P04
#define I2S_DO      WICED_P06
#define I2S_WS      WICED_GPIO_09       // BT_PCM_SYNC, A_GPIO[1]
#define I2S_CLK     WICED_GPIO_08       // BT_PCM_CLK, A_GPIO[0]
#define WICED_RESET_PIN     WICED_P34

#define WICED_BUTTON1 (WICED_GPIO_03)
#define WICED_BUTTON2 (WICED_P07)
#define WICED_BUTTON3 (WICED_P13)
#define WICED_BUTTON4 (WICED_GPIO_10)   // BT_PCM_OUT, A_GPIO[2]

#define WICED_GPIO_BUTTON_SETTINGS(x)                       ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | x )

/**
 * wiced_platform_init
 *
 * Initialize the platform-specific peripheral interface settings
 */
void wiced_platform_init(void);

/**
 * wiced_platform_i2s_init
 *
 * Initialize the I2S interface.
 */
void wiced_platform_i2s_init(void);

/**
 * wiced_platform_spi_init
 *
 * Initialize the SPI interface.
 */
void wiced_platform_spi_init(void);

/**
 * wiced_platform_spi_init
 *
 * Initialize the SPI interface.
 */
void wiced_platform_i2c_init(void);

/* @} */
#endif

/**
 * wiced_platform_transport_init
 *
 * Initialize the WICED HCI Transport interface.
 *
 * @param[in] p_rx_data_handler : user callback for incoming HCI data.
 *
 * @return  WICED_TRUE - success
 *          WICED_FALSE - fail
 */
wiced_bool_t wiced_platform_transport_init(wiced_platform_transport_rx_data_handler *p_rx_data_handler);

/**
 * wiced_platform_set_ptu_fifo
 *
 * Select the source of PTU FIFO, before using spiffy, uart, or usb.
 */
void wiced_platform_set_ptu_fifo(wiced_platform_ptu_sel_t sel);

/**
 * wiced_platform_get_ptu_fifo
 *
 * Get the current source of PTU FIFO
 */
wiced_platform_ptu_sel_t wiced_platform_get_ptu_fifo(void);

/**
 * debug_uart_enable
 *
 * Initialize debug uart and set baud rate.
 *
 * @param[in] baud_rate : the speed of debug messsage
 */
void debug_uart_enable(uint32_t baud_rate);

/**
 * as a flag to present Bluetooth transport opened or not
 * 0: not open; others: open
 */
extern uint8_t platform_transport_started;


extern wiced_debug_uart_types_t wiced_debug_uart;

/* utility functions */
uint32_t wiced_platform_get_button_pressed_value(wiced_platform_button_number_t button);
void     wiced_platform_register_button_callback(wiced_platform_button_number_t button, void (*userfn)(void*, uint8_t), void* userdata, wiced_platform_button_interrupt_edge_t trigger_edge);
