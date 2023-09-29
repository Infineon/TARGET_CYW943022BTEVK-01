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

#include "wiced_bt_dev.h"
#include "wiced_hal_gpio.h"
#include "wiced_platform.h"
#include "wiced_hal_i2c.h"
#include "wiced_rtos.h"
#include "wiced_transport.h"
#include "hci_control_api.h"
#include "platform_mem.h"
#include "wiced_hal_nvram.h"
#include "wiced_bt_app_hal_common.h"
#include "wiced_rtos.h"

#ifndef PLATFORM_REMOTE_HOST_NVRAM_SUPPORT
#define PLATFORM_REMOTE_HOST_NVRAM_SUPPORT  1
#endif

#define REG32(address)  ( *(volatile UINT32*)(address) )

#define cr_pad_config_adr0                             0x00320068
#define cr_pad_config_adr1                             0x0032006c
#define cr_pad_config_adr4                             0x00320078
#define cr_pad_config_adr7                             0x003200e4
#define cr_pad_config_adr8                             0x00320110
#define cr_pad_fcn_ctl_adr0                            0x00320088
#define cr_pad_fcn_ctl_adr1                            0x0032008c
#define cr_pad_fcn_ctl_adr2                            0x00320090
#define cr_pad_fcn_ctl_adr3                            0x003201a8
#define iocfg_p0_adr                                   0x00650cd0
#define iocfg_p1_adr                                   0x00650cd4
#define iocfg_p2_adr                                   0x00650cd8
#define iocfg_p3_adr                                   0x00650cdc
#define iocfg_p4_adr                                   0x00650ce0
#define iocfg_p5_adr                                   0x00650ce4
#define iocfg_p9_adr                                   0x00650cf4
#define iocfg_p11_adr                                  0x00650cfc
#define iocfg_fcn_p0_adr                               0x00438400
#define iocfg_fcn_p1_adr                               0x00438404
#define iocfg_fcn_p2_adr                               0x00438408
#define iocfg_fcn_p3_adr                               0x0043840c
#define iocfg_fcn_p4_adr                               0x00438410
#define iocfg_fcn_p5_adr                               0x00438414
#define iocfg_fcn_p9_adr                               0x00438424
#define iocfg_fcn_p11_adr                              0x0043842c
#define iocfg_premux_0_adr                             0x004383a4
#define iocfg_premux_1_adr                             0x004383b0
#define cr_pad_fcn_ctl_lhl_0_adr                       0x0032021c
#define cr_pad_fcn_ctl_lhl_1_adr                       0x0032023c
#define dc_ptu_hc_sel_adr                              0x00360014
#define cr_smux_ctl_adr1                               0x00320120
#define db_uart_dhbr_adr                               0x00360800
#define db_uart_dlbr_adr                               0x00360804
#define db_uart_mcr_adr                                0x00360810
#define cr_tport_clk_sel_adr                           0x003200b4

extern uint32_t g_debuguart_BaudRate;
extern void uart_cal_divider(UINT32 baudrate, UINT32 *dhbr, UINT32 *dlbr, UINT32 * mcr);
extern BOOL8 debuguart_feature_enabled;
extern BOOL8 debuguart_enabled;
extern void debuguart_Init( UINT32 baud_rate );
extern wiced_bool_t rbg_ready(void);

uint16_t wiced_platform_nvram_write(uint16_t vs_id, uint16_t data_length, uint8_t *p_data, wiced_result_t *p_status);

typedef struct platform_virtual_nvram_t
{
    uint16_t                        data_length;
    struct platform_virtual_nvram_t *p_next;

    struct __attribute__((packed))
    {
        uint16_t                    vs_id;
        uint8_t                     data[0];
    } content;
} platform_virtual_nvram_t;

static struct
{
    struct
    {
        wiced_bool_t init;
        wiced_platform_transport_rx_data_handler *p_app_rx_data_handler;
    } transport;

    platform_virtual_nvram_t *p_virtual_nvram;
} platform_cb = {0};

uint8_t platform_transport_started = 0;

static void wiced_platform_debug_uart_init(void)
{
#if !USE_DESIGN_MODUS
    // BT_GPIO_2
    //cr_pad_config_adr0 [23:16] cr_pad_fcn_ctl_adr0 [11:8] smux_ctl_adr1[15:8]
    REG32(cr_pad_config_adr0) = (REG32(cr_pad_config_adr0) & ~0x00FF0000) | 0x00620000;
    REG32(cr_pad_fcn_ctl_adr0) = (REG32(cr_pad_fcn_ctl_adr0) & ~0x00000F00) | 0x00000800;
    REG32(cr_smux_ctl_adr1) = (REG32(cr_smux_ctl_adr1) & ~0x0000FF00) | 0x00000F00;
#endif // !USE_DESIGN_MODUS
}

/**
 * debug_uart_enable
 *
 * Initialize debug uart and set baud rate.
 */
void debug_uart_enable(uint32_t baud_rate)
{
    wiced_platform_debug_uart_init();
    debuguart_feature_enabled = 1;
    debuguart_enabled = TRUE;
    debuguart_Init(baud_rate);
}

/**
 * debug_uart_set_baudrate
 *
 * Set debug uart baud rate
 */
void debug_uart_set_baudrate(uint32_t baud_rate)
{
    UINT32 mcr=0x81, dhbr=0, dlbr=0, value=0;

    value = REG32(cr_tport_clk_sel_adr);

    g_debuguart_BaudRate = baud_rate;

    // 43022 transport clock rate is 24Mhz for baud rates <= 1.5 Mbit/sec, and 48Mhz for baud rates > 1.5 Mbit/sec.
    // HCI UART and Debug UART both use the Transport clock.
    // Debug UART baud rate need to reduce half, if the HCI UART baud rate is > 1.5 Mbps and debug UART is <= 1.5 Mbps.
    // Debug UART baud rate is not able to double, if the HCI UART baud rate is <= 1.5 Mbps and debug UART is > 1.5 Mpbs.
    // For example, Debug baud rate is 3 Mbps, and the HCI UART baud rate is 115200.
    // If We dobule the debug uart baud rate: 3 Mbps * 2 = 6 Mbps, it will exceed the limitation.
    // The default Debug UART baud rate is 115200, and default HCI UART baud rate is 3Mbps

    if((value & 0x01) && (baud_rate <= 1500000))
    {
        baud_rate = baud_rate / 2;
    }

    uart_cal_divider(baud_rate,&dhbr,&dlbr,&mcr);
    REG32(db_uart_mcr_adr) = 0;
    REG32(db_uart_dlbr_adr) = dlbr;
    REG32(db_uart_dhbr_adr) = dhbr;
    REG32(db_uart_mcr_adr) = mcr & 0x81;

}


/*
 * wiced_platform_warm_up_rbg200
 *
 * Warm up the Random Block Generator 200
 */
static void wiced_platform_warm_up_rbg200(void)
{
    /* Wait the till the RBG200 (Random Block Generator) has been warmed up. */
    while (1)
    {
        if (rbg_ready())
        {
            break;
        }
        else
        {
            wiced_rtos_delay_milliseconds(100, ALLOW_THREAD_TO_SLEEP);
        }
    }
}

/*
 * wiced_platform_warm_up
 *
 * Warm up the platform-specific modules
 */
void wiced_platform_warm_up(void)
{
    wiced_platform_warm_up_rbg200();
}


// FillinFunctionInfo is available in patch, but the ROM replacement entry is not installed
// we can call it directly here if we need to
//  - note the code here only configures the pin, it does not unconfigure a previously configured pin
typedef struct
{
    BOOL8 RetVal;
    UINT8 shift;
    UINT16 fcn;
    UINT32 premux_adr;
    UINT32 enable_adr;
    UINT32 enable_value;
    UINT32 enable_mask;
    UINT32 pin_config;
} function_config_info_t;

void FillinFunctionInfo(wiced_bt_gpio_numbers_t gpio, wiced_bt_gpio_function_t function, function_config_info_t* info);

void wiced_hal_gpio_select_function_local(wiced_bt_gpio_numbers_t gpio, wiced_bt_gpio_function_t function)
{
    function_config_info_t info;
    FillinFunctionInfo(gpio, function, &info);

    if (info.RetVal)
    {
        if (info.fcn)
        {
            REG32(iocfg_fcn_p0_adr + (4 * gpio)) = info.fcn;
        }
        if (info.premux_adr)
        {
            REG32(info.premux_adr) &= ~((0xff) << info.shift);
            REG32(info.premux_adr) |= ((gpio + 1) << info.shift);
        }
        if (info.enable_adr)
        {
            if (info.enable_mask)
            {
                REG32(info.enable_adr) &= ~info.enable_mask;
            }
            REG32(info.enable_adr) |= info.enable_value;
        }
        REG32(iocfg_p0_adr + (4 * gpio)) = info.pin_config;
    }
}

#if !USE_DESIGN_MODUS
void wiced_platform_spi_init(void)
{
    //P0 - nCS  - RSVD_2
    //P3 - SCK  - RSVD_5
    //P4 - MOSI - RSVD_6
    //P5 - MISO - RSVD_7

    REG32(iocfg_p0_adr) = 0x000;
    REG32(iocfg_fcn_p0_adr) = 0x514;
    REG32(iocfg_premux_0_adr) = (REG32(iocfg_premux_0_adr) & 0x00ffffff) | ((0+1) << 24);

    REG32(iocfg_p3_adr) = 0x000;
    REG32(iocfg_fcn_p3_adr) = 0x504;
    REG32(iocfg_premux_1_adr) = (REG32(iocfg_premux_1_adr) & 0xffffff00) | ((3+1) << 0);

    REG32(iocfg_p4_adr) = 0x000;
    REG32(iocfg_fcn_p4_adr) = 0x524;
    REG32(iocfg_premux_1_adr) = (REG32(iocfg_premux_1_adr) & 0xffff00ff) | ((4+1) << 8);

    REG32(iocfg_p5_adr) = 0x000;
    REG32(iocfg_fcn_p5_adr) = 0x534;
    REG32(iocfg_premux_0_adr) = (REG32(iocfg_premux_0_adr) & 0xff00ffff) | ((5+1) << 16);

    REG32(cr_pad_fcn_ctl_lhl_0_adr) = REG32(cr_pad_fcn_ctl_lhl_0_adr) | 0x80000000;
    REG32(cr_pad_fcn_ctl_lhl_1_adr) = (REG32(cr_pad_fcn_ctl_lhl_1_adr) & 0xfffffff0) | 0x0d;
}

void wiced_platform_i2s_init(void)
{
    // Mux to BT_PCM_IN to I2S_DI
    REG32(cr_pad_config_adr4) = (REG32(cr_pad_config_adr4) & 0xFF00FFFF) | (0x09 << 0);
    REG32(cr_pad_fcn_ctl_adr1) = (REG32(cr_pad_fcn_ctl_adr1) & 0xf0ffffff) | (0x07 << 12);

    // Mux to BT_PCM_OUT to I2S_DO
    REG32(cr_pad_config_adr4) = (REG32(cr_pad_config_adr4) & 0xFFFF00FF) | (0x88 << 8);
    REG32(cr_pad_fcn_ctl_adr1) = (REG32(cr_pad_fcn_ctl_adr1) & 0xfff0ffff) | (0x05 << 16);

    // Mux to BT_PCM_SYNC to I2S_WS
    REG32(cr_pad_config_adr4) = (REG32(cr_pad_config_adr4) & 0x00FFFFFF) | (0x88 << 24);
    REG32(cr_pad_fcn_ctl_adr1) = (REG32(cr_pad_fcn_ctl_adr1) & 0xff0fffff) | (0x05 << 20);

    // Mux to BT_PCM_CLK to I2S_SCK
    REG32(cr_pad_config_adr4) = (REG32(cr_pad_config_adr4) & 0xFF00FFFF) | (0x88 << 16);
    REG32(cr_pad_fcn_ctl_adr1) = (REG32(cr_pad_fcn_ctl_adr1) & 0xf0ffffff) | (0x05 << 24);
}

static void wiced_platform_button_init(void)
{
    /* BT_PCM_OUT as User button */
    /* cr_pad_config_adr4 [15:8] = 0x60. Set input enable + pull up for BT_PCM_OUT */
    REG32(cr_pad_config_adr4) = (REG32(cr_pad_config_adr4) & 0xffff00ff) | (0x22 << 8);
    /* cr_pad_fcn_ctl_adr1 [19:16] = 0x2. Select BT_PCM_OUT to A_GPIO[2] */
    REG32(cr_pad_fcn_ctl_adr3) = (REG32(cr_pad_fcn_ctl_adr3) & 0xffff0fff) | (0x2 << 12);

    /* P7 as vol+ button : No extra setting here */

    /* BT_GPIO_3 as custom button */
    /* cr_pad_config_adr0 [31:24] = 0x60. Set input enable for BT_GPIO3 */
    REG32(cr_pad_config_adr0) = (REG32(cr_pad_config_adr0) & 0x00ffffff) | (0x20 << 24);
    /* cr_pad_fcn_ctl_adr0 [15:12] = 0x0. Select BT_GPIO3 to GPIO3 */
    REG32(cr_pad_fcn_ctl_adr0) = (REG32(cr_pad_fcn_ctl_adr0) & 0xffff0fff) | (0x0 << 12);

    /* P13 as vol- button : No extra setting here */
}
#endif // USE_DESIGN_MODUS

void i2c_init_m2base43012(void)
{
    /* 1. Set BT_GPIO5 as I2C_SCL. */
    /*
     * 1.1. Set Control Pad: cr_pad_config_adr1 [15:8]
     *      D[0]:oeb, 0: output enabled, 1: output disabled
     *      D[1]:pub, 0: pull high disabled, 1: pull high enabled
     *      D[2]:pdn, 0: pull down disabled, 1: pull down enabled
     *      D[3]:selhys, 0:hysteresis disabled, 1: hysteresis enabled
     *      D[6]:ind, 0: input not disabled (input enabled), 1: input disabled
     *      D[7,5:4]:src, slew rate control
     */
    REG32(cr_pad_config_adr1) = (REG32(cr_pad_config_adr1) & 0xffff00ff) | (0x02 << 8);
    /*
     * 1.2. Assign Function: cr_pad_fcn_ctl_adr0 [23:20]
     *      0: GPIO[5]
     *      1: HCLK
     *      2: -
     *      3: I2S_MSCK
     *      4: I2S_SSCK
     *      5: PHY_DEBUG_5
     *      6: wlan_clk_req
     *      7: CLK_REQ
     *      8: Super_Mux
     *      9: -
     *      10: -
     *      11: UART2_CTS_N
     *      12: BT_RX_ACTIVE
     *      13: -
     *      14: mia_debug[2]
     *      15: SCL
     */
    REG32(cr_pad_fcn_ctl_adr0) = (REG32(cr_pad_fcn_ctl_adr0) & 0xff0fffff) | (0xf << 20);

    /* 2. Set BT_GPIO4 as I2C_SDA. */
    /*
     * 2.1. Set Control Pad: cr_pad_config_adr1 [7:0]
     *      D[0]:oeb, 0: output enabled, 1: output disabled
     *      D[1]:pub, 0: pull high disabled, 1: pull high enabled
     *      D[2]:pdn, 0: pull down disabled, 1: pull down enabled
     *      D[3]:selhys, 0:hysteresis disabled, 1: hysteresis enabled
     *      D[6]:ind, 0: input not disabled (input enabled), 1: input disabled
     *      D[7,5:4]:src, slew rate control
     */
    REG32(cr_pad_config_adr1) = (REG32(cr_pad_config_adr1) & 0xffffff00) | 0x02;
    /*
     * 2.2. Assign Function: ccr_pad_fcn_ctl_adr0 [19:16]
     *      0: GPIO[4]
     *      1: LINK_IND
     *      2: DEBUG_RXD
     *      3: I2S_MSDO
     *      4: I2S_SSDO
     *      5: PHY_DEBUG_4
     *      6: wlan_clk_req
     *      7: -
     *      8: Super_Mux
     *      9: CLASS1[1]
     *      10: -
     *      11: UART2_RTS_N
     *      12: coex_out[1]
     *      13: -
     *      14: mia_debug[1]
     *      15: SDA
     */
    REG32(cr_pad_fcn_ctl_adr0) = (REG32(cr_pad_fcn_ctl_adr0) & 0xfff0ffff) | (0xf << 16);
}

void i2c_init_bitbang(void)
{
    /* 1. Set BT_GPIO5 as I2C_SCL. */
    /*
     * 1.1. Set Control Pad: cr_pad_config_adr1 [15:8]
     *      D[0]:oeb, 0: output enabled, 1: output disabled
     *      D[1]:pub, 0: pull high disabled, 1: pull high enabled
     *      D[2]:pdn, 0: pull down disabled, 1: pull down enabled
     *      D[3]:selhys, 0:hysteresis disabled, 1: hysteresis enabled
     *      D[6]:ind, 0: input not disabled (input enabled), 1: input disabled
     *      D[7,5:4]:src, slew rate control
     */
    REG32(cr_pad_config_adr1) = (REG32(cr_pad_config_adr1) & 0xffff00ff) | (0x02 << 8);
 //   REG32(cr_pad_config_adr1) = REG32(cr_pad_config_adr1) & 0xffff00ff;
    /*
     * 1.2. Assign Function: cr_pad_fcn_ctl_adr0 [23:20]
     *      0: GPIO[5]
     *      1: HCLK
     *      2: -
     *      3: I2S_MSCK
     *      4: I2S_SSCK
     *      5: PHY_DEBUG_5
     *      6: wlan_clk_req
     *      7: CLK_REQ
     *      8: Super_Mux
     *      9: -
     *      10: -
     *      11: UART2_CTS_N
     *      12: BT_RX_ACTIVE
     *      13: -
     *      14: mia_debug[2]
     *      15: SCL
     */
    REG32(cr_pad_fcn_ctl_adr0) = REG32(cr_pad_fcn_ctl_adr0) & 0xff0fffff;

    /* 2. Set BT_GPIO4 as I2C_SDA. */
    /*
     * 2.1. Set Control Pad: cr_pad_config_adr1 [7:0]
     *      D[0]:oeb, 0: output enabled, 1: output disabled
     *      D[1]:pub, 0: pull high disabled, 1: pull high enabled
     *      D[2]:pdn, 0: pull down disabled, 1: pull down enabled
     *      D[3]:selhys, 0:hysteresis disabled, 1: hysteresis enabled
     *      D[6]:ind, 0: input not disabled (input enabled), 1: input disabled
     *      D[7,5:4]:src, slew rate control
     */
    REG32(cr_pad_config_adr1) = (REG32(cr_pad_config_adr1) & 0xffffff00) | 0x02;
 //   REG32(cr_pad_config_adr1) = REG32(cr_pad_config_adr1) & 0xffffff00;
    /*
     * 2.2. Assign Function: ccr_pad_fcn_ctl_adr0 [19:16]
     *      0: GPIO[4]
     *      1: LINK_IND
     *      2: DEBUG_RXD
     *      3: I2S_MSDO
     *      4: I2S_SSDO
     *      5: PHY_DEBUG_4
     *      6: wlan_clk_req
     *      7: -
     *      8: Super_Mux
     *      9: CLASS1[1]
     *      10: -
     *      11: UART2_RTS_N
     *      12: coex_out[1]
     *      13: -
     *      14: mia_debug[1]
     *      15: SDA
     */
    REG32(cr_pad_fcn_ctl_adr0) = REG32(cr_pad_fcn_ctl_adr0) & 0xfff0ffff;
}

#if !USE_DESIGN_MODUS
/**
 * wiced_platform_i2c_init
 *
 * Initialize the I2C interface.
 */
void wiced_platform_i2c_init(void)
{
    wiced_hal_i2c_init();
    i2c_init_m2base43012();
    wiced_hal_i2c_set_speed(I2CM_SPEED_400KHZ);
}


/**
 * wiced_platform_init
 *
 * Initialize the platform-specific peripheral interface settings.
 */
void wiced_platform_init(void)
{
    wiced_bool_t result;
    wiced_bt_app_hal_init();
    wiced_platform_warm_up();
    debug_uart_enable(115200);
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_DBG_UART );
    wiced_platform_spi_init();
    wiced_platform_i2s_init();
    wiced_platform_i2c_init();
    wiced_platform_button_init();

    result = platform_mem_init();
	if(result == WICED_FALSE)
		WICED_BT_TRACE( "platform_mem_init fail\n" );

}
#endif // USE_DESIGN_MODUS

/*
 * platform_transport_status_handler
 */
static void platform_transport_status_handler( wiced_transport_type_t type )
{
    wiced_transport_send_data(HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0);
}

static void platform_transport_rx_data_handler_push_nvram_data(uint8_t *p_data, uint32_t data_len)
{
    uint16_t vs_id;
    uint32_t payload_len;
    platform_virtual_nvram_t *p_new;
    wiced_result_t status;

    /* Check parameter. */
    if ((p_data == NULL) ||
        (data_len == 0))
    {
        return;
    }

    /* Parse information. */
    STREAM_TO_UINT16(vs_id, p_data);
    payload_len = data_len - sizeof(vs_id);

    wiced_platform_nvram_write(vs_id, payload_len, p_data, &status);
    (void) status;
}

/*
 * platform_transport_rx_data_handler
 */
static uint32_t platform_transport_rx_data_handler(uint8_t *p_buffer, uint32_t length)
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t *p_data = p_buffer;
    uint32_t sample_rate = 16000;
    uint8_t wiced_hci_status = 1;
    wiced_result_t status;
    uint8_t param8;

    /* Check parameter. */
    if (p_buffer == NULL)
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    // Expected minimum 4 byte as the wiced header
    if (length < (sizeof(opcode) + sizeof(payload_len)))
    {
        wiced_transport_free_buffer(p_buffer);
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    STREAM_TO_UINT16(opcode, p_data);       // Get OpCode
    STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length

    switch(opcode)
    {
    case HCI_CONTROL_HCI_AUDIO_COMMAND_PUSH_NVRAM_DATA:
        platform_transport_rx_data_handler_push_nvram_data(p_data, payload_len);
        break;
    default:
        if (platform_cb.transport.p_app_rx_data_handler)
        {
            (*platform_cb.transport.p_app_rx_data_handler)(opcode, p_data, payload_len);
        }
        break;
    }

    // Freeing the buffer in which data is received
    wiced_transport_free_buffer(p_buffer);

    return HCI_CONTROL_STATUS_SUCCESS;
}

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
wiced_bool_t wiced_platform_transport_init(wiced_platform_transport_rx_data_handler *p_rx_data_handler)
{

wiced_transport_cfg_t cfg = {
        .type = WICED_TRANSPORT_UART,
        .cfg.uart_cfg = {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
            .baud_rate = HCI_UART_DEFAULT_BAUD,
        },
#ifdef NEW_DYNAMIC_MEMORY_INCLUDED
        .heap_config =
        {
            .data_heap_size = 1024 * 4 + 1500 * 2,
            .hci_trace_heap_size = 1024 * 2,
            .debug_trace_heap_size = 1024,
        },
#else
        .rx_buff_pool_cfg = {
            .buffer_size = 0,
            .buffer_count = 0,
        },
#endif
        .p_status_handler = platform_transport_status_handler,
        .p_data_handler = platform_transport_rx_data_handler,
        .p_tx_complete_cback = NULL,
    };

    wiced_result_t result;

    if (platform_cb.transport.init)
    {
        return WICED_FALSE;
    }

    /* Initialize the transport. */
    result = wiced_transport_init(&cfg);

    if (result == WICED_BT_SUCCESS)
    {
        platform_cb.transport.init = WICED_TRUE;
        platform_cb.transport.p_app_rx_data_handler = p_rx_data_handler;

        return WICED_TRUE;
    }

    return WICED_FALSE;
}

/**
 * wiced_platform_nvram_read
 *
 * Reads the data from NVRAM
 *
 * @param[in] vs_id         : Volatile Section Identifier. Application can use the VS IDs from
 *                            WICED_NVRAM_VSID_START to WICED_NVRAM_VSID_END
 * @param[in] data_length   : Length of the data to be read from NVRAM
 * @param[out] p_data       : Pointer to the buffer to which data will be copied
 * @param[out] p_status     : Pointer to location where status of the call is returned
 *
 * @return length of data that is read
 */
uint16_t wiced_platform_nvram_read(uint16_t vs_id, uint16_t data_length, uint8_t *p_data, wiced_result_t *p_status)
{
#if (PLATFORM_REMOTE_HOST_NVRAM_SUPPORT)
    platform_virtual_nvram_t *p_index;

    /* Check parameter. */
    if (p_status == NULL)
    {
        return 0;
    }

    *p_status = WICED_BADARG;

    if ((data_length == 0) ||
        (p_data == NULL))
    {
        return 0;
    }

    /* Check if the target vs_id exists. */
    p_index = platform_cb.p_virtual_nvram;
    while (p_index)
    {
        if (p_index->content.vs_id == vs_id)
        {
            /* Check the data length. */
            if (data_length < p_index->data_length)
            {
                return 0;
            }

            memcpy((void *) p_data, (void *) &p_index->content.data[0], p_index->data_length);

            break;
        }

        p_index = p_index->p_next;
    }

    if (p_index == NULL)
    {
        return 0;
    }

    *p_status = WICED_SUCCESS;

    return p_index->data_length;
#else
    *p_status = WICED_ERROR;
    return 0;
#endif
}

/**
 * wiced_platform_nvram_write
 *
 * Reads the data to NVRAM
 *
 * @param[in] vs_id         : Volatile Section Identifier. Application can use the VS IDs from
 *                            WICED_NVRAM_VSID_START to WICED_NVRAM_VSID_END
 * @param[in] data_length   : Length of the data to be written to the NVRAM
 * @param[in] p_data        : Pointer to the data to be written to the NVRAM
 * @param[out] p_status     : Pointer to location where status of the call is returned
 *
 * @return number of bytes written, 0 on error
 */
uint16_t wiced_platform_nvram_write(uint16_t vs_id, uint16_t data_length, uint8_t *p_data, wiced_result_t *p_status)
{
#if (PLATFORM_REMOTE_HOST_NVRAM_SUPPORT)
    platform_virtual_nvram_t *p_index = NULL;
    wiced_bool_t update = WICED_FALSE;

    /* Check parameter. */
    if (p_status == NULL)
    {
        return 0;
    }

    *p_status = WICED_BADARG;

    if ((data_length == 0) ||
        (p_data == NULL))
    {
        return 0;
    }

    /* Check if the target vs_id exists. */
    p_index = platform_cb.p_virtual_nvram;
    while (p_index)
    {
        if (p_index->content.vs_id == vs_id)
        {
            wiced_result_t result;

            /* Check the data length. */
            if (data_length != p_index->data_length)
            {
                /* Delete this entry. */
                wiced_platform_nvram_delete(vs_id, p_status);

                /* Add a new entry. */
                return wiced_platform_nvram_write(vs_id, data_length, p_data, p_status);
            }

            /* Check if the data shall be updated. */
            if (memcmp((void *) p_data,
                       (void *) &p_index->content.data[0],
                       data_length) == 0)
            {
                *p_status = WICED_SUCCESS;
                return data_length;
            }

            /* Update data content. */
            memcpy((void *) &p_index->content.data[0], (void *) p_data, data_length);

#if 0
            /* Inform Host device. */
            result = wiced_transport_send_data(HCI_CONTROL_HCI_AUDIO_EVENT_WRITE_NVRAM_DATA,
                                          (uint8_t *) &p_index->content,
                                          sizeof(p_index->content.vs_id) + p_index->data_length);
            if (result != WICED_SUCCESS)
            {
                /* fail, delete entry due to unsync */
                wiced_platform_nvram_delete(vs_id, p_status);

                *p_status = result;
                return 0;
            }
#endif
            *p_status = WICED_SUCCESS;
            return p_index->data_length;
        }

        p_index = p_index->p_next;
    }

    /* Acquire memory. */
    p_index = (platform_virtual_nvram_t *) platform_mem_allocate(sizeof(platform_virtual_nvram_t) - sizeof(uint8_t) + data_length);
    if (p_index == NULL)
    {
        *p_status = WICED_NO_MEMORY;
        return 0;
    }

    /* Write information. */
    p_index->content.vs_id = vs_id;
    p_index->data_length = data_length;
    memcpy((void *) &p_index->content.data[0], (void *) p_data, data_length);

    /* Add the new entry to the list. */
    p_index->p_next = platform_cb.p_virtual_nvram;
    platform_cb.p_virtual_nvram = p_index;
#if 0
    /* Inform Host device. */
    if (wiced_transport_send_data(HCI_CONTROL_HCI_AUDIO_EVENT_WRITE_NVRAM_DATA,
                                  (uint8_t *) &p_index->content,
                                  sizeof(p_index->content.vs_id) + p_index->data_length) != WICED_SUCCESS)
    {
        wiced_platform_nvram_delete(p_index->content.vs_id, p_status);
        *p_status = WICED_NO_MEMORY;
        return 0;
    }
#endif
    *p_status = WICED_SUCCESS;
    return p_index->data_length;

#else
    *p_status = WICED_ERROR;
    return 0;
#endif
}

/**
 * wiced_platform_nvram_delete
 *
 * Deletes data from NVRAM at specified VS id
 *
 * @param vs_id     : Volatile Section Identifier. Application can use the VS IDs from
 *                    WICED_NVRAM_VSID_START to WICED_NVRAM_VSID_END
 * @param p_status  : Pointer to location where status of the call is returned
 */
void wiced_platform_nvram_delete(uint16_t vs_id, wiced_result_t *p_status)
{
#if (PLATFORM_REMOTE_HOST_NVRAM_SUPPORT)
    platform_virtual_nvram_t *p_index = NULL;
    platform_virtual_nvram_t *p_pre = NULL;

    /* Check parameter. */
    if (p_status == NULL)
    {
        return;
    }

    *p_status = WICED_BADARG;

    p_index = platform_cb.p_virtual_nvram;
    p_pre = NULL;
    while (p_index)
    {
        if (p_index->content.vs_id == vs_id)
        {
#if 0
            /* Inform Host device. */
            wiced_transport_send_data(HCI_CONTROL_HCI_AUDIO_EVENT_DELETE_NVRAM_DATA,
                                      (uint8_t *) &p_index->content.vs_id,
                                      sizeof(p_index->content.vs_id));
#endif
            /* Remove this entry from the list. */
            if (p_pre == NULL)
            {
                platform_cb.p_virtual_nvram = p_index->p_next;
            }
            else
            {
                p_pre->p_next = p_index->p_next;
            }

            platform_mem_free((void *) p_index);

            break;
        }

        p_pre = p_index;
        p_index = p_index->p_next;
    }
#endif
}

/**
 * wiced_platform_set_ptu_fifo
 *
 * Select the source of PTU FIFO, before using spiffy, uart, or usb.
 */
void wiced_platform_set_ptu_fifo(wiced_platform_ptu_sel_t sel)
{
/* force a delay before new ptu_fifo_sel value set. let previous module HW finish its job */
/* the delay value (ms) is an experiment value, not a calculataed value */
#define PTU_FIFO_BEFORE_SEL_DELAY   20

    /* if UART transport is not open, don't need to switch between SPI and UART */
    if (platform_transport_started == 0)
        return;

    if (REG32(dc_ptu_hc_sel_adr) == sel)
        return;

    if (REG32(dc_ptu_hc_sel_adr) == WICED_PLATFORM_PTU_SEL_UART)
        wiced_transport_uart_pause();

    wiced_rtos_delay_milliseconds(PTU_FIFO_BEFORE_SEL_DELAY, ALLOW_THREAD_TO_SLEEP);
    REG32(dc_ptu_hc_sel_adr) = (uint32_t)sel;

    if (sel == WICED_PLATFORM_PTU_SEL_UART)
        wiced_transport_uart_resume();
}

/**
 * wiced_platform_get_ptu_fifo
 *
 * Get the current source of PTU FIFO
 */
wiced_platform_ptu_sel_t wiced_platform_get_ptu_fifo(void)
{
    uint32_t sel = REG32(dc_ptu_hc_sel_adr);

    if ((sel == WICED_PLATFORM_PTU_SEL_USB) || (sel == WICED_PLATFORM_PTU_SEL_UART)
            || (sel == WICED_PLATFORM_PTU_SEL_I2C_SLAVE) || (sel == WICED_PLATFORM_PTU_SEL_SPIFFY))
        return sel;

    return WICED_PLATFORM_PTU_SEL_UNKNOWN;
}
