/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  ArduPilot bootloader. This implements the same protocol originally
  developed for PX4, but builds on top of the ChibiOS HAL

  It does not use the full AP_HAL API in order to keep the firmware
  size below the maximum of 16kByte required for F4 based
  boards. Instead it uses the ChibiOS APIs directly
 */

#include <AP_HAL/AP_HAL.h>
#include "ch.h"
#include "hal.h"
#include "hwdef.h"
#include <AP_HAL_ChibiOS/hwdef/common/usbcfg.h>
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>
#include "support.h"
#include "bl_protocol.h"
#include "can.h"
#include <stdio.h>
//#include <string>
//#include <iostream>
//#include <sstream>
#include "sha256.h"
#if EXTERNAL_PROG_FLASH_MB
#include <AP_FlashIface/AP_FlashIface_JEDEC.h>
#endif

/*              Memory details

#define BOARD_FLASH_SIZE 2048
#define FLASH_BOOTLOADER_LOAD_KB 128



*/
int inttostr(uint8_t *s, int n);

#define DEBUG_BUFFER_SIZE 72

#define APP_START_ADDRESS (FLASH_LOAD_ADDRESS + (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB) * 1024U)
#define CRC_STORE_ADDRESS 0x081E0000
extern "C"
{
    int main(void);
}

struct boardinfo board_info = {
    .board_type = APJ_BOARD_ID,
    .board_rev = 0,
    .fw_size = (BOARD_FLASH_SIZE - (FLASH_BOOTLOADER_LOAD_KB + FLASH_RESERVE_END_KB + APP_START_OFFSET_KB))*1024,
    .extf_size = (EXTERNAL_PROG_FLASH_MB * 1024 * 1024)
};

#ifndef HAL_BOOTLOADER_TIMEOUT
#define HAL_BOOTLOADER_TIMEOUT 5000
#endif

#ifndef HAL_STAY_IN_BOOTLOADER_VALUE
#define HAL_STAY_IN_BOOTLOADER_VALUE 0
#endif

#if EXTERNAL_PROG_FLASH_MB
AP_FlashIface_JEDEC ext_flash;
#endif
//uint32_t buf[100];
uint8_t bootFlag = 0;
uint8_t dbgBuf[DEBUG_BUFFER_SIZE] = {0};
const uint32_t *app_base = (const uint32_t *)(APP_START_ADDRESS);
//const uint8_t  *crc_base = (uint8_t *)(0x081E0000);
const uint32_t *crc_base = (uint32_t *)(0x081E0000);
static char shaData[100];

int inttostr(uint8_t *s, int n)
{
    unsigned int i = 1000000000;

    if (((signed)n) < 0)
    {
        *s++ = '-';
        n = -n;
    }

    while (i > n)
        i /= 10;

    do
    {
        *s++ = '0' + (n - n % i) / i % 10;
    } while (i /= 10);

    *s = 0;

    return n;
}

int main(void)
{   
    static union
    {
        uint8_t c[256];
        uint32_t w[64];
    } flash_buffer;
    //uint8_t Cbuf[64];
    static char token[8];
    //const char s[2] = ",";
    //uint32_t codeLen = 0;
    if (BOARD_FLASH_SIZE > 1024 && check_limit_flash_1M())
    {
        board_info.fw_size = (1024 - (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB)) * 1024;
    }

    bool try_boot = false;
    uint32_t timeout = HAL_BOOTLOADER_TIMEOUT;
    SHA256 sha256;
    //uint32_t codeLen = 0;
    //std::string strData;
    //std::ostringstream data;

#ifdef HAL_BOARD_AP_PERIPH_ZUBAXGNSS
    // setup remapping register for ZubaxGNSS
    uint32_t mapr = AFIO->MAPR;
    mapr &= ~AFIO_MAPR_SWJ_CFG;
    mapr |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
    AFIO->MAPR = mapr | AFIO_MAPR_CAN_REMAP_REMAP2 | AFIO_MAPR_SPI3_REMAP;
#endif

#ifndef NO_FASTBOOT
    enum rtc_boot_magic m = check_fast_reboot();
    bool was_watchdog = stm32_was_watchdog_reset();
    if (was_watchdog) {
        try_boot = true;
        timeout = 0;
    } else if (m == RTC_BOOT_HOLD) {
        timeout = 0;
    } else if (m == RTC_BOOT_FAST) {
        try_boot = true;
        timeout = 0;
    }
#if HAL_USE_CAN == TRUE || HAL_NUM_CAN_IFACES
    else if ((m & 0xFFFFFF00) == RTC_BOOT_CANBL) {
        try_boot = false;
        timeout = 10000;
        can_set_node_id(m & 0xFF);
    }
    can_check_update();
    if (!can_check_firmware()) {
        // bad firmware CRC, don't try and boot
        timeout = 0;
        try_boot = false;
    }
#ifndef BOOTLOADER_DEV_LIST
    else if (timeout != 0) {
        // fast boot for good firmware
        try_boot = true;
        timeout = 1000;
    }
#endif
    if (was_watchdog && m != RTC_BOOT_FWOK) {
        // we've had a watchdog within 30s of booting main CAN
        // firmware. We will stay in bootloader to allow the user to
        // load a fixed firmware
        stm32_watchdog_clear_reason();
        try_boot = false;
        timeout = 0;
    }
#endif
#if defined(HAL_GPIO_PIN_VBUS) && defined(HAL_ENABLE_VBUS_CHECK)
#if HAL_USE_SERIAL_USB == TRUE
    else if (palReadLine(HAL_GPIO_PIN_VBUS) == 0)  {
        try_boot = true;
        timeout = 0;
    }
#endif
#endif

    // if we fail to boot properly we want to pause in bootloader to give
    // a chance to load new app code
    set_fast_reboot(RTC_BOOT_OFF);
#endif

#ifdef HAL_GPIO_PIN_STAY_IN_BOOTLOADER
    // optional "stay in bootloader" pin
    if (palReadLine(HAL_GPIO_PIN_STAY_IN_BOOTLOADER) == HAL_STAY_IN_BOOTLOADER_VALUE) {
        try_boot = false;
        timeout = 0;
    }
#endif
    init_uarts();
    cout((uint8_t *)"\r\n", 4);

    for (int i = 0; i < 18; i++)
    {        
        flash_buffer.w[i] = crc_base[i];
    }
    chThdSleepSeconds(2);
    cout((uint8_t *)"In Bootloader\r\n", 29);
    strncpy(token, (char *)flash_buffer.c, 7);
    cout((uint8_t *)token, 8);
    cout((uint8_t*)"\r\n", 4);
    uint32_t len = atoi(token);
    cout((uint8_t *)&flash_buffer.c[8], 72);
    cout((uint8_t*)"\r\n", 4);

    if (len != 0xFFFFFFFF)
    {
        strcpy(shaData, sha256(app_base, len));
        cout((uint8_t*)"Calculated SHA : ", 16);
        cout((uint8_t *)shaData, strlen((char *)shaData));
    }
    else
    {
        cout((uint8_t *)"SHA256 Not Found\r\n", 20);
    }
    cout((uint8_t *)"\r\n", 4);
    chThdSleepSeconds(1);
    memset(dbgBuf, 0, sizeof(dbgBuf) );
    strncpy((char*)dbgBuf, (char*)&flash_buffer.c[8], 64 );
	cout((uint8_t*)dbgBuf, strlen((char*)dbgBuf) );
	
	if( strncmp((char*)dbgBuf, (char*)shaData, 64) == 0 )
	{
        cout((uint8_t*)"\r\n", 4);
		cout((uint8_t*)"SHA256 Matched\r\n", 16);
		bootFlag = 1;
	}
	else
	{
        cout((uint8_t*)"\r\n", 4);
		cout((uint8_t*)"SHA256 Mismatch\r\n", 17);
		bootFlag = 0;
	}
	
    chThdSleepSeconds(2);
    if (try_boot && (bootFlag == 1))
    {
        jump_to_app();
    }

#if defined(BOOTLOADER_DEV_LIST)
    //init_uarts();
#endif
#if HAL_USE_CAN == TRUE || HAL_NUM_CAN_IFACES
    can_start();
#endif
    flash_init();


#if EXTERNAL_PROG_FLASH_MB
    while (!ext_flash.init()) {
        // keep trying until we get it working
        // there's no future without it
        chThdSleep(1000);
    }
#endif
    chThdSleepSeconds(2);
    
#if defined(BOOTLOADER_DEV_LIST)
    while (true) {
        bootloader(timeout);
        if( bootFlag == 1 )
		{
			jump_to_app();
		}
    }
#else
    // CAN only
    while (true) {
        uint32_t t0 = AP_HAL::millis();
        while (timeout == 0 || AP_HAL::millis() - t0 <= timeout) {
            can_update();
            chThdSleep(chTimeMS2I(1));
        }
        if( bootFlag == 1 )
        {
            jump_to_app();
        }    
    }
#endif
}


