#include "MPU9250.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"



void app_main()
{
    //MPU9250(long clock, uint8_t cs, uint8_t low_pass_filter = BITS_DLPF_CFG_188HZ, uint8_t low_pass_filter_acc = BITS_DLPF_CFG_188HZ)
    MPU9250 senzor(32,14,0x11,0x11);
    senzor.spiinitialize();
}