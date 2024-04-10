#include "MPU9250.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"



void app_main()
{
    
    senzor =MPU9250();
    senzor.spiinitialize();
}