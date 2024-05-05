#include "MPU9250.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "MPU9250.h"


//int main()
extern "C" {
    void app_main(void);
}


void app_main()
{
    //MPU9250(long clock, uint8_t cs, uint8_t low_pass_filter = BITS_DLPF_CFG_188HZ, uint8_t low_pass_filter_acc = BITS_DLPF_CFG_188HZ)
    MPU9250 senzor(32,14,0x11,0x11);
    senzor.spiinitialize();
    /*
    senzor.whoami();
    senzor.WriteReg(0x1A,0x11); 
    uint8_t RBuf[8];
    senzor.ReadRegs(0x1A, RBuf,8);
         
    Accelometer_Scale scale=BITSFS_16G;
    senzor.set_acc_scale(scale);
    Gyro_Scale scale2=BITSFS_2000; 
    senzor.set_gyro_scale(scale2); 
    */
    /*
        -set gyro dlpf
        -set gyro scale, 
        -egybe és külön config-olás
        -scale,offset egybe struktűraként mindenes fgv
    */
    
    while(1)
    {
        const TickType_t delay= 50/portTICK_PERIOD_MS;
        vTaskDelay(delay);
        senzor.read_gyro();
       printf("%f,%f,%f\n", senzor.gyro_data[0],senzor.gyro_data[1],senzor.gyro_data[2]);
    }
    senzor.calib_gyro(0,-50,0,-50,0,-50);
    /*
    while(1)
    {
        const TickType_t delay= 50/portTICK_PERIOD_MS;
        vTaskDelay(delay);
        senzor.read_acc();
       printf("%f,%f,%f\n", senzor.accel_data[0],senzor.accel_data[1],senzor.accel_data[2]);

    }
    senzor.calib_acc(0,-50,0,-50,0,-50);
    */

    /*
    void calib_mag();
    uint8_t AK8963_whoami();
    uint8_t get_CNTL1();
    void read_mag();
    void read_all();
    void calibrate(float *dest1, float *dest2);
    */

    //return 0;
}