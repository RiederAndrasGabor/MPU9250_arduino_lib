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
    senzor.WriteReg(0x1A,0x11);
    /* Alábbi függvények helyesen működnek
    senzor.ReadReg(0x1A,0);
    uint8_t RBuf[8];
    senzor.ReadRegs(0x1A, RBuf,8);
    
    AccelometerScale scale=BITSFS_16G;
    senzor.set_acc_scale(scale);

    GyroScale scale2=BITSFS_250; 
    senzor.set_gyro_scale(scale2); 

    senzor.set_gyro_scale(scale); 
    senzor.whoami();
    senzor.calib_acc();
    senzor.read_acc();
    senzor.read_gyro();
    senzor.read_gyro();*/
    

    /*hiányoznak még:
    void calib_mag();
    uint8_t AK8963_whoami();
    uint8_t get_CNTL1();
    void read_mag();
    void read_all();
    void calibrate(float *dest1, float *dest2);
    */

    //return 0;
}