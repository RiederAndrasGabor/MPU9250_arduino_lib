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
    

    // // // /*     ALAP FGV TESZTELÉS
    senzor.init(true,true);
    // senzor.initialize_acc_and_gyro(BITSFS_16G, BITSFS_500, true,false);
    // senzor.whoami();
    // senzor.WriteReg(0x1A,0x0A); 
    // senzor.WriteReg(0x1C,0x10); 
    // senzor.ReadReg(0x1A,0x2A); 
    // senzor.ReadReg(0x1C,0X00); 
    // uint8_t RBuf[3];
    // senzor.ReadRegs(0x1A, RBuf,3);
    // printf("%d\n%d\n%d\n",RBuf[0],RBuf[1],RBuf[2]);
    // // // */
   

    // // // /*
    // // //     -set gyro dlpf 
    // // //     -scale,offset egybe struktűraként mindenes fgv
    // // // */
    

    // // /*GYRO TESZTELÉS
    // Gyro_Scale scale2=BITSFS_250; 
    // senzor.set_gyro_scale(scale2); 
    // senzor.calib_gyro(2,0,-10); 
    // //senzor.auto_calib_gyro(); 
    // while(1)
    // { 
    //     const TickType_t delay= 50/portTICK_PERIOD_MS;
    //     vTaskDelay(delay);
    //     senzor.read_gyro();
    //     printf("%f,%f,%f\n", senzor.gyro_data[0],senzor.gyro_data[1],senzor.gyro_data[2]);
    // }
    // // // */

    // // // /*     ACC FGV TESZTELÉS
    // Accelometer_Scale scale=BITSFS_16G;
    // senzor.set_acc_scale(scale);
    // senzor.auto_calib_acc();
    // //senzor.calib_acc(0,-10,4);
    //  const TickType_t delay2= 1000/portTICK_PERIOD_MS;
    //     vTaskDelay(delay2);
    // while(1)
    // {
    //     const TickType_t delay= 50/portTICK_PERIOD_MS;
    //     vTaskDelay(delay);
    //     senzor.read_acc();
    //    printf("%f,%f,%f\n", senzor.accel_data[0],senzor.accel_data[1],senzor.accel_data[2]);
        
    // }
    // // // */
    

    // // // /*     MAG FGV TESZTELÉS
    //   Magneto_Scale scale2=BITSFS_16; 
    //     float a =senzor.set_mag_scale(scale2); 
    //     printf(" %f\n",a);
    senzor.AK8963_whoami(); 
    senzor.get_CNTL1(); 
     while(1)
    {
        const TickType_t delay= 50/portTICK_PERIOD_MS;
        vTaskDelay(delay);
        senzor.read_mag(); 
       printf("%f,%f,%f\n", senzor.mag_data[0],senzor.mag_data[1],senzor.mag_data[2]);
        
    }
    

    // void calib_mag();
    // // // */


    // // // /*     MARADÉK FGV TESZTELÉS EZEK NEM JÓK MÉG
    // uint8_t ret= senzor.AK8963_whoami();
    // uint8_t ret= senzor.get_CNTL1();
    // printf("visszaadott érték: %d\n",ret);
    // void read_mag();
    // void read_all();
    // void calibrate(float *dest1, float *dest2);
    // // // */
    
}