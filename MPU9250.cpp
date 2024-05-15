#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "MPU9250.h"
#include <ctime>
#include "esp_err.h"

/**
 * \brief    Blokkoló késleltetést megvalósító függvény
 */

int delay(int milliseconds) //blokkoló késleltetés
{
     clock_t goal = milliseconds + clock();
     while(goal>clock());
     return 1;
}


/**
 *  SPI inicializálását elvégző függvény
 *  meghívás esetén először beállítja a megfelelő használt portokat,
 * ezt követően pedig a kommunikáció konfigurálása következik
 */

void MPU9250::spiinitialize()
{
    // SPI busz inicializálása
    spi_bus_config_t buscfg={
        .mosi_io_num = 25,
        .miso_io_num = 34, 
        .sclk_io_num = 32,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
        };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, 0));
    //SPI konfigurációja
    spi_device_interface_config_t devcfg={
         .mode = 0,                  //SPI mode 0
        .clock_speed_hz = 250000,  // 0,5 MHz
        .spics_io_num = 14,         // CS Pin
          .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_dev_mpu9250));
    return;
}

/**
 * \brief    SPI busz konfigurációjának paraméterei itt állíthatóak
 */

void MPU9250::busconfig(int mosi_io_num, int miso_io_num, int sclk_io_num,int quadwp_io_num, int quadhd_io_num, int max_transfer_sz)
{
     spi_bus_config_t buscfg={
        .mosi_io_num = mosi_io_num,
        .miso_io_num = miso_io_num, 
        .sclk_io_num = sclk_io_num,
        .quadwp_io_num = quadwp_io_num,
        .quadhd_io_num = quadhd_io_num,
        .max_transfer_sz = max_transfer_sz,
        };
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, 0));
        return;
}

/**
 * \brief    SPI busz inicializálásának paraméterei itt állíthatóak 
 */

void MPU9250::businitialize(uint8_t mode, int clock_speed_hz, int spics_io_num, uint32_t  flags, int queue_size , transaction_cb_t pre_cb, transaction_cb_t  post_cb)
{
    spi_device_interface_config_t devcfg={
         .mode = mode,                  //SPI mode 0
        .clock_speed_hz = clock_speed_hz,  // 0,5 MHz
        .spics_io_num = spics_io_num,         // CS Pin
          .flags = flags,
        .queue_size = queue_size,
        .pre_cb = pre_cb,
        .post_cb = post_cb,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_dev_mpu9250));
}


/**
 *  regiszter írása
*  Meg kell adni a címet, amire írni szeretnénk, illetve az adatot
 *  0-val kell kezdődnie az adatnak, a 2 bájtos adat a regiszterbe kerül
 *  visszatérési érték: a regiszter tartalma (ellenőrzésképpen)
 */

unsigned int MPU9250::WriteReg( uint8_t WriteAddr, uint8_t WriteData )
{
    uint8_t a=0b01111111;
    WriteAddr=(WriteAddr&a);
    uint8_t tx_data[2] = { WriteAddr, WriteData};
    uint8_t rx_data[2] = { 0xFF, 0xFF};
    spi_transaction_t t = {
        .length = 2*8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };  
    // kiírásra került az adat
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_dev_mpu9250, &t));
    return(rx_data[1]);
}


/**
 *  regiszter olvasása
 *  Meg kell adni a címet, amiről olvasni szeretnénk, illetve az adatot
 *  1-gyel kell kezdődnie az adatnak, 2 bájtot olvasunk ki a megadott regiszterből
 *  visszatérési érték: a regiszter tartalma
 */

unsigned int  MPU9250::ReadReg( uint8_t WriteAddr, uint8_t WriteData )
{
    uint8_t a=0b10000000;
    WriteAddr=(WriteAddr|a);
    uint8_t tx_data[2] = { WriteAddr, WriteData};
    uint8_t rx_data[2] = { 0xFF, 0xFF};
    spi_transaction_t t = {
        .length = 2*8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };  
    // kiírásra került az adat
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_dev_mpu9250, &t));
    printf("Received data: %d, %d\n",rx_data[0],rx_data[1]);
    return(rx_data[1]);
    // uint8_t a=0b10000000;
    // WriteAddr=(WriteAddr|a);
    // // uint8_t a=0b01111111;
    // // WriteAddr=(WriteAddr&a);
    // uint8_t tx_data[2] = { WriteAddr, WriteData};
    // uint8_t rx_data[2] = { 0xFF, 0xFF};
    // spi_transaction_t t = {
    //     .length = 2*8,
    //     .tx_buffer = tx_data,
    //     .rx_buffer = rx_data
    // };  
    // printf("Asked data: %d \n",rx_data[1]);
    // ESP_ERROR_CHECK(spi_device_polling_transmit(spi_dev_mpu9250, &t));
    // return(rx_data[1]);
}


/**
 *  több regiszter olvasása
 *  Meg kell adni a címet, egy tömböt amelybe az eredményt rögzítjük, illetve a regiszterek számát
 *  1-gyel kell kezdődnie az adatnak, 2-2 bájtot olvasunk ki a megadott regiszterekből sorjában
 */

void MPU9250::ReadRegs( uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes )
{
    unsigned int  i = 0;
    for(i = 0; i < Bytes; i++)
    {
        uint8_t a=0b10000000;
        uint8_t b=0x00;
        a =((ReadAddr+i)|a);
        uint8_t tx_data[2] = { a, b };
        uint8_t rx_data[2] = { 0xFF, 0xFF};
    
    spi_transaction_t t = {
        .length = 2*8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
     };  
     
     ESP_ERROR_CHECK(spi_device_polling_transmit(spi_dev_mpu9250, &t));
     ReadBuf[i] = rx_data[1];
    //  printf("Az i. regiszter értékei: %d\n", rx_data[1]);
    }
}


/*                                     INITIALIZATION
 * usage: call this function at startup, giving the sample rate divider (raging from 0 to 255) and
 * low pass filter value; suitable values are:
 * BITS_DLPF_CFG_256HZ_NOLPF2
 * BITS_DLPF_CFG_188HZ
 * BITS_DLPF_CFG_98HZ
 * BITS_DLPF_CFG_42HZ
 * BITS_DLPF_CFG_20HZ
 * BITS_DLPF_CFG_10HZ 
 * BITS_DLPF_CFG_5HZ 
 * BITS_DLPF_CFG_2100HZ_NOLPF
 * returns 1 if an error occurred
 */

#define MPU_InitRegNum 17

bool MPU9250::init(bool calib_gyro, bool calib_acc){
/*    //pinMode(my_cs, OUTPUT);
#ifdef CORE_TEENSY
    digitalWriteFast(my_cs, HIGH);
#else
    //digitalWrite(my_cs, HIGH);
    my_cs=1;
#endif*/
    float temp[3];

    if(calib_gyro && calib_acc){
        calibrate(g_bias, a_bias);
    }
    else if(calib_gyro){
        calibrate(g_bias, temp);
    }
    else if(calib_acc){
        calibrate(temp, a_bias);
    }
    
    uint8_t i = 0;
    uint8_t MPU_Init_Data[MPU_InitRegNum][2] = { //[17][2]
        {BIT_H_RESET, MPUREG_PWR_MGMT_1},        // Reset Device
        {0x01, MPUREG_PWR_MGMT_1},               // Clock Source
        {0x00, MPUREG_PWR_MGMT_2},               // Enable Acc & Gyro
        {my_low_pass_filter, MPUREG_CONFIG},     // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
        {BITS_FS_250DPS, MPUREG_GYRO_CONFIG},    // +-250dps
        {BITS_FS_2G, MPUREG_ACCEL_CONFIG},       // +-2G
        {my_low_pass_filter_acc, MPUREG_ACCEL_CONFIG_2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
        {0x12, MPUREG_INT_PIN_CFG},      //
        //{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
        //{0x20, MPUREG_USER_CTRL},      // Enable AUX
        {0x30, MPUREG_USER_CTRL},        // I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
        {0x0D, MPUREG_I2C_MST_CTRL},     // I2C configuration multi-master  IIC 400KHz
        
        {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  // Set the I2C slave addres of AK8963 and set for write.
        //{0x09, MPUREG_I2C_SLV4_CTRL},
        //{0x81, MPUREG_I2C_MST_DELAY_CTRL}, // Enable I2C delay

        {AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, // I2C slave 0 register address from where to begin data transfer
        {0x01, MPUREG_I2C_SLV0_DO},   // Reset AK8963
        {0x81, MPUREG_I2C_SLV0_CTRL}, // Enable I2C and set 1 byte

        {AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, // I2C slave 0 register address from where to begin data transfer
#ifdef AK8963FASTMODE
        {0x16, MPUREG_I2C_SLV0_DO},   // Register value to 100Hz continuous measurement in 16bit
#else
        {0x12, MPUREG_I2C_SLV0_DO},   // Register value to 8Hz continuous measurement in 16bit
#endif
        {0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte
        
    };
    for(i = 0; i < MPU_InitRegNum; i++) {
        WriteReg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
        //delayMicroseconds(1000);  // I2C must slow down the write speed, otherwise it won't work
        delay(1);
    }
    Accelometer_Scale scale1=BITSFS_2G;
    set_acc_scale(scale1);
    Gyro_Scale scale2=BITSFS_250;
    set_gyro_scale(scale2);
    calib_mag();  // If experiencing problems here, just comment it out. Should still be somewhat functional.
    return 0;
}

/**                                Gyorsulásmérő skálázása
 *
 * 2,4,8 és 16 G értékek beállítására ad lehetőséget FS tartományként a gyorsulásmérőnek
 * visszatérési értéke a ténylegesen beállított skála
 */

unsigned int MPU9250::set_acc_scale(Accelometer_Scale scale){
    int scale1=(int)scale;
    unsigned int temp_scale;
    WriteReg(MPUREG_ACCEL_CONFIG, scale1);
    switch(scale1){
        case BITS_FS_2G: acc_divider=16384; break;
        case BITS_FS_4G: acc_divider=8192;  break;
        case BITS_FS_8G: acc_divider=4096;  break;
        case BITS_FS_16G: acc_divider=2048; break;   
    }
    temp_scale = WriteReg(MPUREG_ACCEL_CONFIG|READ_FLAG, 0x00);
    switch (temp_scale){
        case BITS_FS_2G:  temp_scale=2;     break;
        case BITS_FS_4G:  temp_scale=4;     break;
        case BITS_FS_8G:  temp_scale=8;    break;
        case BITS_FS_16G: temp_scale=16;    break;   
    }
    return temp_scale;
    // a_scale = WriteReg(MPUREG_GYRO_CONFIG|READ_FLAG, 0x00);
    // switch (g_scale){
    //     case BITS_FS_250DPS:   g_scale = 250; g_scale_g=2;   break;
    //     case BITS_FS_500DPS:   g_scale = 500; g_scale_g=4;   break;
    //     case BITS_FS_1000DPS:  g_scale = 1000;g_scale_g=8;   break;
    //     case BITS_FS_2000DPS:  g_scale = 2000;g_scale_g=16;   break;   
    // }
    // return g_scale;
}



/**                                giroszkóp skálázása
 *
 * 250,500,1000 és 2000 DPS értékek beállítására ad lehetőséget FS tartományként a giroszkópnak
 * visszatérési értéke a ténylegesen beállított skála
 */

unsigned int MPU9250::set_gyro_scale(Gyro_Scale scale){
    
    WriteReg(MPUREG_GYRO_CONFIG, 24);
    switch (scale){
        case BITSFS_250:   gyro_divider = 131;  break;
        case BITSFS_500:   gyro_divider = 65.5; break;
        case BITSFS_1000:  gyro_divider = 32.8; break;
        case BITSFS_2000:  gyro_divider = 16.4; break;   
    }
    g_scale = WriteReg(MPUREG_GYRO_CONFIG|READ_FLAG, 0x00);
    switch (g_scale){
        case BITS_FS_250DPS:   g_scale = 250; g_scale_g=2;   break;
        case BITS_FS_500DPS:   g_scale = 500; g_scale_g=4;   break;
        case BITS_FS_1000DPS:  g_scale = 1000;g_scale_g=8;   break;
        case BITS_FS_2000DPS:  g_scale = 2000;g_scale_g=16;   break;   
    }
    return g_scale;
}



/**                                 WHO AM I?
 * SPI tesztelésére használható, a WHOAMI regiszter  érétkét adja vissza
 * Megfelelő esetben 0x73 (113)-mal tér vissza (Habár adatlap szerint 71-gyel kellene)
 */

unsigned int MPU9250::whoami(){
    unsigned int response;
    response = ReadReg(MPUREG_WHOAMI, 0x00);
    return response;
}



/*                                 gyorsulásmérő kiolvasása
 * A függvény kigyűjti a gyorsulásmérő adta aktuális adatokat. (3 (x,y,z)*2 (alsó, és felső) bájt)
 * Ezt követően a accel_data[0,1,2] változóba x,y,z koordináták szerint rendezi az adatokat.
 */

void MPU9250::read_acc()
{
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    ReadRegs(MPUREG_ACCEL_XOUT_H,response,6);
    for(i = 0; i < 3; i++) {
        bit_data = ((int16_t)response[i*2]<<8)|response[i*2+1];
        data = (float)bit_data;
        accel_data[i] = data/acc_divider - a_bias[i];
       // gyro_data[i] = (data*a_scale_g)/(acc_divider*a_scale)- a_bias[i];
    }   
}

/*                                 Giroszkóp kiolvasása
 * A függvény kigyűjti a giroszkóp adta aktuális adatokat. (3 (x,y,z)*2 (alsó, és felső) bájt)
 * Ezt követően a gyro_data[0,1,2] változóba x,y,z koordináták szerint rendezi az adatokat.
 */

void MPU9250::read_gyro()
{
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    ReadRegs(MPUREG_GYRO_XOUT_H,response,6);
    for(i = 0; i < 3; i++) {
        bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
        data = (float)bit_data;
        gyro_data[i] = (data*g_scale_g)/(gyro_divider*g_scale)- g_bias[i];
    }
}


void MPU9250::calib_acc(float XA, float YA, float ZA)
{
    a_bias[0] = XA;  
    a_bias[1] = YA;
    a_bias[2] = ZA;
}
void MPU9250::auto_calib_acc() 
{
    int ii;
    float datas[3]={0.0,0.0,0.0};
    for (ii = 0; ii < 50; ii++) {
        const TickType_t delay= 50/portTICK_PERIOD_MS;
        vTaskDelay(delay);
        read_acc();
        datas[0]+=accel_data[0];
        datas[1]+=accel_data[1];
        datas[2]+=accel_data[2];           
    }
    a_bias[0] = datas[0]/50;  
    a_bias[1] = datas[1]/50; 
    a_bias[2] = datas[2]/50;
}



void MPU9250::calib_gyro(float XG, float YG, float ZG)
{
    g_bias[0] = XG;  
    g_bias[1] = YG;
    g_bias[2] = ZG;
}

void MPU9250::auto_calib_gyro() 
{
    int ii;
    float datas[3]={0.0,0.0,0.0};
    for (ii = 0; ii < 50; ii++) {
        const TickType_t delay= 50/portTICK_PERIOD_MS;
        vTaskDelay(delay);
        read_gyro();
        datas[0]+=gyro_data[0];
        datas[1]+=gyro_data[1];
        datas[2]+=gyro_data[2];           
    }
    g_bias[0] = datas[0]/50;  
    g_bias[1] = datas[1]/50; 
    g_bias[2] = datas[2]/50;
}

unsigned int MPU9250::set_mag_scale(Magneto_Scale scale){
 float temp_scale=0.0;
  switch (scale)
  {
    
   // Possible magnetometer scales (and their register bit settings) are:
  // 14 bit resolution (0) and 16 bit resolution (1)
    case BITSFS_14:
          temp_scale = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case BITSFS_16:
          temp_scale = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
    
  }
  return temp_scale;
}


void MPU9250::calib_mag(){
    uint8_t response[3];
    float data;
    int i;
    // Choose either 14-bit or 16-bit magnetometer resolution
    //uint8_t MFS_14BITS = 0; // 0.6 mG per LSB
    uint8_t MFS_16BITS =1; // 0.15 mG per LSB
    // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
    uint8_t M_8HZ = 0x02; // 8 Hz update
    //uint8_t M_100HZ = 0x06; // 100 Hz continuous magnetometer

    /* get the magnetometer calibration */

    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG);   // Set the I2C slave    addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_ASAX);                 // I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x83);                       // Read 3 bytes from the magnetometer

    //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);                     // Enable I2C and set bytes
    //delayMicroseconds(100000);  
    delay(100);
    //response[0]=WriteReg(MPUREG_EXT_SENS_DATA_01|READ_FLAG, 0x00); //Read I2C 

    WriteReg(AK8963_CNTL1, 0x00);                               // set AK8963 to Power Down
    //delayMicroseconds(50000);   
    delay(50);                                               // long wait between AK8963 mode changes
    WriteReg(AK8963_CNTL1, 0x0F);                               // set AK8963 to FUSE ROM access
    //delayMicroseconds(50000);    
    delay(50);                                              // long wait between AK8963 mode changes

    ReadRegs(MPUREG_EXT_SENS_DATA_00,response,3);
    //response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);              // Read I2C 
    for(i = 0; i < 3; i++) {
        data=response[i];
        Magnetometer_ASA[i] = ((data-128)/256+1)*Magnetometer_Sensitivity_Scale_Factor;
    }
    WriteReg(AK8963_CNTL1, 0x00); // set AK8963 to Power Down
    //delayMicroseconds(50000);
    delay(50);  
    // Configure the magnetometer for continuous read and highest resolution.
    // Set bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL
    // register, and enable continuous mode data acquisition (bits [3:0]),
    // 0010 for 8 Hz and 0110 for 100 Hz sample rates.   
    WriteReg(AK8963_CNTL1, MFS_16BITS << 4 | M_8HZ);            // Set magnetometer data resolution and sample ODR
    //delayMicroseconds(50000);
    delay(50);  
}

void MPU9250::read_mag(){
    uint8_t response[7];
    float data;
    int i;
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG);  // Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL);                 // I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87);                      // Read 6 bytes from the magnetometer

    // delayMicroseconds(1000);
    const TickType_t delay= 1000/portTICK_PERIOD_MS;
    vTaskDelay(delay);
    ReadRegs(MPUREG_EXT_SENS_DATA_00,response,7);
    // must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
    for(i = 0; i < 3; i++) {
        mag_data_raw[i] = ((int16_t)response[i*2+1]<<8)|response[i*2];
        data = (float)mag_data_raw[i];
        mag_data[i] = data*Magnetometer_ASA[i];
    }
}

uint8_t MPU9250::AK8963_whoami(){
    uint8_t response;
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_WIA); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer
    // ReadReg(MPUREG_I2C_SLV0_ADDR,0x00); //Set the I2C slave addres of AK8963 and set for read.
    // ReadReg(MPUREG_I2C_SLV0_REG, 0x00); //I2C slave 0 re gister address from where to begin data transfer
    // ReadReg(MPUREG_I2C_SLV0_CTRL, 0x00); //Read 1 byte from the magnetometer
    const TickType_t delay= 1000/portTICK_PERIOD_MS;
    vTaskDelay(delay);

    response=WriteReg(MPUREG_EXT_SENS_DATA_00,0x00 );    //Read I2C 
    ReadReg(MPUREG_EXT_SENS_DATA_00, 0x00);
    return response;
}

uint8_t MPU9250::get_CNTL1(){
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_CNTL1 );              // I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer
    const TickType_t delay= 1000/portTICK_PERIOD_MS;
    vTaskDelay(delay);
    // delayMicroseconds(1000);
    uint8_t RBuf[18];
    ReadRegs(AK8963_WIA, RBuf,18);
    for(int i=0; 18>i;i++)
    {
    printf(" Az i. regiszter értéke: %d\n",RBuf[i]);
    }
        return WriteReg(MPUREG_EXT_SENS_DATA_00|READ_FLAG, 0x00);    //Read I2C 
} 
 

void MPU9250::read_all(){
    uint8_t response[21];
    int16_t bit_data;
    float data;
    int i;

    // Send I2C command at first
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL);                // I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87);                     // Read 7 bytes from the magnetometer
    // must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.

    ReadRegs(MPUREG_ACCEL_XOUT_H,response,21);
    // Get accelerometer value
    for(i = 0; i < 3; i++) {
        bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
        data = (float)bit_data;
        accel_data[i] = data/acc_divider - a_bias[i];
    }
    // Get gyroscope value
    for(i=4; i < 7; i++) {
        bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
        data = (float)bit_data;
        gyro_data[i-4] = data/gyro_divider - g_bias[i-4];
    }
    // Get Magnetometer value
    for(i=7; i < 10; i++) {
        mag_data_raw[i-7] = ((int16_t)response[i*2+1]<<8) | response[i*2];
        data = (float)mag_data_raw[i-7];
        mag_data[i-7] = data * Magnetometer_ASA[i-7];
    }
}

void MPU9250::calibrate(float *dest1, float *dest2 /*m/s^2-ben adjuk meg !!! */){  
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
    // reset device
    WriteReg(MPUREG_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    delay(100);
   
    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
    // else use the internal oscillator, bits 2:0 = 001
    WriteReg(MPUREG_PWR_MGMT_1, 0x01);  
    WriteReg(MPUREG_PWR_MGMT_2, 0x00);
    delay(200);                                    

    // Configure device for bias calculation
    WriteReg(MPUREG_INT_ENABLE, 0x00);   // Disable all interrupts
    WriteReg(MPUREG_FIFO_EN, 0x00);      // Disable FIFO
    WriteReg(MPUREG_PWR_MGMT_1, 0x00);   // Turn on internal clock source
    WriteReg(MPUREG_I2C_MST_CTRL, 0x00); // Disable I2C master
    WriteReg(MPUREG_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    WriteReg(MPUREG_USER_CTRL, 0x0C);    // Reset FIFO and DMP
    delay(15);
  
    // Configure MPU6050 gyro and accelerometer for bias calculation
    WriteReg(MPUREG_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    WriteReg(MPUREG_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    WriteReg(MPUREG_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    WriteReg(MPUREG_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
    
    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g
    
    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    WriteReg(MPUREG_USER_CTRL, 0x40);   // Enable FIFO  
    WriteReg(MPUREG_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    WriteReg(MPUREG_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    ReadRegs(MPUREG_FIFO_COUNTH, data, 2); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
    
    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        ReadRegs(MPUREG_FIFO_R_W, data, 12); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
        
        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
            
    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) accelsensitivity;}
   
    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
    // Push gyro biases to hardware registers
    WriteReg(MPUREG_XG_OFFS_USRH, data[0]);
    WriteReg(MPUREG_XG_OFFS_USRL, data[1]);
    WriteReg(MPUREG_YG_OFFS_USRH, data[2]);
    WriteReg(MPUREG_YG_OFFS_USRL, data[3]);
    WriteReg(MPUREG_ZG_OFFS_USRH, data[4]);
    WriteReg(MPUREG_ZG_OFFS_USRL, data[5]);
  
    // Output scaled gyro biases for display in the main program
    dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
    dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    ReadRegs(MPUREG_XA_OFFSET_H, data, 2); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    ReadRegs(MPUREG_YA_OFFSET_H, data, 2);
    accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    ReadRegs(MPUREG_ZA_OFFSET_H, data, 2);
    accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    
    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
    
    for(ii = 0; ii < 3; ii++) {
      if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }
    
    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);
  
    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
 
// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
    WriteReg(MPUREG_XA_OFFSET_H, data[0]);
    WriteReg(MPUREG_XA_OFFSET_L, data[1]);
    WriteReg(MPUREG_YA_OFFSET_H, data[2]);
    WriteReg(MPUREG_YA_OFFSET_L, data[3]);
    WriteReg(MPUREG_ZA_OFFSET_H, data[4]);
    WriteReg(MPUREG_ZA_OFFSET_L, data[5]);

// Output scaled accelerometer biases for display in the main program
    dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
    dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
    dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}