#include "main.h"

    #define   X  0

    #define   Y  1

    #define   Z  2

    #define on   1

    #define off  0


#define FsDiv8          7       // Fs/(1+7)
#define FsDiv7          6       // Fs/(1+6)
#define FsDiv6          5       // Fs/(1+5)
#define FsDiv5          4       // Fs/(1+4)
#define FsDiv4          3       // Fs/(1+3)
#define FsDiv3          2       // Fs/(1+2)
#define FsDiv2          1       // Fs/(1+1)
#define FsDiv1          0       // Fs/(1+0)
#define SampleRateDiv   FsDiv4
// You can find the Fs value from the DLPF section

#define EXT_SYNC_SET_Dis    0   // Input disabled
#define EXT_SYNC_SET_Temp   1   // TEMP_OUT_L[0]
#define EXT_SYNC_SET_GX     2   // GYRO_XOUT_L[0]
#define EXT_SYNC_SET_GY     3   // GYRO_YOUT_L[0]
#define EXT_SYNC_SET_GZ     4   // GYRO_ZOUT_L[0]
#define EXT_SYNC_SET_AX     5   // ACCEL_XOUT_L[0]
#define EXT_SYNC_SET_AY     6   // ACCEL_YOUT_L[0]
#define EXT_SYNC_SET_AZ     7   // ACCEL_ZOUT_L[0]
#define EXT_SYNC_SET        EXT_SYNC_SET_Dis

                       // Accelerometer    |         Gyroscope             |
                       // (Fs = 1kHz)      |                               |
                       // BW(Hz) Delay(ms) |   BW(Hz)    Delay(ms) Fs(kHz) |
#define DLPF_CFG_0     0 // 260    0       |    256         0.98      8    |
#define DLPF_CFG_1     1 // 184   2.0      |    188         1.9       1    |
#define DLPF_CFG_2     2 // 94    3.0      |    98          2.8       1    |
#define DLPF_CFG_3     3 // 44    4.9      |    42          4.8       1    |
#define DLPF_CFG_4     4 // 21    8.5      |    20          8.3       1    |
#define DLPF_CFG_5     5 // 10    13.8     |    10          13.4      1    |
#define DLPF_CFG_6     6 // 5     19.0     |    5           18.6      1    |
#define DLPF_CFG     DLPF_CFG_1

#define GYRO_FullScale_Range_250     0x00
#define GYRO_FullScale_Range_500     0x08
#define GYRO_FullScale_Range_1000    0x10
#define GYRO_FullScale_Range_2000    0x18
#define GFS_SEL                      GYRO_FullScale_Range_500

#define XG_ST    on  // Active Or Deactive self-test for X gyroscope axis
#define YG_ST    on  // Active Or Deactive self-test for Y gyroscope axis
#define ZG_ST    on  // Active Or Deactive self-test for Z gyroscope axis

#define GYRO_Sensitivity_Range_250   131.0   // LSB/(º/s)
#define GYRO_Sensitivity_Range_500   65.5    // LSB/(º/s)
#define GYRO_Sensitivity_Range_1000  32.8    // LSB/(º/s)
#define GYRO_Sensitivity_Range_2000  16.4    // LSB/(º/s)

  #if   GFS_SEL == GYRO_FullScale_Range_250
#define GYRO_Sensitivity       GYRO_Sensitivity_Range_250
  #elif GFS_SEL == GYRO_FullScale_Range_500
#define GYRO_Sensitivity       GYRO_Sensitivity_Range_500
  #elif GFS_SEL == GYRO_FullScale_Range_1000
#define GYRO_Sensitivity       GYRO_Sensitivity_Range_1000
  #elif GFS_SEL == GYRO_FullScale_Range_2000
#define GYRO_Sensitivity       GYRO_Sensitivity_Range_2000
  #endif

#define ACCEL_FullScale_Range_2g     0x00     // 0
#define ACCEL_FullScale_Range_4g     0x08     // 1
#define ACCEL_FullScale_Range_8g     0x10     // 2
#define ACCEL_FullScale_Range_16g    0x18     // 3
#define AFS_SEL                      ACCEL_FullScale_Range_8g

#define XA_ST    on  // Active Or Deactive self-test for X accelerometer axis
#define YA_ST    on  // Active Or Deactive self-test for Y accelerometer axis
#define ZA_ST    on  // Active Or Deactive self-test for Z accelerometer axis

#define ACCEL_Sensitivity_Range_2g   16384   // LSB/g
#define ACCEL_Sensitivity_Range_4g   8192    // LSB/g
#define ACCEL_Sensitivity_Range_8g   4096    // LSB/g
#define ACCEL_Sensitivity_Range_16g  2048    // LSB/g

    #if AFS_SEL == ACCEL_FullScale_Range_2g
#define ACCEL_Sensitivity                   ACCEL_Sensitivity_Range_2g
  #elif AFS_SEL == ACCEL_FullScale_Range_4g
       #define ACCEL_Sensitivity            ACCEL_Sensitivity_Range_4g
  #elif AFS_SEL == ACCEL_FullScale_Range_8g
       #define ACCEL_Sensitivity            ACCEL_Sensitivity_Range_8g
  #elif AFS_SEL == ACCEL_FullScale_Range_16g
       #define ACCEL_Sensitivity            ACCEL_Sensitivity_Range_16g
  #endif

#define TEMP_Sensitivity    340.0   // LSB/ºC

#define TEMP_FIFO_En        0x80
#define X_GYRO_FIFO_En      0x40
#define Y_GYRO_FIFO_En      0x20
#define Z_GYRO_FIFO_En      0x10
#define ACCEL_FIFO_En       0x08
#define SLV2_FIFO_En        0x04
#define SLV1_FIFO_En        0x02
#define SLV0_FIFO_En        0x01
#define ANY_FIFO_Bit_Dis    0
#define FIFO_En_Parameters  ANY_FIFO_Bit_Dis//Z_GYRO_FIFO_En

#define Int8MHz              0
#define PLL_XG_ref           1
#define PLL_YG_ref           2
#define PLL_ZG_ref           3
#define PLL_ext32768Hz_ref   4
#define PLL_ext19200KHz_ref  5
#define Stop                 7
#define CLKSEL               PLL_ZG_ref//PLL_XG_ref

#define TEMP_DIS        off  // When set to 1, this bit disables the temperature sensor
#define CYCLE           off  // When this bit is set to 1 and SLEEP is disabled, the MPU-6050 will cycle between sleep mode and waking up to take a single sample of data from active sensors at a rate determined by LP_WAKE_CTRL (register 108).
//#define SLEEP           off  // When set to 1, this bit puts the MPU-60X0 into sleep mode.
#define DEVICE_RESET    off  // When set to 1, this bit resets all internal registers to their default values.The bit automatically clears to 0 once the reset is done.The default values for each register can be found in RA_MPU6050.h.

#define LP_WAKE_CTRL_1_25Hz   0 // Wake-up Frequency : 1.25 Hz
#define LP_WAKE_CTRL_5Hz      1 // Wake-up Frequency : 5 Hz
#define LP_WAKE_CTRL_20Hz     2 // Wake-up Frequency : 20 Hz
#define LP_WAKE_CTRL_40Hz     3 // Wake-up Frequency : 40 Hz
#define LP_WAKE_CTRL          LP_WAKE_CTRL_40Hz//LP_WAKE_CTRL_1_25Hz

#define STBY_XA   off  // When set to 1, this bit puts the X axis accelerometer into standby mode.
#define STBY_YA   off  // When set to 1, this bit puts the Y axis accelerometer into standby mode.
#define STBY_ZA   off  // When set to 1, this bit puts the Z axis accelerometer into standby mode.
#define STBY_XG   off  // When set to 1, this bit puts the X axis gyroscope into standby mode.
#define STBY_YG   off  // When set to 1, this bit puts the Y axis gyroscope into standby mode.
#define STBY_ZG   off  // When set to 1, this bit puts the Z axis gyroscope into standby mode.


