#include "stm32f4xx.h"

#define MPU6050_ADDRESS 0xD0 // 0xD0 : 0b110100(ad0=0)0------>0xD0
// MPU6050 REGISTER ADDRESS MAP:
// The reset value is 0x00 for all registers other than the registers below.
//  Register PWR_MGMT_1 (0x6B): 0x40.
//  Register WHO_AM_I   (0x75): 0x68.

//    Register Name    Addr(Dec.) Serial I/F    Bit7     Bit6     Bit5     Bit4     Bit3     Bit2     Bit1     Bit0
#define RA_XG_OFFS_TC 0 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define RA_YG_OFFS_TC 1 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define RA_ZG_OFFS_TC 2 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD

#define RA_X_FINE_GAIN 3 // R/W   X_FINE_GAIN[7:0]
#define RA_Y_FINE_GAIN 4 // R/W   Y_FINE_GAIN[7:0]
#define RA_Z_FINE_GAIN 5 // R/W   Z_FINE_GAIN[7:0]

#define RA_XA_OFFS_H 6     // R/W   XA_OFFS[15:0]
#define RA_XA_OFFS_L_TC 7  // R/W
#define RA_YA_OFFS_H 8     // R/W   YA_OFFS[15:0]
#define RA_YA_OFFS_L_TC 9  // R/W
#define RA_ZA_OFFS_H 10    // R/W   ZA_OFFS[15:0]
#define RA_ZA_OFFS_L_TC 11 // R/W

#define RA_SELF_TEST_X 13 // R/W  XA_TEST[4-2]     XG_TEST[4-0]
#define RA_SELF_TEST_Y 14 // R/W  YA_TEST[4-2]     YG_TEST[4-0]
#define RA_SELF_TEST_Z 15 // R/W  ZA_TEST[4-2]     ZG_TEST[4-0]
#define RA_SELF_TEST_A 16 // R/W    RESERVED       XA_TEST[1-0]     YA_TEST[1-0]     ZA_TEST[1-0]

#define RA_XG_OFFS_USRH 19 // R/W   XG_OFFS_USR[15:0]
#define RA_XG_OFFS_USRL 20 // R/W
#define RA_YG_OFFS_USRH 21 // R/W   YG_OFFS_USR[15:0]
#define RA_YG_OFFS_USRL 22 // R/W
#define RA_ZG_OFFS_USRH 23 // R/W   ZG_OFFS_USR[15:0]
#define RA_ZG_OFFS_USRL 24 // R/W

#define RA_SMPLRT_DIV 25 // R/W  SMPLRT_DIV[7:0]

#define RA_CONFIG 26       // R/W    -     -     EXT_SYNC_SET[2:0]     DLPF_CFG[2:0]
#define RA_GYRO_CONFIG 27  // R/W    -     -     -     FS_SEL [1:0]     -     -     -
#define RA_ACCEL_CONFIG 28 // R/W  XA_ST     YA_ST     ZA_ST     AFS_SEL[1:0]    -     -     -

#define RA_FF_THR 29    // R/W   FF_THR[7:0]
#define RA_FF_DUR 30    // R/W   FF_DUR[7:0]
#define RA_MOT_THR 31   // R/W   MOT_THR[7:0]
#define RA_MOT_DUR 32   // R/W   MOT_DUR[7:0]
#define RA_ZRMOT_THR 33 // R/W   ZRMOT_THR[7:0]
#define RA_ZRMOT_DUR 34 // R/W   ZRMOT_DUR[7:0]

#define RA_FIFO_EN 35 // R/W  TEMP _FIFO_EN     XG _FIFO_EN     YG _FIFO_EN     ZG _FIFO_EN     ACCEL _FIFO_EN     SLV2 _FIFO_EN     SLV1 _FIFO_EN     SLV0 _FIFO_EN

#define RA_I2C_MST_CTRL 36   // R/W  MULT _MST_EN     WAIT _FOR_ES     SLV_3 _FIFO_EN     I2C_MST _P_NSR     I2C_MST_CLK[3:0]
#define RA_I2C_SLV0_ADDR 37  // R/W  I2C_SLV0 _RW     I2C_SLV0_ADDR[6:0]
#define RA_I2C_SLV0_REG 38   // R/W  I2C_SLV0_REG[7:0]
#define RA_I2C_SLV0_CTRL 39  // R/W  I2C_SLV0 _EN     I2C_SLV0 _BYTE_SW     I2C_SLV0 _REG_DIS     I2C_SLV0 _GRP     I2C_SLV0_LEN[3:0]
#define RA_I2C_SLV1_ADDR 40  // R/W  I2C_SLV1 _RW     I2C_SLV1_ADDR[6:0]
#define RA_I2C_SLV1_REG 41   // R/W  I2C_SLV1_REG[7:0]
#define RA_I2C_SLV1_CTRL 42  // R/W  I2C_SLV1 _EN     I2C_SLV1 _BYTE_SW     I2C_SLV1 _REG_DIS     I2C_SLV1 _GRP     I2C_SLV1_LEN[3:0]
#define RA_I2C_SLV2_ADDR 43  // R/W  I2C_SLV2 _RW     I2C_SLV2_ADDR[6:0]
#define RA_I2C_SLV2_REG 44   // R/W  I2C_SLV2_REG[7:0]
#define RA_I2C_SLV2_CTRL 45  // R/W  I2C_SLV2 _EN     I2C_SLV2 _BYTE_SW     I2C_SLV2 _REG_DIS     I2C_SLV2 _GRP     I2C_SLV2_LEN[3:0]
#define RA_I2C_SLV3_ADDR 46  // R/W  I2C_SLV3 _RW     I2C_SLV3_ADDR[6:0]
#define RA_I2C_SLV3_REG 47   // R/W  I2C_SLV3_REG[7:0]
#define RA_I2C_SLV3_CTRL 48  // R/W  I2C_SLV3 _EN     I2C_SLV3 _BYTE_SW    I2C_SLV3 _REG_DIS    I2C_SLV3 _GRP    I2C_SLV3_LEN[3:0]
#define RA_I2C_SLV4_ADDR 49  // R/W  I2C_SLV4 _RW    I2C_SLV4_ADDR[6:0]
#define RA_I2C_SLV4_REG 50   // R/W  I2C_SLV4_REG[7:0]
#define RA_I2C_SLV4_DO 51    // R/W  I2C_SLV4_DO[7:0]
#define RA_I2C_SLV4_CTRL 52  // R/W  I2C_SLV4 _EN    I2C_SLV4  _INT_EN    I2C_SLV4 _REG_DIS    I2C_MST_DLY[4:0]
#define RA_I2C_SLV4_DI 53    // R    I2C_SLV4_DI[7:0]
#define RA_I2C_MST_STATUS 54 // R    PASS_ THROUGH    I2C_SLV4 _DONE    I2C_LOST _ARB    I2C_SLV4 _NACK    I2C_SLV3 _NACK    I2C_SLV2 _NACK    I2C_SLV1 _NACK    I2C_SLV0 _NACK

#define RA_INT_PIN_CFG 55 // R/W  INT_LEVEL    INT_OPEN    LATCH _INT_EN    INT_RD _CLEAR    FSYNC_ INT_LEVEL    FSYNC _INT_EN    I2C _BYPASS _EN    -
#define RA_INT_ENABLE 56  // R/W   -    -    -    FIFO _OFLOW _EN    I2C_MST _INT_EN    -    -    DATA _RDY_EN
#define RA_INT_STATUS 58  // R    -    -    -    FIFO _OFLOW _INT    I2C_MST _INT    -    -    DATA _RDY_INT

#define RA_ACCEL_XOUT_H 59 // R    ACCEL_XOUT[15:8]
#define RA_ACCEL_XOUT_L 60 // R    ACCEL_XOUT[7:0]
#define RA_ACCEL_YOUT_H 61 // R    ACCEL_YOUT[15:8]
#define RA_ACCEL_YOUT_L 62 // R    ACCEL_YOUT[7:0]
#define RA_ACCEL_ZOUT_H 63 // R    ACCEL_ZOUT[15:8]
#define RA_ACCEL_ZOUT_L 64 // R    ACCEL_ZOUT[7:0]

#define RA_TEMP_OUT_H 65 // R    TEMP_OUT[15:8]
#define RA_TEMP_OUT_L 66 // R    TEMP_OUT[7:0]

#define RA_GYRO_XOUT_H 67 // R    GYRO_XOUT[15:8]
#define RA_GYRO_XOUT_L 68 // R    GYRO_XOUT[7:0]
#define RA_GYRO_YOUT_H 69 // R    GYRO_YOUT[15:8]
#define RA_GYRO_YOUT_L 70 // R    GYRO_YOUT[7:0]
#define RA_GYRO_ZOUT_H 71 // R    GYRO_ZOUT[15:8]
#define RA_GYRO_ZOUT_L 72 // R    GYRO_ZOUT[7:0]

#define RA_EXT_SENS_DATA_00 73 // R    EXT_SENS_DATA_00[7:0]
#define RA_EXT_SENS_DATA_01 74 // R    EXT_SENS_DATA_01[7:0]
#define RA_EXT_SENS_DATA_02 75 // R    EXT_SENS_DATA_02[7:0]
#define RA_EXT_SENS_DATA_03 76 // R    EXT_SENS_DATA_03[7:0]
#define RA_EXT_SENS_DATA_04 77 // R    EXT_SENS_DATA_04[7:0]
#define RA_EXT_SENS_DATA_05 78 // R    EXT_SENS_DATA_05[7:0]
#define RA_EXT_SENS_DATA_06 79 // R    EXT_SENS_DATA_06[7:0]
#define RA_EXT_SENS_DATA_07 80 // R    EXT_SENS_DATA_07[7:0]
#define RA_EXT_SENS_DATA_08 81 // R    EXT_SENS_DATA_08[7:0]
#define RA_EXT_SENS_DATA_09 82 // R    EXT_SENS_DATA_09[7:0]
#define RA_EXT_SENS_DATA_10 83 // R    EXT_SENS_DATA_10[7:0]
#define RA_EXT_SENS_DATA_11 84 // R    EXT_SENS_DATA_11[7:0]
#define RA_EXT_SENS_DATA_12 85 // R    EXT_SENS_DATA_12[7:0]
#define RA_EXT_SENS_DATA_13 86 // R    EXT_SENS_DATA_13[7:0]
#define RA_EXT_SENS_DATA_14 87 // R    EXT_SENS_DATA_14[7:0]
#define RA_EXT_SENS_DATA_15 88 // R    EXT_SENS_DATA_15[7:0]
#define RA_EXT_SENS_DATA_16 89 // R    XT_SENS_DATA_16[7:0]
#define RA_EXT_SENS_DATA_17 90 // R    EXT_SENS_DATA_17[7:0]
#define RA_EXT_SENS_DATA_18 91 // R    EXT_SENS_DATA_18[7:0]
#define RA_EXT_SENS_DATA_19 92 // R    EXT_SENS_DATA_19[7:0]
#define RA_EXT_SENS_DATA_20 93 // R    EXT_SENS_DATA_20[7:0]
#define RA_EXT_SENS_DATA_21 94 // R    EXT_SENS_DATA_21[7:0]
#define RA_EXT_SENS_DATA_22 95 // R    EXT_SENS_DATA_22[7:0]
#define RA_EXT_SENS_DATA_23 96 // R    EXT_SENS_DATA_23[7:0]

#define RA_MOT_DETECT_STATUS 97 // R    MOT_XNEG[7]   MOT_XPOS[6]   MOT_YNEG[5]   MOT_YPOS[4]   MOT_ZNEG[3]   MOT_ZPOS[2]   MOT_ZRMOT[0]

#define RA_I2C_SLV0_DO 99  // R/W  I2C_SLV0_DO[7:0]
#define RA_I2C_SLV1_DO 100 // R/W  I2C_SLV1_DO[7:0]
#define RA_I2C_SLV2_DO 101 // R/W  I2C_SLV2_DO[7:0]
#define RA_I2C_SLV3_DO 102 // R/W  I2C_SLV3_DO[7:0]

#define RA_I2C_MST_DELAY_CTRL 103 // R/W  DELAY_ES _SHADOW    -    -    I2C_SLV4 _DLY_EN    I2C_SLV3 _DLY_EN    I2C_SLV2 _DLY_EN    I2C_SLV1 _DLY_EN    I2C_SLV0 _DLY_EN
#define RA_SIGNAL_PATH_RESET 104  // R/W    -    -    -    -    -    GYRO _RESET    ACCEL _RESET    TEMP _RESET
#define RA_MOT_DETECT_CTRL 105    // R/W       ACCEL_ON_DELAY[5:4]    FF_COUNT[3:2]
#define RA_USER_CTRL 106          // R/W    -    FIFO_EN    I2C_MST_EN    I2C_IF_DIS    -    FIFO_RESET    I2C_MST_RESET    SIG_COND_RESET
#define RA_PWR_MGMT_1 107         // R/W  DEVICE _RESET    SLEEP    CYCLE    -    TEMP_DIS    CLKSEL[2:0]
#define RA_PWR_MGMT_2 108         // R/W  LP_WAKE_CTRL[1:0]    STBY_XA    STBY_YA    STBY_ZA    STBY_XG    STBY_YG    STBY_ZG

#define RA_BANK_SEL 109       // R/W   PRFTCH_EN[6]    CFG_USER_BANK[5]    MEM_SEL[4:0]
#define RA_MEM_START_ADDR 110 // R/W   START_ADDR[7:0]
#define RA_MEM_R_W 111        // R/W   MEM_R_W[7:0]
#define RA_DMP_CFG_1 112      // R/W
#define RA_DMP_CFG_2 113      // R/W

#define RA_FIFO_COUNTH 114 // R/W  FIFO_COUNT[15:8]
#define RA_FIFO_COUNTL 115 // R/W  FIFO_COUNT[7:0]
#define RA_FIFO_R_W 116    // R/W  FIFO_DATA[7:0]
#define RA_WHO_AM_I 117    // R    -    WHO_AM_I[6:1]    -
