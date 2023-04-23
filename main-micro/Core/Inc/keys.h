#ifndef KEYS_H
#define KEYS_H

#define START_KEY_STATUS() !(LL_GPIO_ReadInputPort(StartKey_GPIO_Port) & StartKey_Pin)

#define KEY_STATUS() !(LL_GPIO_ReadInputPort(Key_GPIO_Port) & Key_Pin)

// #define ROLE_STATUS() !(LL_GPIO_ReadInputPort(ROBOT_ROLE_GPIO_Port) & ROBOT_ROLE_Pin)
#define ROLE_STATUS() (LL_GPIO_ReadInputPort(ROBOT_ROLE_GPIO_Port) & ROBOT_ROLE_Pin)

#define BALL_SENSOR_1_STATUS() (LL_GPIO_ReadInputPort(BALL_SENSOR_1_GPIO_Port) & BALL_SENSOR_1_Pin)
#define BALL_SENSOR_2_STATUS() (LL_GPIO_ReadInputPort(BALL_SENSOR_2_GPIO_Port) & BALL_SENSOR_2_Pin)

#define CAPTURED_BALL_STATUS() (BALL_SENSOR_1_STATUS() && BALL_SENSOR_2_STATUS())

#endif
