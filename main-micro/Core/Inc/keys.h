#ifndef KEYS_H
#define KEYS_H

#define START_KEY_STATUS() !(LL_GPIO_ReadInputPort(StartKey_GPIO_Port) & StartKey_Pin)

#define KEY_STATUS() !(LL_GPIO_ReadInputPort(Key_GPIO_Port) & Key_Pin)

// #define ROLE_STATUS() !(LL_GPIO_ReadInputPort(ROBOT_ROLE_GPIO_Port) & ROBOT_ROLE_Pin)
#define ROLE_STATUS() (LL_GPIO_ReadInputPort(ROBOT_ROLE_GPIO_Port) & ROBOT_ROLE_Pin)

#endif