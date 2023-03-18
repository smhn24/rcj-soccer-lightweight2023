#ifndef KEYS_H
#define KEYS_H

#define START_KEY_STATUS() !(LL_GPIO_ReadInputPort(StartKey_GPIO_Port) & StartKey_Pin)

#define KEY_STATUS() !(LL_GPIO_ReadInputPort(Key_GPIO_Port) & Key_Pin)

#endif