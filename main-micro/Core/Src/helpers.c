#include "helpers.h"

void start_timers()
{
    //* PWM timers
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //? Motor 2
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); //? Motor 3
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); //? Motor 4
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); //? Motor 1

    //* Input Capture timers for TSSP sensors
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); //? TSSP7
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2); //? TSSP8
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3); //? TSSP4
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4); //? TSSP11

    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1); //? TSSP6
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2); //? TSSP9
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3); //? TSSP3
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4); //? TSSP12

    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1); //? TSSP5
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2); //? TSSP10

    HAL_TIM_Base_Start_IT(&htim8);
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1); //? TSSP1
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2); //? TSSP14
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_3); //? TSSP15
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_4); //? TSSP0

    HAL_TIM_Base_Start_IT(&htim12);
    HAL_TIM_IC_Start_IT(&htim12, TIM_CHANNEL_1); //? TSSP2
    HAL_TIM_IC_Start_IT(&htim12, TIM_CHANNEL_2); //? TSSP13

    HAL_TIM_Base_Start_IT(&htim4); //? Start timer for checkout tssp pulses
    HAL_TIM_Base_Start_IT(&htim7); //? Timer for timing of tasks, brake,... with 1ms overflow and 1us counter
}
