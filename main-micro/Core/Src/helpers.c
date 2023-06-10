#include "helpers.h"
#include "camera.h"
#include "robot_movement.h"

extern Robot robot;
extern uint8_t openmv_data[OPENMV_DATA_LENGTH];

uint8_t read_bno055_step = 0;
uint8_t BNO_lsb, BNO_msb;

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

    HAL_TIM_Base_Start_IT(&htim4); //? Timer for srf captures
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);

    HAL_TIM_Base_Start_IT(&htim7);  //? Timer for timing of tasks, brake,... with 1ms overflow and 1us counter
    HAL_TIM_Base_Start_IT(&htim13); //? Start timer for checkout tssp pulses
}

void uart_error_handler()
{
    if (UART5->SR & 0x0000000A) //? Error check
    {
        UART5->DR;
        UART5->SR &= ~(0x000003E0);
        if (robot.role == attacker)
        {
            HAL_UART_Receive_DMA(&huart5, openmv_data, OPENMV_DATA_LENGTH);
        }
    }
}

void update_robot_angle()
{
    static bool ignore = false;
    int angle;

    if (read_bno055_step == 0)
    {
        HAL_GPIO_WritePin(SPI1_BNO_SS_GPIO_Port, SPI1_BNO_SS_Pin, 0);
        __NOP();
        HAL_SPI_Transmit(&hspi1, "L", 1, 100);
    }

    if (read_bno055_step == 1)
    {
        HAL_SPI_Receive(&hspi1, &BNO_lsb, 1, 100);
        __NOP();
        HAL_SPI_Transmit(&hspi1, "H", 1, 100);
    }

    if (read_bno055_step == 2)
    {
        HAL_SPI_Receive(&hspi1, &BNO_msb, 1, 100);
        __NOP();
        HAL_GPIO_WritePin(SPI1_BNO_SS_GPIO_Port, SPI1_BNO_SS_Pin, 1);

        angle = (BNO_msb << 8) | BNO_lsb;
        if (angle >= 360)
        {
            angle = 0;
        }

        angle *= -1;
        if (angle < -180)
        {
            angle += 360;
        }

        if (abs(angle - robot.angle) < 15 || ignore)
        {
            robot.angle = angle;
            ignore = false;
        }
        else
        {
            ignore = true;
        }
    }

    read_bno055_step++;
    if (read_bno055_step > 2)
    {
        read_bno055_step = 0;
    }
}
