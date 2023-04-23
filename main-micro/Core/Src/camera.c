#include "camera.h"

#include "robot_movement.h"

extern GOAL goal;
extern uint8_t openmv_data[OPENMV_DATA_LENGTH];

uint8_t openmv_sorted_data[OPENMV_DATA_LENGTH] = {0};

void read_openmv()
{
    uint8_t first_index = 10;
    for (uint8_t i = 0; i < OPENMV_DATA_LENGTH; i++)
    {
        if (openmv_data[i] == OPENMV_START_CHAR)
        {
            first_index = i;
            break;
        }
    }
    if (first_index != 10)
    {
        for (uint8_t i = first_index, j = 0; i < OPENMV_DATA_LENGTH; i++, j++)
        {
            openmv_sorted_data[j] = openmv_data[i];
        }

        for (uint8_t i = 0, j = OPENMV_DATA_LENGTH - first_index; i < first_index; i++, j++)
        {
            openmv_sorted_data[j] = openmv_data[i];
        }
    }
    else
    {
        for (uint8_t i = 0; i < OPENMV_DATA_LENGTH; i++)
        {
            openmv_sorted_data[i] = 0;
        }
    }

    sscanf(openmv_sorted_data, "/%d,%d", &goal.width, &goal.height);

    if (goal.width == 0 && goal.height == 0)
    {
        goal.detection = false;
    }
    else
    {
        goal.detection = true;
    }

    // uint8_t tx[100];
    // sprintf(tx, "%s      %s    W: %d   H: %d   D: %d\r\n", openmv_data, openmv_sorted_data, goal.width, goal.height, goal.detection);
    // HAL_UART_Transmit(&huart4, tx, strlen(tx), 200);
}
