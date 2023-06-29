#include "robot_connection.h"

extern SECOND_ROBOT second_robot;
extern uint8_t second_robot_pocket[POCKET_LENGTH];

uint8_t second_robot_sorted_pocket[POCKET_LENGTH] = {0};

uint8_t second_robot_task_char;

void read_second_robot_pocket()
{
    uint8_t first_index = POCKET_LENGTH + 1;
    for (uint8_t i = 0; i < POCKET_LENGTH; i++)
    {
        if (second_robot_pocket[i] == 'A' || second_robot_pocket[i] == 'G')
        {
            first_index = i;
            if (second_robot_pocket[i] == 'A')
            {
                second_robot.role = attacker;
            }
            else if (second_robot_pocket[i] == 'G')
            {
                second_robot.role = goal_keeper;
            }

            break;
        }
    }

    if (first_index != (POCKET_LENGTH + 1))
    {
        for (uint8_t i = first_index, j = 0; i < POCKET_LENGTH; i++, j++)
        {
            second_robot_sorted_pocket[j] = second_robot_pocket[i];
        }

        for (uint8_t i = 0, j = POCKET_LENGTH - first_index; i < first_index; i++, j++)
        {
            second_robot_sorted_pocket[j] = second_robot_pocket[i];
        }
    }
    else
    {
        for (uint8_t i = 0; i < POCKET_LENGTH; i++)
        {
            second_robot_sorted_pocket[i] = 0;
        }
    }

    sscanf(second_robot_sorted_pocket, "%c,%d", &second_robot_task_char, &second_robot.ball_distance);

    if (second_robot_task_char == 'A')
    {
        second_robot.role = attacker;
    }
    else if (second_robot_task_char == 'G')
    {
        second_robot.role = goal_keeper;
    }
}
