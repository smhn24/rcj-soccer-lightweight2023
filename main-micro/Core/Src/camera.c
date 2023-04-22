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

// void read_pixy()
// {
//     static uint8_t cc_while = 0, pair_check = 0, data[15];
//     static bool new_data = 0, f_0x55 = 0, startf = 0, new_frame = 0;
//     int16_t checksum;

//     while (PIXY_RX_DMA_CNT != cc_while)
//     {
//         if (pixycam_data[cc_while] == 0x55)
//         {
//             f_0x55 = 1;
//         } // if(new_frame) {t=time; time=0;}
//         else if (pixycam_data[cc_while] == 0xaa && f_0x55 == 1)
//         {
//             pair_check = 0;
//             f_0x55 = 0;
//             startf = 1;
//         } //  new_frame=1;
//         else if (f_0x55 == 1)
//         {
//             f_0x55 = 0;
//             data[pair_check] = 0x55;
//             pair_check++;
//             data[pair_check] = pixycam_data[cc_while];
//             pair_check++;
//         }
//         else if (startf)
//         {
//             new_frame = 0;
//             data[pair_check] = pixycam_data[cc_while];
//             pair_check++;
//         }

//         if (pair_check >= 12)
//         {
//             new_data = 1;
//             startf = 0;
//             pair_check = 0;

//             int16_t checksum = ((int16_t)data[1] << 8) | data[0];
//             int16_t sign = ((int16_t)data[3] << 8) | data[2]; // Signiture Number
//             int16_t xs = ((int16_t)data[5] << 8) | data[4];   // X
//             int16_t ys = ((int16_t)data[7] << 8) | data[6];   // Y
//             int16_t ws = ((int16_t)data[9] << 8) | data[8];   // width
//             int16_t hs = ((int16_t)data[11] << 8) | data[10]; // height

//             if (checksum == (sign + xs + ys + ws + hs))
//             {
//                 uint8_t tx[100];
//                 sprintf(tx, "xs: %d  ys: %d   ws: %d   hs: %d\r\n", xs, ys, ws, hs);
//                 HAL_UART_Transmit(&huart4, tx, strlen(tx), 100);
//             }
//         }

//         cc_while++;
//         if (cc_while >= PIXYCAM_DATA_LENGTH)
//         {
//             cc_while = 0;
//         }
//     }
// }