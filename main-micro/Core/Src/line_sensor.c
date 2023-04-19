#include "line_sensor.h"

#include "usart.h"

extern Robot robot;

uint8_t a1[20], a2[20];
uint8_t c1 = 0;

void read_line_sensors(bool *line_sensors)
{
    uint8_t out_data[3];
    HAL_I2C_Master_Receive(&hi2c2, LINE_SENSORS_ADDRESS, (uint8_t *)out_data, 3, 100);
    for (uint8_t i = 0; i < 8; i++)
    {
        line_sensors[i] = (out_data[2] & 0x01) == 1;
        out_data[2] >>= 1;
    }
    for (uint8_t i = 8; i < 16; i++)
    {
        line_sensors[i] = (out_data[1] & 0x01) == 1;
        out_data[1] >>= 1;
    }
    for (uint8_t i = 16; i < 20; i++)
    {
        line_sensors[i] = (out_data[0] & 0x01) == 1;
        out_data[0] >>= 1;
    }

    bool temp = line_sensors[0];
    for (uint8_t i = 0; i < 19; i++)
    {
        line_sensors[i] = line_sensors[i + 1];
    }
    line_sensors[19] = temp;
}

uint8_t on_line_sensors_number(bool *line_sensors)
{
    uint8_t on_line_sensors = 0;
    for (uint8_t i = 0; i < 20; i++)
    {
        if (line_sensors[i])
        {
            on_line_sensors++;
        }
    }
    return on_line_sensors;
}

void get_edges(bool *line_sensors, direction_t out_direction, uint8_t *out_edges)
{
    for (int8_t i = 0; i < 20; i++)
    {
        a1[i] = 0;
        a2[i] = 0;
        c1 = 0;
        for (int8_t j = i + 1; j != i; j++)
        {
            if (j == 20)
            {
                j = 0;
            }

            if (!line_sensors[j])
            {
                c1++;
                a1[i] = c1;
            }
            else
            {
                break;
            }
        }
        c1 = 0;
        for (int8_t j = i - 1; j != i;)
        {
            if (j < 0)
            {
                j += 20;
            }

            if (!line_sensors[j])
            {
                c1++;
                a2[i] = c1;
            }
            else
            {
                break;
            }
            j--;
            if (j < 0)
            {
                j += 20;
            }
        }
    }
    uint8_t max_val = 0, max_index = 0, last = 0;
    for (int8_t i = 0; i < 20; i++)
    {
        if (a1[i] > max_val)
        {
            max_val = a1[i];
            max_index = i;
        }
    }
    last = max_val + max_index + 1;
    if (last > 19)
        last -= 20;
    robot.out_edges[0] = max_index;
    robot.out_edges[1] = last;
    robot.out_error = max_val;
    robot.out_error -= 8;
    if (robot.out_error < 0)
    {
        robot.out_error = 0;
    }
    // uint8_t tx[100];
    // sprintf(tx, "err: %u   i: %u   L: %u   FOS: %d\r\n", robot.out_error, robot.out_edges[0], robot.out_edges[1], robot.first_out_sensor);
    // HAL_UART_Transmit(&huart4, tx, strlen(tx), 200);
}

void get_out_direction(bool *line_sensors)
{
    int8_t start = 0;
    int8_t end = 0;

    int s1, s2, s3, s4;
    int e1, e2, e3, e4;
    bool measared = false;
    for (uint8_t i = 0; i < 20; i++)
    {
        if (line_sensors[i])
        {
            robot.first_out_sensor = i;
            update_range(&start, &end, 8, 4);
            // end = start + 4;
            s1 = start;
            e1 = end;
            if (i >= start && i <= end && !measared) //? Left
            {
                robot.out_direction = W;
                robot.out_angle = 270;
                measared = true;
                // return;
            }
            update_range(&start, &end, 13, 4);
            // end = start + 4;
            s2 = start;
            e2 = end;
            if (i >= start && i <= end && !measared) //? Back
            {
                robot.out_direction = S;
                robot.out_angle = 180;
                // return;
            }
            update_range(&start, &end, 3, 4);
            // end = start + 4;
            s4 = start;
            e4 = end;
            if (i >= start && i <= end && !measared) //? Front
            {
                robot.out_direction = N;
                robot.out_angle = 0;
                // return;
            }

            update_range(&start, &end, 18, 4);
            // end = start + 4;
            while (end > 19)
            {
                end -= 20;
            }
            s3 = start;
            e3 = end;

            if (start > 10)
            {
                if (i >= start || i <= end && !measared) //? Right
                {
                    robot.out_direction = E;
                    robot.out_angle = 90;
                    // return;
                }
            }
            else
            {
                if (i >= start && i <= end && !measared) //? Right
                {
                    robot.out_direction = E;
                    robot.out_angle = 90;
                    // return;
                }
            }
            // uint8_t tx[100];
            // sprintf(tx, "L[%u,%u]B[%u,%u]R:[%u,%u]F[%u,%u]\r\n", s1, e1, s2, e2, s3, e3, s4, e4);
            // HAL_UART_Transmit(&huart4, tx, strlen(tx), 100);

            break;
        }
    }
}

void get_out_direction_edge(bool *line_sensors, uint8_t first_sensor)
{
    switch (robot.out_direction)
    {
    case N:
        // if (is_online_quad_3(line_sensors) && is_quad_4_empty(line_sensors))
        if (online_quad(3, line_sensors) > MIN_NJL_SENSORS && online_quad(4, line_sensors) == 0)
        {

            robot.out_direction = NW;
            robot.out_angle = 315;
        }
        // else if (is_online_quad_4(line_sensors) && is_quad_3_empty(line_sensors))
        else if (online_quad(4, line_sensors) > MIN_NJL_SENSORS && online_quad(3, line_sensors) == 0)
        {

            robot.out_direction = NE;
            robot.out_angle = 45;
        }
        break;
    case E:
        // if (is_online_quad_3(line_sensors))
        if (online_quad(3, line_sensors) > MIN_NJL_SENSORS)
        {

            robot.out_direction = SE;
            robot.out_angle = 135;
        }
        // else if (is_online_quad_2(line_sensors))
        else if (online_quad(2, line_sensors) > MIN_NJL_SENSORS)
        {
            robot.out_direction = NE;
            robot.out_angle = 45;
        }
        break;
    case S:
        // if (is_online_quad_2(line_sensors))
        if (online_quad(2, line_sensors) > MIN_NJL_SENSORS)
        {
            robot.out_direction = SW;
            robot.out_angle = 225;
        }
        // else if (is_online_quad_1(line_sensors))
        else if (online_quad(1, line_sensors) > MIN_NJL_SENSORS)
        {
            robot.out_direction = SE;
            robot.out_angle = 135;
        }
        break;
    case W:
        // if (is_online_quad_1(line_sensors))
        if (online_quad(1, line_sensors) > MIN_NJL_SENSORS)
        {
            robot.out_direction = NW;
            robot.out_angle = 315;
        }
        // else if (is_online_quad_4(line_sensors))
        else if (online_quad(4, line_sensors) > MIN_NJL_SENSORS)
        {
            robot.out_direction = SW;
            robot.out_angle = 225;
        }
        break;
    default:
        break;
    }
}

// void get_out_angle()
// {
//     robot.out_angle = (robot.out_edges[0] + robot.out_edges[1]) / 2;
//     robot.out_angle *= 18;

//     if (robot.out_direction == E)
//     {
//         robot.out_angle = abs(robot.out_angle - 180);
//     }

//     //? Convert grading system
//     robot.out_angle -= 90;
//     if (robot.out_angle < 0)
//         robot.out_angle += 360;
//     robot.out_angle *= -1;
//     robot.out_angle += 360;
//     // robot.out_angle += 13;
// }

void get_out_error()
{
    int offset = njl_offset(robot.angle);
    int8_t left, right, front, back;
    left = 10 + offset;
    right = 0 + offset;
    front = 5 + offset;
    back = 15 + offset;

    while (left > 19)
    {
        left -= 20;
    }
    while (front > 19)
    {
        front -= 20;
    }
    while (back > 19)
    {
        back -= 20;
    }

    if (robot.out_direction == E)
    {
        if (right < 10)
        {
            robot.out_error_x = (robot.out_edges[0] > robot.out_edges[1]) ? abs(right - robot.out_edges[1]) : abs(right - robot.out_edges[0]);
        }
        else
        {
            robot.out_error_x = (robot.out_edges[0] > robot.out_edges[1]) ? abs(right - robot.out_edges[0]) : abs(right - robot.out_edges[1]);
        }
    }
    else if (robot.out_direction == W)
    {
        robot.out_error_x = (robot.out_edges[0] > robot.out_edges[1]) ? abs(left - robot.out_edges[1]) : abs(left - robot.out_edges[0]);
    }
    else if (robot.out_direction == N)
    {
        robot.out_error_y = (robot.out_edges[0] > robot.out_edges[1]) ? abs(front - robot.out_edges[1]) : abs(front - robot.out_edges[0]);
    }
    else if (robot.out_direction == S)
    {
        robot.out_error_y = (robot.out_edges[0] > robot.out_edges[1]) ? abs(back - robot.out_edges[0]) : abs(back - robot.out_edges[1]);
    }
    else if (robot.out_direction == NW)
    {
        // robot.out_error = (robot.out_edges[0] > 10) ? (abs(left - robot.out_edges[0]) + abs(front - robot.out_edges[1])) : (abs(left - robot.out_edges[1]) + abs(front - robot.out_edges[0]));
        if (robot.out_edges[0] > 10)
        {
            robot.out_error_x = abs(left - robot.out_edges[0]);
            robot.out_error_y = abs(front - robot.out_edges[1]);
        }
        else
        {
            robot.out_error_x = abs(left - robot.out_edges[1]);
            robot.out_error_y = abs(front - robot.out_edges[0]);
        }
    }
    else if (robot.out_direction == NE)
    {
        if (robot.out_edges[0] > robot.out_edges[1])
        {
            robot.out_error_x = abs(right - robot.out_edges[0]);
            robot.out_error_y = abs(front - robot.out_edges[1]);
        }
        else
        {
            robot.out_error_x = abs(right - robot.out_edges[1]);
            robot.out_error_y = abs(front - robot.out_edges[0]);
        }
        if (robot.out_error_x > 10)
        {
            robot.out_error_x = abs(20 - robot.out_error_x);
        }
    }
    else if (robot.out_direction == SE)
    {
        if (robot.out_edges[0] > robot.out_edges[1])
        {
            robot.out_error_x = abs(right - robot.out_edges[1]);
            robot.out_error_y = abs(back - robot.out_edges[0]);
        }
        else
        {
            robot.out_error_x = abs(right - robot.out_edges[0]);
            robot.out_error_y = abs(back - robot.out_edges[1]);
        }
        if (robot.out_error_x > 10)
        {
            robot.out_error_x = abs(20 - robot.out_error_x);
        }
        if (robot.out_error_y > 10)
        {
            robot.out_error_y = abs(20 - robot.out_error_y);
        }
    }
    else if (robot.out_direction == SW)
    {
        if (robot.out_edges[0] > robot.out_edges[1])
        {
            robot.out_error_x = abs(left - robot.out_edges[1]);
            robot.out_error_y = abs(back - robot.out_edges[0]);
        }
        else
        {
            robot.out_error_x = abs(left - robot.out_edges[0]);
            robot.out_error_y = abs(back - robot.out_edges[1]);
        }
    }

    // uint8_t tx[100];
    // sprintf(tx, "D: %d  B: %d  E0: %d  E1: %d  ErrX: %d   ErrY: %d  FOS: %d\r\n", robot.out_direction, back, robot.out_edges[0], robot.out_edges[1], robot.out_error_x, robot.out_error_y, robot.first_out_sensor);
    // HAL_UART_Transmit(&huart4, tx, strlen(tx), 100);
}

uint8_t online_quad(uint8_t quad, bool *line_sensors)
{
    int offset = njl_offset(robot.angle);
    quad -= 1;
    int start = offset + (quad * 5);
    int end = start + 6;
    while (start > 19)
    {
        start -= 20;
    }
    while (end > 19)
    {
        end -= 20;
    }
    uint8_t counter = 0;
    for (uint8_t i = start; i != end;)
    {
        if (line_sensors[i])
        {
            counter++;
        }

        i++;
        if (i > 19)
        {
            i = 0;
        }
    }
    return counter;
}

int njl_offset(int angle)
{
    int offset = angle / 19;
    if (angle % 19 > 9)
        offset++;

    return offset;
}

void update_range(int8_t *start, int8_t *end, int8_t s, int8_t e)
{
    int offset = njl_offset(robot.angle);
    *start = offset + s;
    *end = *start + e;
    while (*start > 19)
    {
        *start -= 20;
    }
    while (*end > 19)
    {
        *end -= 20;
    }
}
