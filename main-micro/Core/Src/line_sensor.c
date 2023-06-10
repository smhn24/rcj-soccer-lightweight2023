#include "line_sensor.h"

extern Robot robot;

extern bool line_sensors[20];
volatile uint8_t out_data[4] = {0};

const int16_t njl_angle[20] = {0, 18, 36, 54, 72, 90, 108, 126, 144, 162, 180, 198, 216, 234, 252, 270, 288, 306, 324, 342};
const float njl_x[20] = {1, 0.95, 0.81, 0.58, 0.31, 0, -0.31, -0.58, -0.81, -0.95, -1, -0.95, -0.81, -0.58, -0.31, 0, 0.31, 0.58, 0.81, 0.95};
const float njl_y[20] = {0, 0.31, 0.58, 0.81, 0.95, 1, 0.95, 0.81, 0.58, 0.31, 0, -0.31, -0.58, -0.81, -0.95, -1, -0.95, -0.81, -0.58, -0.31};

float angle_x[20] = {0}, angle_y[20] = {0};

uint8_t time_outed_sensor[20] = {255};
uint8_t time_outed_sensor_index = 0;

float average_x = 0;
float average_y = 0;

bool status[20] = {0};

void read_line_sensors(bool *line_sensors)
{
    HAL_I2C_Master_Receive(&hi2c2, LINE_SENSORS_ADDRESS, out_data, 3, 100);

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

    // TODO: It must change
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

void update_out_data()
{
    read_line_sensors(line_sensors); //? Update line_sensors array
    robot.on_line_sensors = on_line_sensors_number(line_sensors);

    bool special_status = false, end_sensor_neighbor = false;
    uint8_t neighbor_sensor_number = 0, neighborhood_counter = 0;
    int16_t average_neighbor_angle[20] = {0};
    // uint8_t start = 0, end = 20, temp = 0;
    uint8_t start = 0, end = 21, temp = 0;

    //? Initializing average_x & average_y

    if (robot.on_line_sensors > 0)
    {
        average_x = 0;
        average_y = 0;
        for (uint8_t i = 0; i < 20; i++)
        {
            angle_x[i] = 0;
            angle_y[i] = 0;
        }
        //? Finding the first sensor that went out
        for (uint8_t i = 0; i < 20; i++)
        {
            if (line_sensors[i])
            {
                if (!status[i])
                {
                    time_outed_sensor[time_outed_sensor_index] = i;
                    time_outed_sensor_index++;
                }
                status[i] = true;
            }
        }
        // TODO: Understanding special status
        if (line_sensors[0] && line_sensors[19])
        {
            special_status = true;
        }

        while (special_status)
        {
            if (line_sensors[temp])
            {
                angle_x[neighborhood_counter] += njl_x[temp];
                angle_y[neighborhood_counter] += njl_y[temp];
                neighbor_sensor_number++;
                temp++;
                start++;

                if (temp > 19)
                {
                    break;
                }
            }
            else
            {
                break;
            }
        }
        temp = 19;
        while (special_status)
        {
            if (line_sensors[temp])
            {
                angle_x[neighborhood_counter] += njl_x[temp];
                angle_y[neighborhood_counter] += njl_y[temp];
                neighbor_sensor_number++;
                temp--;
                end--;

                if (temp < 1)
                {
                    break;
                }
            }
            else
            {
                break;
            }
        }

        if (special_status)
        {
            // end = 20;
            angle_x[neighborhood_counter] /= neighbor_sensor_number;
            angle_y[neighborhood_counter] /= neighbor_sensor_number;
            neighborhood_counter++;
            neighbor_sensor_number = 0;
        }

        bool brk = 0;
        // bool brk = false;

        for (uint8_t i = start; i < end; i++)
        {
            if (i == 20)
            {
                i = 0;
                brk = 1;
            }
            //? For a sensor that sees we add line sensors to angle x and we count that sensor as a neighbor
            if (line_sensors[i])
            {
                angle_x[neighborhood_counter] += njl_x[i];
                angle_y[neighborhood_counter] += njl_y[i];
                neighbor_sensor_number++;
            }
            //? For the first sensor thast doesn't see after that neighborhood, we get average of the last neighborhood and stop counting neighbors until the next neighborhood
            else if (neighbor_sensor_number > 0)
            {
                angle_x[neighborhood_counter] /= neighbor_sensor_number;
                angle_y[neighborhood_counter] /= neighbor_sensor_number;

                neighborhood_counter++;
                neighbor_sensor_number = 0;
            }

            if (brk)
                break;
        }
        //? We add the neighborhoods to average x and y
        for (uint8_t i = 0; i < neighborhood_counter; i++)
        {
            average_x += angle_x[i];
            average_y += angle_y[i];
        }
        //? If the condition is symmetric
        if (average_x == 0)
        {
            if (average_y == 0)
            {
                robot.current_out_angle = njl_angle[time_outed_sensor[0]];
            }
            else if (average_y < 0)
            {
                robot.current_out_angle = -90;
            }
            else
            {
                robot.current_out_angle = 90;
            }
        }
        else
        {
            robot.current_out_angle = (int)(atan2f(average_y, average_x) * RADIAN_TO_DEGREE);
        }

        if (robot.current_out_angle < 0)
        {
            robot.current_out_angle += 360;
        }

        robot.invert_out_angle = robot.current_out_angle - 180;
        if (robot.invert_out_angle < 0)
        {
            robot.invert_out_angle += 360;
        }
        else if (robot.invert_out_angle > 359)
        {
            robot.invert_out_angle -= 360;
        }

        int16_t min = 360, min_index = 19;
        //? find the closest sensor to out angle
        for (uint8_t i = 0; i < 20; i++)
        {
            if (abs(njl_angle[i] - robot.current_out_angle) < min)
            {
                min = abs(njl_angle[i] - robot.current_out_angle);
                min_index = i;
            }
        }
        if (status[min_index])
        {
            uint8_t index = min_index + 10;
            if (index > 19)
            {
                index -= 20;
            }
            //? Check if the out angle should be inverted
            if (status[index])
            {
                for (uint8_t i = 0; i < 20; i++)
                {
                    if (time_outed_sensor[i] == min_index)
                    {
                        robot.out_angle = robot.current_out_angle;
                        break;
                    }
                    else if (time_outed_sensor[i] == index)
                    {
                        robot.out_angle = robot.invert_out_angle;
                        break;
                    }
                }
            }
            else
            {
                robot.out_angle = robot.current_out_angle;
            }
        }
        else
        {
            robot.out_angle = robot.invert_out_angle;
        }

        //? Convert grading system
        robot.out_angle -= 90;
        if (robot.out_angle < -180)
        {
            robot.out_angle += 360;
        }
        else if (robot.out_angle > 180)
        {
            robot.out_angle -= 360;
        }
        //? Apply out conditions to robot movement
        if (!robot.line_detect)
        {
            for (uint8_t i = 0; i < 20; i++)
            {
                if (line_sensors[i] && robot.green_time > 500)
                {
                    robot.first_out_sensor = i;
                    break;
                }
            }
            robot.line_detect = true;
            robot.in_out_area = true;
        }

        if (robot.role == attacker)
        {
            robot.move_angle = robot.out_angle - 180;
            while (robot.move_angle < 0)
            {
                robot.move_angle += 360;
            }
            robot.percent_speed = ENTERING_PERCENT_SPEED;
        }
    }
    else
    {
        robot.line_detect = false;
        if (robot.role == attacker)
        {
            robot.percent_speed = 0;
        }

        //? All of the sensors don't see after getting in the field again
        if (robot.green_time > 200)
        {
            for (uint8_t i = 0; i < 20; i++)
            {
                time_outed_sensor[i] = 255;
                time_outed_sensor_index = 0;
                status[i] = false;
            }
        }
    }
}