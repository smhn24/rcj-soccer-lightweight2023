#include "srf_helper.h"

extern volatile SRF left_srf, right_srf, back_srf;

extern float pidout_line;
extern int X_srf;
extern float Y_srf;
extern uint8_t TrueSRF;

extern uint16_t Y_timer;

extern uint16_t go_ahead_flag;

void update_srf_data()
{
    right_srf.dis = right_srf.width / 55.7;
    left_srf.dis = left_srf.width / 55.7;
    back_srf.dis = back_srf.width / 55.7;

    if (back_srf.dis < 200)
    {
        Y_srf = (Y_srf * 0.95) + (back_srf.dis * 0.05);
    }

    int SumRL = right_srf.dis + left_srf.dis;
    if (SumRL > 155 && SumRL < 205)
    {
        X_srf = left_srf.dis;
        TrueSRF = 0;
    }
    else if (SumRL < 160)
    {
        if (TrueSRF == 0)
        {
            if (abs(right_srf.pre_dis - right_srf.dis) < abs(left_srf.pre_dis - left_srf.dis))
            {
                TrueSRF = 1;
            }
            else
            {
                TrueSRF = 2;
            }
        }

        if (TrueSRF == 1)
        {
            // TODO: define 182
            X_srf = 182 - right_srf.dis;
        }
        else if (TrueSRF == 2)
        {
            X_srf = left_srf.dis;
        }
    }
    else
    {
        TrueSRF = 3;
    }

    right_srf.pre_dis = right_srf.dis;
    left_srf.pre_dis = left_srf.dis;
}
