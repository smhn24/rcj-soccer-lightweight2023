#include "attacker_strategy.h"

extern Robot robot;
extern GOAL goal;

void attacker_strategy()
{
    if (robot.captured_ball)
    {
        update_head_angle();
    }
    else
    {
        robot.head_angle = 0;
    }
}
