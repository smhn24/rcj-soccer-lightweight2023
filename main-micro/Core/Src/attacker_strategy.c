#include "attacker_strategy.h"
#include "helpers.h"

extern Robot robot;

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
