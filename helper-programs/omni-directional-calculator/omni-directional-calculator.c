#include <stdio.h>
#include <math.h>

void omni_calculator(int angle);

int main()
{
    int angle;
    printf("Enter angle: ");
    scanf("%d", &angle);
    omni_calculator(angle);
    return 0;
}

void omni_calculator(int angle)
{
    angle += 45;
    if (angle >= 360)
    {
        angle -= 360;
    }

    // double f1 = cos(angle * M_PI / 180);
    // double f2 = sin(angle * M_PI / 180);
    double f1 = sinf(angle * 3.14 / 180) * 100;
    double f2 = cosf(angle * 3.14 / 180) * 100;

    printf("F1: %.2lf %%\tF2: %.2lf %%\n", f1, f2);
}
