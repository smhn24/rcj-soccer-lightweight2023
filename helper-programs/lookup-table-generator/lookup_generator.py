import math

lookup = []

MAX_VELOCITY = 100.0

for i in range(0, 360, 5):
    angle = i + 45
    while angle > 360:
        angle -= 360

    lookup.append([
        int(round(math.sin(angle * 0.01744) * MAX_VELOCITY/5.0))*5,
        int(round(math.cos(angle * 0.01744) * MAX_VELOCITY/5.0))*5,
    ])

    # lookup.append(int(round(math.sin(angle * 0.01744) * 2800.0/5.0))*5)

# print(lookup)
for i in range(72):
    print(f'{lookup[i]},')
