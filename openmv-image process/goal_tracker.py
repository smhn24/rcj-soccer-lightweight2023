import sensor, image, time
import math

from machine import UART

class GoalRect():
    def __init__(self, x, y, width, height, color):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.color = color


thresholds = [
    (35, 65, -16, 20, 15, 40), # Yellow -> code = 1
    (20, 40, -9, 11, -14, 5) # Blue -> code = 2
]

blobs = []
goal_rects = []

uart = UART(3, 115200)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
sensor.set_brightness(1) #!!!
#sensor.set_saturation(1)
#sensor.set_contrast(3)
sensor.set_auto_gain(False, gain_db=22)     # must be turned off for color tracking
sensor.set_auto_whitebal(False)      # must be turned off for color tracking
#sensor.set_gainceiling(128) #!!!
clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot()
    img = img.rotation_corr(z_rotation=180)

    blobs = img.find_blobs(thresholds, pixels_threshold=75, area_threshold=75, merge=True)

    for blob in blobs:
        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        if blob.code() == 1:
            img.draw_string(blob.x() + 2, blob.y() + 2, "yellow")
            goal_rects.append(GoalRect(x=blob.cx(), y=blob.cy(), width=blob.w(), height=blob.h(), color='Yellow'))

        elif blob.code() == 2:
            img.draw_string(blob.x() + 2, blob.y() + 2, "blue")
            goal_rects.append(GoalRect(x=blob.cx(), y=blob.cy(), width=blob.w(), height=blob.h(), color='Blue'))

    if len(goal_rects) > 0:
        goal = goal_rects[0]
        for rect in goal_rects:
            if rect.width > goal.width:
                goal = rect
    else:
        goal = GoalRect(x=0, y=0, width=0, height=0, color='')

    if goal.width != 0:
        goal_length = 160-goal.width
        goal_width = goal.x-80


        if goal_width < 0:
            goal_width -= int(goal.width/4)
        elif goal_width > 0:
            goal_width += int(goal_width/4)

    else:
        goal_length = 0
        goal_width = 0

    uart.write(f'/{goal_width:04},{goal_length:03}')

    #print(clock.fps())

    goal_rects.clear()
