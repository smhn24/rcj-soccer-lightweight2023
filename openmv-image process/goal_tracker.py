import sensor, image, time

from machine import UART

class GoalRect():
    def __init__(self, x, y, width, height, color):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.color = color


thresholds = [
    #(36, 70, -128, 31, 40, 70), # Yellow -> code = 1
    #(62, 95, -13, 20, 23, 56),
    #(40, 75, -11, 20, 15, 50),
    #(40, 100, -23, 11, 19, 81),
    (70, 100, -15, 25, 35, 72),
    #(74, 95, -20, 20, 15, 45),
    #(12, 17, -3, 15, -35, -16)  # Blue -> code = 2
    #(15, 35, 10, 45, -85, -30)
    #(31, 50, -10, 35, -75, -24)
    #(60, 80, -24, 4, -55, -25)
    #(25, 80, -40, 40, -85, -28)
    #(65, 87, -23, 1, -45, -22)
    #(45, 87, -23, 1, -45, -22)
    #(64, 80, -20, 10, -47, -20)
    (40, 80, -15, 15, -56, -20)
]

blobs = []
goal_rects = []

uart = UART(3, 115200)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
#sensor.set_brightness(-2) #!!!
#sensor.set_saturation(3)
#sensor.set_contrast(-3)
#sensor.set_contrast(3) #!!!
#sensor.set_auto_whitebal(False)
#sensor.set_auto_gain(False)          # must be turned off for color tracking
#sensor.set_auto_gain(False, gain_db=19)          # must be turned off for color tracking  #!!!
#sensor.set_auto_exposure(False, exposure_us=2500)
#sensor.set_auto_whitebal(False)      # must be turned off for color tracking
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
            #img.draw_string(blob.x() + 2, blob.y() + 2, "yellow")
            goal_rects.append(GoalRect(x=blob.cx(), y=blob.cy(), width=blob.w(), height=blob.h(), color='Yellow'))

        elif blob.code() == 2:
            #img.draw_string(blob.x() + 2, blob.y() + 2, "blue")
            goal_rects.append(GoalRect(x=blob.cx(), y=blob.cy(), width=blob.w(), height=blob.h(), color='Blue'))

    if len(goal_rects) > 0:
        goal = goal_rects[0]
        for rect in goal_rects:
            if rect.width > goal.width:
                goal = rect
    else:
        goal = GoalRect(x=0, y=0, width=0, height=0, color='')

    #print(f'{rect.width}')
    #print(f'Length: {120 - rect.y}   Width: {rect.x - 80}')
    #print(rect.x)

    #goal_length = 120 - goal.y

    if goal.width != 0:
        goal_length = 160-goal.width
        goal_width = goal.x-80

        if goal_width < 0:
            #goal_width -= int(goal.width/4)
            goal_width -= int(goal.width/3.5)
        elif goal_width > 0:
            #goal_width += int(goal_width/4)
            goal_width += int(goal_width/3.5)

    else:
        goal_length = 0
        goal_width = 0

    #print(f'Length: {goal_length}   Width: {goal_width}')

    uart.write(f'/{goal_width:04},{goal_length:03}')
    #print(f'/{goal_width:04},{goal_length:03}')


        #print(f"coord: ({blob.cx()}, {blob.cy()})")
        #print(f"Width: {blob.w()}   Height: {blob.h()}")

    #print(clock.fps())

    goal_rects.clear()
