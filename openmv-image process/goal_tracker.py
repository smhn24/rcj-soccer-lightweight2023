import sensor, image, time

class GoalRect():
    def __init__(self, x, y, width, height, color):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.color = color


thresholds = [
    (36, 70, -128, 31, 40, 70), # Yellow -> code = 1
    (12, 17, -3, 15, -35, -16)  # Blue -> code = 2
]

blobs = []
goal_rects = []

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)          # must be turned off for color tracking
sensor.set_auto_whitebal(False)      # must be turned off for color tracking
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


    for rect in goal_rects:
        print(f'{rect.width}')
        #print(f'Length: {120 - rect.y}   Width: {rect.x - 80}')
        #print(rect.x)


    if len(blobs) == 0:
        #print("No goal detected!!!")
        pass


        #print(f"coord: ({blob.cx()}, {blob.cy()})")
        #print(f"Width: {blob.w()}   Height: {blob.h()}")

    #print(clock.fps())

    goal_rects.clear()



#def goal_merger(rects):
    #if len(rects) == 1:
        #return rects[0]

    #if rects[0].x > rects[1].x:
        #left = rects[0]
        #right = rects[1]
    #else:
        #left = rects[1]
        #right = rects[0]

    #cx = (left.x + (left.w / 2) + right.x - (right.width / 2)) / 2
    #cy = left.y

    #w = left.x - rigth.x
    #h = left.height

    #return GoalRect(cx, cy, w, h, left.color)




