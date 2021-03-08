# Untitled - By: te160 - Fri Aug 14 2020

import sensor, image, time, math, pyb

def get_centroid(img):
    w, h = img.width(), img.height()
    wsum = 0
    hsum = 0
    num = 0
    for i in range(h):
        for j in range(w):
            val = img.get_pixel(j, i)
            if val == 1:
                wsum += j
                hsum += i
                num += 1
    return (int(wsum/num), int(hsum/num))

def find_best_blob(w, h, blobs):
    cx, cy = w//2, h//2
    best_dist = None
    best_b = None
    for b in blobs:
        bx, by = b.cx(), b.cy()
        dist_2 = (bx-cx)**2 + (by-cy)**2
        if best_dist == None or dist_2 < best_dist:
            best_dist = dist_2
            best_b = b
    return best_b

def get_R(c, rectangle):
    x0, y0 = c
    x1, y1 = rectangle[0], rectangle[1]
    x2, y2 = x1 + rectangle[2], y1 + rectangle[3]
    return min(x0-x1, y0-y1, x2-x0, y2-y0)

def sample_from_circle(c, r, num, w, h):
    points = []
    cx, cy = c
    for i in range(num):
        rnd = pyb.rng() / (2**30-1)
        angle = rnd * 2 * math.pi
        x = int(round(r * math.cos(angle)) + cx)
        y = int(round(r * math.sin(angle)) + cy)
        if x >= 0 and x < w and y >= 0 and y < h:
            points.append((int(x), int(y)))
    return points

def color_hit(color):
    r, g, b = color
    return r > 80 and g > 80 and b > 80

def percent_hit(img, points):
    num = 0
    for p in points:
        color = img.get_pixel(p)
        if color_hit(color):
            num += 1
    return num / len(points)


def find_palm(blob, img, w, h):
    cx, cy = blob.cx(), blob.cy()
    rectangle = blob.rect()
    Rmax = get_R((cx, cy), rectangle)
    print(Rmax)
    Rmin = 1
    num = 30
    th = 0.7
    d = 0.05
    times = 0
    while times < 10:
        guess = round((Rmax + Rmin)/2)
        points = sample_from_circle((cx, cy), guess, num, w, h)
        hit = percent_hit(img, points)
        delta = th - hit
        times += 1
        if delta > d:
            Rmax = guess
        elif -delta > d:
            Rmin = guess
        else:
            break
    print(times, guess, hit)
    return (cx, cy, guess)

thresholds = [(60, 100, -40, 127, 0, 127)] # hand threshold, assume dark hand and bright bg

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
img_w, img_h = 320, 240
img_masked = sensor.alloc_extra_fb(img_w, img_h, sensor.BINARY)
img_tmp = sensor.alloc_extra_fb(img_w, img_h, sensor.RGB565)
clock = time.clock()
# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. Don't set "merge=True" becuase that will merge blobs which we don't want here.
while(True):
    clock.tick()
    img = sensor.snapshot()
    img.gaussian(1)
    img.binary(thresholds)
    blobs = img.find_blobs([(90,100,-10,10,-10,10)], pixels_threshold=300, area_threshold=200)
    for b in blobs:
        img.draw_cross(b.cx(), b.cy(), size=10, color=(255,0,0), thickness=2)
    best_blob = find_best_blob(img_w, img_h, blobs)
    if best_blob != None:
        img.draw_circle(best_blob.cx(), best_blob.cy(), 10, color=(0,255,0), thickness=3)
        img.draw_rectangle(best_blob.rect())
        palm = find_palm(best_blob, img, img_w, img_h)
        img.draw_circle(palm, color=(255,175,38), thickness=3)
        t = 15
        palm_rect = (palm[0]-palm[2]-t, palm[1]-palm[2]-t, (palm[2]+t)*2, (palm[2]+t)*2)
        img.mask_rectangle(best_blob.rect())
        image.Image(img_w, img_h, sensor.BINARY, copy_to_fb=img_masked)
        #image.Image(img_w, img_h, sensor.RGB565, copy_to_fb=img_tmp)
        img.copy(copy_to_fb=img_tmp)
        img_tmp.to_bitmap().copy(copy_to_fb=img_masked)

        img_masked.draw_circle((palm[0],palm[1],palm[2]+t), color=0, fill=True, thickness=0)
        blob_fingers = img_masked.find_blobs([(1,)], pixels_threshold=100, area_threshold=100)
        print("#fingers: ", len(blob_fingers))
        for bf in blob_fingers:
            if True:
                img.draw_edges(bf.min_corners(), color=(255,0,0))
                img.draw_line(bf.major_axis_line(), color=(0,255,0))
                img.draw_line(bf.minor_axis_line(), color=(0,0,255))
            # These values are stable all the time.
            img.draw_rectangle(bf.rect())
            img.draw_cross(bf.cx(), bf.cy())
            # Note - the blob rotation is unique to 0-180 only.
            img.draw_keypoints([(bf.cx(), bf.cy(), int(math.degrees(bf.rotation())))], size=20)
    #print(clock.fps())
