import sys
import numpy as np
import cv2
from copy import deepcopy
import math
import datetime

from do_sphere_calibration import create_dot_mask, do_background_subtraction_with_masking
click_x = []
click_y = []

def handle_rod_click(event, x, y, flags, param):
    # grab references to the global variables
    global click_x, click_y
 
    # if the left mouse button was clicked, record the starting
    # (x, y) coordinates and indicate that cropping is being
    # performed
    if event == cv2.EVENT_LBUTTONDOWN:
        click_x.append(x)
        click_y.append(y)

if __name__ =="__main__":
    if len(sys.argv) != 3:
        print "Requires 2 arguments: <rod_image_name> <background_image_name>"
        exit(0)

    rod_image_name = sys.argv[1]
    bg_image_name = sys.argv[2]

    bg_image = cv2.imread(bg_image_name, cv2.IMREAD_COLOR).astype(float)/255.
    rod_image = cv2.imread(rod_image_name, cv2.IMREAD_COLOR).astype(float)/255.

    if bg_image.shape != rod_image.shape:
        print "Rod and background images have different sizes!"
        exit(-1)

    # Detect any tracking dots or dark spots in the bg and ball images
    bg_mask = create_dot_mask(bg_image)
    rod_mask = create_dot_mask(rod_image)
    
    # background substraction
    rod_foreground = do_background_subtraction_with_masking(rod_image, rod_mask, bg_image, bg_mask)

    # Show rod, get clicked points around bounding box of the box    
    cv2.namedWindow("Rod")
    rod_foreground_drawing = deepcopy(rod_foreground)
    cv2.imshow("Rod", rod_foreground_drawing)
    cv2.setMouseCallback("Rod", handle_rod_click)

    click_x = []
    click_y = []
    cv2.waitKey(0)

    if len(click_x) != 4:
        print "Click only on the four corners! Saw ", len(click_x), "!=4 clicks."
        exit(-1)
    dists = []
    for i in range(4):
        i2 = (i + 1) % 4
        cv2.line(rod_foreground_drawing, (click_x[i],click_y[i]),(click_x[i2],click_y[i2]),(0,0,255),2)
        dists.append( math.sqrt(float(click_y[i2] - click_y[i])**2 + float(click_x[i2] - click_x[i])**2) )
        cv2.imshow("Rod", rod_foreground_drawing)
    distance = np.mean(np.array(dists))
    rod_side_length_mm = 6.35 # 0.25 inches -> mm
    pixel_to_mm_scale = rod_side_length_mm / distance
    print "Detected scaling: ", pixel_to_mm_scale, " mm per pixel"
    print "Detected scaling: ", 1. / pixel_to_mm_scale, " pixels per mm"

    # Save out to a file the image size, image scaling,
    # and a list of [color from bg], [normal] pairs
    f = open("rod_calib_%s.calib" % (datetime.datetime.now().isoformat()), 'w')
    f.write("image_size: %d, %d\n" % (bg_image.shape[0], bg_image.shape[1]))
    f.write("pixel_to_mm_scale: %0.5f\n" % pixel_to_mm_scale)

    f.close()

    cv2.waitKey(0)

    # close all open windows
    cv2.destroyAllWindows()
