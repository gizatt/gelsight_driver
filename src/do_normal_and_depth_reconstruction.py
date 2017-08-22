import numpy as np
import cv2
from copy import deepcopy
from scipy  import optimize, spatial
import math
import datetime
import sys

from do_ball_calibration import create_dot_mask, do_background_subtraction_with_masking

if __name__ =="__main__":
    # Load in the calibration file
    if len(sys.argv) != 2:
        print "Must supply a filtered calibration file!"
        exit(-1)

    color_set = []
    mean_var_count_set = []

    f = open(sys.argv[1])
    for line in f:
        contents = line.split(":")
        if contents[0] == "pixel_to_mm_scale":
            pixel_to_mm_scale = float(contents[1])
        elif contents[0] == "image_size":
            image_size = [int(x) for x in contents[1].split(",")]
        elif contents[0] == "n_summary_pt":
            data = [float(x) for x in contents[1].split(",")]
            color_set.append(data[0:3])
            mean_var_count_set.append(data[3:])

    color_set = np.array(color_set)
    mean_var_count_set = np.array(mean_var_count_set)
    mean_set = mean_var_count_set[:, 0:3]
    variance_set = mean_var_count_set[:, 3:6]
    count_set = mean_var_count_set[:, 6]

    color_tree = spatial.cKDTree(color_set)

    cap = cv2.VideoCapture(0)
    cv2.namedWindow("Reconstructions")
    ret, bg_image = cap.read()
    bg_image = bg_image.astype(float)/255.
    if bg_image.shape[0] != image_size[0] or bg_image.shape[1] != image_size[1]:
        print "Current webcam doesn't match required image size from calibration."
        exit(-1)

    bg_mask = create_dot_mask(bg_image)

    while(True):
        # Capture frame-by-frame
        ret, image = cap.read()
        image = image.astype(float)/255.

        # Display the resulting frame
        image_mask = create_dot_mask(image)
        # background substraction
        foreground = do_background_subtraction_with_masking(image, image_mask, bg_image, bg_mask)

        scaled_image = cv2.resize(image,None,fx=0.25, fy=0.25, interpolation = cv2.INTER_CUBIC)
        pts_distances, pts_inds = color_tree.query(scaled_image, k=10)
        # Todo: use variances and counts to weight how I pick my normals?
        normals_image = np.mean(mean_set[pts_inds, :], 2)

        full_size_normals_image = cv2.resize(normals_image, (foreground.shape[1], foreground.shape[0]))


        foreground_display = (foreground + 1.0)/2.0
        normals_display = (full_size_normals_image + 1.0) / 2.0

        print "forground: ", np.mean(foreground_display, axis=(0, 1))
        print "normals: ", np.mean(normals_display, axis=(0, 1))

        print normals_display[200, 200, :]
        print normals_display[200, 201, :]
        print normals_display[200, 202, :]

        display_image = np.concatenate((foreground_display, normals_display), axis=1)
        cv2.imshow('Reconstructions',display_image)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break
        elif key & 0xFF == ord('b'):
            bg_image = deepcopy(image)
            bg_mask = deepcopy(image_mask)
        


    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()