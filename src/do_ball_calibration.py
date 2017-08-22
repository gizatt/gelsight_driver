import numpy as np
import cv2
from copy import deepcopy
from scipy  import optimize
import math
import datetime
import sys

# Circle fitting modified from
# http://scipy-cookbook.readthedocs.io/items/Least_Squares_Circle.html
def fit_circle_to_pts(x, y):
    x = np.array(x)
    y = np.array(y)

    def calc_R(xc, yc):
        """ calculate the distance of each data points from the center (xc, yc) """
        return np.sqrt((x-xc)**2 + (y-yc)**2)

    def f_2b(c):
        """ calculate the algebraic distance between the 2D points and the mean circle centered at c=(xc, yc) """
        Ri = calc_R(*c)
        return Ri - Ri.mean()

    def Df_2b(c):
        """ Jacobian of f_2b
        The axis corresponding to derivatives must be coherent with the col_deriv option of leastsq"""
        xc, yc     = c
        df2b_dc    = np.empty((len(c), x.size))

        Ri = calc_R(xc, yc)
        df2b_dc[0] = (xc - x)/Ri                   # dR/dxc
        df2b_dc[1] = (yc - y)/Ri                   # dR/dyc
        df2b_dc    = df2b_dc - df2b_dc.mean(axis=1)[:, np.newaxis]

        return df2b_dc

    x_m = np.mean(np.array(x))
    y_m = np.mean(np.array(y))
    center_estimate = x_m, y_m
    center_2b, ier = optimize.leastsq(f_2b, center_estimate, Dfun=Df_2b, col_deriv=True)

    xc_2b, yc_2b = center_2b
    Ri_2b        = calc_R(*center_2b)
    R_2b         = Ri_2b.mean()
    residu_2b    = np.sum((Ri_2b - R_2b)**2)

    return (xc_2b, yc_2b, R_2b, residu_2b)

click_x = []
click_y = []

def handle_circle_click(event, x, y, flags, param):
    # grab references to the global variables
    global click_x, click_y
 
    # if the left mouse button was clicked, record the starting
    # (x, y) coordinates and indicate that cropping is being
    # performed
    if event == cv2.EVENT_LBUTTONDOWN:
        click_x.append(x)
        click_y.append(y)


def create_dot_mask(input_image):
    dot_threshold = 1.5
    mask = np.greater(np.sum(input_image, axis=2), dot_threshold).astype(float)
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)
    return mask

def do_background_subtraction_with_masking(image, image_mask, bg_image, bg_mask):
    foreground = image - bg_image
    foreground_mask = np.logical_and(image_mask, bg_mask).astype(float)
    # Expand foreground mask from MxN to MxNx3 (full color)
    foreground_mask = np.expand_dims(foreground_mask, 2)
    foreground_mask = np.tile(foreground_mask, [1, 1, 3])
    foreground = foreground * foreground_mask
    return foreground

if __name__ =="__main__":
    if len(sys.argv) != 4:
        print "Requires 3 arguments: <ball_image_name> <background_image_name> <calibration file with scaling"
        exit(0)

    ball_image_name = sys.argv[1]
    bg_image_name = sys.argv[2]
    calibration_file = sys.argv[3]

    f = open(calibration_file)
    for line in f:
        contents = line.split(":")
        if contents[0] == "pixel_to_mm_scale":
            pixel_to_mm_scale = float(contents[1])
        elif contents[0] == "image_size":
            image_size = [int(x) for x in contents[1].split(",")]

    bg_image = cv2.imread(bg_image_name, cv2.IMREAD_COLOR).astype(float)/255.
    ball_image = cv2.imread(ball_image_name, cv2.IMREAD_COLOR).astype(float)/255.

    if bg_image.shape != ball_image.shape:
        print "Ball and background images have different sizes!"
        exit(-1)

    if bg_image.shape[0] != image_size[0] or bg_image.shape[1] != image_size[1]:
        print "Ball and background image have different size from calibration file!"
        print bg_image.shape[0:2], " vs ", image_size
        exit(-1)

    # Detect any tracking dots or dark spots in the bg and ball images
    bg_mask = create_dot_mask(bg_image)
    ball_mask = create_dot_mask(ball_image)
    
    # background substraction
    ball_foreground = do_background_subtraction_with_masking(ball_image, ball_mask, bg_image, bg_mask)

    # Get circle points
    cv2.namedWindow("Ball")
    ball_foreground_drawing = (ball_foreground + 1.0) / 2.0
    cv2.imshow("Ball", ball_foreground_drawing)
    cv2.setMouseCallback("Ball", handle_circle_click)
    click_x = []
    click_y = []
    cv2.waitKey(0)

    # Fit a circle to the suggested points
    xc, yc, rc, residual_c = fit_circle_to_pts(click_x, click_y)
    xc = int(xc)
    yc = int(yc)
    rc = int(rc)
    cv2.circle(ball_foreground_drawing,(xc,yc), rc, (0,0,255), 2)
    cv2.imshow('Ball', ball_foreground_drawing)

    # Calculate how much of the spherical bearing is present
    circle_radius_mm = pixel_to_mm_scale * rc
    bearing_radius_mm = 6.35/2. # 0.25 inches -> mm
    print "Circle radius in mm: ", circle_radius_mm
    if circle_radius_mm > bearing_radius_mm:
        print "Circle radius of ", circle_radius_mm, " is greater than bearing radius of ", bearing_radius_mm
        normals = None
        samples_x_range = None
        samples_y_range = None
    else:
        theta = math.acos(circle_radius_mm / bearing_radius_mm)
        penetration_depth = bearing_radius_mm * (1 - math.sin(theta))
        # Bearing height over this domain:
        # At radius r from center of visible circle:
        #  Gamma = acos(r / bearing_radius)
        #  Visible height is bearing_radius*sin(gamma) + penetration_depth - R
        print "Detecting penetration depth of ", penetration_depth, " at angle ", theta

        # Create a grid the same size as our input image (cropped down to near the circle)
        samples_x_range = range(xc - rc, xc + rc + 1)
        samples_y_range = range(yc - rc, yc + rc + 1)
        samples_x, samples_y = np.meshgrid(samples_x_range, samples_y_range, indexing="ij")
        # Calculate the distance to the center of the circle at each point
        samples = np.sqrt((samples_x - xc)**2 + (samples_y - yc)**2)
        # And from there to the elevation
        samples = np.arccos(samples * pixel_to_mm_scale / bearing_radius_mm)
        # Generate the normal map from that -- normals face in the direction of
        # [x distance from center of circle, y distance from center of circle, sphere height at that point]
        normals_x = (samples_x - xc)*pixel_to_mm_scale
        normals_y = (samples_y - yc)*pixel_to_mm_scale
        normals_z = bearing_radius_mm * np.sin(samples)
        normals = np.dstack([normals_x, normals_y, normals_z])
        normals = normals / np.linalg.norm(normals, axis=2)[:, :, np.newaxis]

    # Save out to a file the image size, image scaling,
    # and a list of [color from bg], [normal] pairs
    f = open("ball_%s.calib" % (datetime.datetime.now().isoformat()), 'w')
    f.write("image_size: %d, %d\n" % (bg_image.shape[0], bg_image.shape[1]))
    f.write("pixel_to_mm_scale: %0.5f\n" % pixel_to_mm_scale)

    print len(samples_x_range), len(samples_y_range), ball_foreground.shape, normals.shape
    if normals is not None:
        for i in range(normals.shape[0]):
            for j in range(normals.shape[1]):
                im_j = samples_x_range[i] # Flip coordinate indexing due to a flip in opencv's dimensioning
                im_i = samples_y_range[j]
                if np.sum(ball_foreground[im_i, im_j, :]) > 0.001:
                    f.write("n_calib_pt: %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f\n" % (
                        ball_foreground[im_i, im_j, 0], ball_foreground[im_i, im_j, 1], ball_foreground[im_i, im_j, 2],
                        normals[i, j, 0], normals[i, j, 1], normals[i, j, 2]))
    f.close()

    cv2.waitKey(0)

    # close all open windows
    cv2.destroyAllWindows()
