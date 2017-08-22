import numpy as np
import cv2
from copy import deepcopy
from scipy  import optimize, spatial
import math
import datetime
import sys


if __name__ =="__main__":
    # Track the mean and variance of normals in each
    # histogram grid, as a voxel-grid-style filter.
    bins_per_axis = 20

    # Referencing https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
    # for online variance estimate in every bin
    # While all normals live on a sphere, I'm going to assume they point in roughly
    # the same direction (small variance), so assuming that they live in R^3 is not
    # a horrible approximation. This is invalid for very large variances and should
    # be revisited in that case.
    means = np.zeros([bins_per_axis, bins_per_axis, bins_per_axis, 3])
    M2 = np.zeros([bins_per_axis, bins_per_axis, bins_per_axis, 3])
    counts = np.zeros([bins_per_axis, bins_per_axis, bins_per_axis]) 

    pixel_to_mm_scale_mean = 0
    pixel_to_mm_scale_M2 = 0
    pixel_to_mm_scale_count = 0

    image_size = None

    for file in sys.argv[1:]:
        f = open(file)
        for line in f:
            contents = line.split(":")
            if contents[0] == "n_calib_pt":
                this_sample = [float(x) for x in contents[1].split(",")]
                color = np.array(this_sample[0:3])
                normal = np.array(this_sample[3:6])

                # Find the bin that it fits in
                # (assuming colors range [0, 1])
                cb = (color * bins_per_axis).astype(int)

                if np.max(cb) >= bins_per_axis or np.min(cb) < 0:
                    print "Color bin outside of valid range detected -- are colors properly normalized?"
                    exit(-1)

                counts[cb[0], cb[1], cb[2]] += 1
                delta = normal - means[cb[0], cb[1], cb[2], :]
                means[cb[0], cb[1], cb[2], :] += delta / float(counts[cb[0], cb[1], cb[2]])
                delta2 = normal - means[cb[0], cb[1], cb[2], :]
                M2[cb[0], cb[1], cb[2], :] += delta * delta2

            elif contents[0] == "pixel_to_mm_scale":
                pixel_to_mm_scale = float(contents[1])
                pixel_to_mm_scale_count += 1
                delta = pixel_to_mm_scale - pixel_to_mm_scale_mean
                pixel_to_mm_scale_mean += delta / float(pixel_to_mm_scale_count)
                delta2 = pixel_to_mm_scale - pixel_to_mm_scale_mean
                pixel_to_mm_scale_M2 += delta * delta2

            elif contents[0] == "image_size":
                this_image_size = [int(x) for x in contents[1].split(",")]
                if image_size is None:
                    image_size = this_image_size
                else:
                    if image_size != this_image_size:
                        print "Mismatched image size in input calibrations!"
                        exit(-1)

    for i in range(3):
        M2[:, :, :, i] /= (counts - 1)

    if (pixel_to_mm_scale_count > 1):
        pixel_to_mm_scale_M2 /= (pixel_to_mm_scale_count - 1)
    else:
        pixel_to_mm_scale_M2 = 0

    # Pick out a column of the color datapoints (bin centers)
    # for which we have nonzero data, and assemble our reduced
    # dataset (from which we might build a kdtree for lookup).
    color_set = np.zeros([bins_per_axis**3, 3])
    mean_var_count_set = np.zeros([bins_per_axis**3, 7])

    ind = 0
    for i in range(bins_per_axis):
        for j in range(bins_per_axis):
            for k in range(bins_per_axis):
                if counts[i, j, k] >= 2:
                    color_set[ind, :] = np.array([float(i)/bins_per_axis, float(j)/bins_per_axis, float(k)/bins_per_axis]) + 1./(2*bins_per_axis)
                    mean_var_count_set[ind, 0:3] = means[i, j, k, :]
                    mean_var_count_set[ind, 3:6] = M2[i, j, k, :]
                    mean_var_count_set[ind, 6] = counts[i, j, k]
                    ind+=1
    
    color_set.resize(ind, 3)
    mean_var_count_set.resize(ind, 7)

    # And save these out to a reduced calibration
    f = open("filtered_%s.calib" % (datetime.datetime.now().isoformat()), 'w')

    f.write("image_size: %d, %d\n" % (image_size[0], image_size[1]))
    f.write("pixel_to_mm_scale: %0.5f\n" % pixel_to_mm_scale_mean)
    f.write("pixel_to_mm_scale_variance: %0.5f\n" % pixel_to_mm_scale_M2)
    f.write("total_files_sampled: %d\n" % pixel_to_mm_scale_count)

    for i in range(ind):
        f.write("n_summary_pt: %0.4f, %0.4f, %0.4f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %d\n" % (
            color_set[i, 0], color_set[i, 1], color_set[i, 2],
            mean_var_count_set[i, 0], mean_var_count_set[i, 1], mean_var_count_set[i, 2],
            mean_var_count_set[i, 3], mean_var_count_set[i, 4], mean_var_count_set[i, 5],
            mean_var_count_set[i, 6]
            ))
    f.close()

    print "Done -- filled %d nodes at %d bin-per-axis resolution." % (ind, bins_per_axis)