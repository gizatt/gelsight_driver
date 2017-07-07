# MIT RLG GelSight Driver #

--------------------

This repository includes both calibration and driver code for using
a GelSight touch sensor.

## Installation and Dependencies ##

This repository has been tested on 
[Ubuntu 14.04](https://travis-ci.org/gizatt/gelsight_driver). It probably
works on other versions of Linux, at least, but they are not actively
supported yet. (One thing at a time.)

For quick setup on Ubuntu, use:

```
sudo apt-get install cmake pkg-config libopencv-dev xawtv
```

`pkg-config` is necessary for the build, `opencv` is used for grabbing
images from webcams and some image processing, and `xawtv` is used to
adjust exposure settings on the GelSight webcam.

To build, follow the standard CMake workflow:
```
mkdir -p build
cd build
cmake ..
make
make install
```

Installed binaries are placed in `build/install/bin`.

## Running a GelSight Sensor ##

The data flow with a GelSight sensor is:
1) The webcam inside of the camera takes an RGB image.
2) The driver maps from RGB to normal vector at each pixel using a calibration
profile.
3) The driver outputs a depth map for external consumption (which, for now,
can be done via a render window to observe the pretty pictures, or over
[LCM](https://lcm-proj.github.io/)).

## Running the Depth Driver ##

Assuming you have a calibration file available, you can run the
depth driver with, for example,

```build/install/bin/gelsight_depth_driver 0 -v 1 -o 0 -l trained_lookup.dat```

Running `gelsight_depth_driver` with no arguments will cause it to spit out
usage information.


## Generating Calibration Data ##

Included in this repository is a calibration file `trained_lookup.dat`, but
it will almost certainly not work well for your sensor. To generate your
own lookup table, you will need a small ball bearing as a calibration target.
(`TODO(gizatt): McMaster/Amazon link?`)

Procedure:
1. Use the depth driver to collect raw images of the ball bearing being
rolled around on the sensor surface. Use the command
```build/install/bin/gelsight_depth_driver <webcam #> -v 1 -o calibration_ims```,
and slowly roll the ball bearing around on the GelSight surface so that there are
plenty of images of the ball in all parts of the view. Be patient -- 5+ minutes
worth of data helps a lot.
2. `TODO(gizatt) Get hands on a gelsight and try this so I can write the next step
correctly.`

## Old README starts here, pls ignore

1. Use `groundtruth_gen` utility to automatically generate a
ground-truth look-up table from reference footage of ball bearings
on the gelsight surface. To run this utility, make sure you
first record the footage you will be using, eg. by running

        ./build/gelsight_depth_driver [ARGS]

  with the appropriate arguments so that it records all footage to some
  output folder.

2. Next, run the groundtruth generator on these image:

        ./build/groundtruth_gen [path_to_imgs]

  for example, if the images were in the `output` folder and were  named
  `img_0000000.jpg` - `img_0000314.jpg` then one could run:

        ./build/groundtruth_gen output/img_%07d.jpg

  This will create the folder `./groundtruth/` with the following structure:

  * `groundtruth/`
    * `spherealigned/`
    * `sphereextracted/`
    * `sphererefptimgs/`
    * `sphere_standard.jpg`
    * `circle_index.csv`

  This contains the ground truth information necessary to use a
  look-up table  to invert the depth map (for reference, the final
  derived sphere images are in `sphererefptimgs/`).

4. If the images in `spherealigned/` look off-center,
  you can pick a new reference
  image by looking in `sphereextracted/` for a more-centered or more-clear
  candidate. Copy the desired image to the top level directory:

        cp ./groundtruth/sphereextracted/img_0000XXX.jpg ./my_reference_img.jpg

  Then, run `groundtruth_gen` again with this image as the "reference
  image", eg.:

        ./build/groundtruth_gen output/img_%07d.jpg -r ./my_reference_img.jpg

  This will produced final images that are more centered.

