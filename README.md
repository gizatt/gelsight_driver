# MIT RLG Gelsight Repository #

--------------------

Sandbox for drivers and experimental code for interface with Gelsight,
for use with Robot Locomotion Group research.

Dependencies: Roughly working OpenHumanoids reposities, plus
```sudo apt-get install xawtv```, which containts a command-line tool
for changing exposure values on the gelsight.



## Deriving Ground Truth ##

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

