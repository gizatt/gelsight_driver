# MIT RLG Gelsight Repository #

===================

Sandbox for drivers and experimental code for interface with Gelsight,
for use with Robot Locomotion Group research.

Dependencies: Roughly working OpenHumanoids reposities, plus
```sudo apt-get install xawtv```, which containts a command-line tool
for changing exposure values on the gelsight.



## Deriving GroundTruth ##

1. The system offers a utility for generating a ground-truth lookup table
from images of spheres imprinted at various locations around the
surface of the GelSight sensor. To run this utility, make sure you
first record the footage you will be using, eg. by running

    ./build/gelsight_depth_driver

with the appropriate flags so that it saves all images.

2. Next, place these images in a folder `./spherereference/` such that they
can all be accessed as `"spherereference/img_%07d.jpg"` (NOTE: This will change).

3. Next, run the groundtruth generator. At first, you won't have a
reference image to which to align the matched spheres; thus, simply
run the groundtruth gen with no arguments:

    ./build/groundtruth_gen

This will create the folder `./groundtruth/` with the following structure:

  * groundtruth/
    * spherealigned/
    * sphereextracted/
    * sphererefptimgs/
    * sphere_standard.jpg
    * circle_index.csv

This contains the ground truth information necessary to use a
lookup-table-based approach to invert the depth map. If the images
in `spherealigned/` look off-center, you can pick a new reference
image by looking in `sphereextracted/` for a more-centered candidate.
Copy this image to the top level directory:

    cp ./groundtruth/sphereextracted/img_0000XXX.jpg ./

Then, run `groundtruth_gen` again with this image as the "reference
image":

    ./build/groundtruth_gen -r ./img_0000XXX.jpg

This will produced more-centered images.

