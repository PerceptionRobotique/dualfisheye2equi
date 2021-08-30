dualfisheye2equi: warps a dualfisheye image to an equirectangular image

August 2017, August 2021
Author: G. Caron
Contact: guillaume.caron@u-picardie.fr

Prerequisities
0. CMake (version 3.14.5 tested)
1. ViSP (version 3.2.0 tested)
2. libPeR_base (version 0.0.2 tested, https://github.com/PerceptionRobotique/libPeR_base)

Configure and prepare equi2omni to build with catkin
0. export PER_DIR=/path/to/libPeR_base/build/
1. create a "build" directory in the same directory than CMakeLists.txt and cd in
2. run ccmake .., configure and generate
3. make

Run dualfisheye2equi
0. Create a "media" directory next to the "build" directory
1. Download examples of dualfisheye images in the media directory: https://mis.u-picardie.fr/~panoramis/Sequences/Sequence7-L1-0-9.zip (please see the PanoraMIS official website https://mis.u-picardie.fr/~panoramis for the full list of image sequences)
2. run from the command line
  ./dualfisheye2equi ../data/RicohThetaS_calib.xml ../media/ 0 9 1 ../media/maskFull.png ../media/poses.txt 1
command line arguments are:
* xmlFic the dualfisheye camera calibration xml file (the data directory stores an example of a dualfisheye camera xml file)
* imagesDir directory where dualfisheye images to read (with 6 digits before the extension) are and where the output equirectangular images will be written (with characters 'e_' before the 6 digits)
* iFirst the number of the first image to transform
* iLast the number of the last image to transform
* iStep (optional if none after) the increment to the next image to transform
* maskFic (optional if none after) the dualfisheye image mask to discard useless areas (png file: a black pixel is not to be transformed)
* posesFic (optional if none after) a text file of one 3D pose per image (one row - one image) stored as the 3 elements of the translation vector followed by the 3 elements of the axis-angle vector representation of the rotation
* iInvertPose (optional) a flag to let the program knows if poses of posesFic must be applied to images as they are or inversed

TODO: remove boost dependencies currently obtained through ViSP

