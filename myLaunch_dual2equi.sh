# Fill in the various paths involved in the conversion

calib_file=/home/guillaume/Develop/PeR-ws/dualfisheye2equi/data/RicohThetaS_calib_withTrans.xml
#image_dir=/home/guillaume/Data/PanoraMIS/Sequence4/images/
image_dir=/home/guillaume/Data/SVMISplus/Matrice600Pro/SixDOFs/Images/
mask=/home/guillaume/Develop/PeR-ws/VisualGyroscope/2017_MPP_SSD_gyroEstimation_media/images_full/maskFull.png
poses_fic=/home/guillaume/Data/SVMISplus/Matrice600Pro/SixDOFs/camera_poses.txt

echo "variables initialized"    

./dualfisheye2equi-build/dualfisheye2equi $calib_file $image_dir 0 2798 1 $mask
