
dataset=/media/icsl/CAE2-F955/ICSL-DE/icsl-de-pioneer
for i in 3 4
do
	roslaunch rosbag_packer rosbag_packer.launch path:=${dataset}-$i/ file:=icsl-de-pioneer-$i.bag
done

#dataset=/media/icsl/CAE2-F955/ICSL-DE/icsl-de-place-items
#for i in 1 2 3 4
#do
#	./Examples/RGB-D/rgbd_tum ./Vocabulary/ORBvoc.txt ./Examples/RGB-D/ASUS.yaml ${dataset}-$i/ ${dataset}-$i/associations.txt
#	mkdir ${dataset}-$i/ORB-VO/
#	mv CameraTrajectory.txt ${dataset}-$i/ORB-VO/CameraTrajectory.txt
#done
