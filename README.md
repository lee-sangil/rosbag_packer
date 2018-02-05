# Rosbag Packer

ROS package to generate a rosbag from a collection of images. Images are ordered in a time sequence. The timestamp for each image is assigned according to the filename of image. 

The bag will publish the images to topic as specified.

Tested in ROS Kinetic.

## Installation

In your ROS_PACKAGE_PATH (check your environment variable ROS_PACKAGE_PATH):

	git clone https://github.com/lee-sangil/rosbag_packer.git rosbag_packer
	cd ${catkin_workspace}
	catkin_make

## Usage:

	rosrun rosbag_packer rosbag_packer -i /PATH/TO/IMAGES/ -d /PATH/TO/DEPTH/ -ext IMAGE_EXTENSION -o /PATH/TO/OUPUT.BAG

- `PATH_TO_IMAGES`: Path to the folder with the images
- `IMAGE_EXTENSION`: .jpg, .png, etc. write the dot "." (default is .png)
- `PATH_TO_OUTPUT_BAG`: Path to save the bag (including the filename e.g. directory/filename.bag)

