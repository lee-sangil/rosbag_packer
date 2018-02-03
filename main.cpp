#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Time.h>
#include <std_msgs/Header.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "Thirdparty/DLib/FileFunctions.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rosbag_packer");
	ros::start();

	if(argc != 5 && argc != 7)
	{
		std::cerr << "Usage: rosrun rosbag_packer rosbag_packer /path/to/image/directory/ /image/topic (optional: /path/to/depth/directory/ /depth/topic) <image extension .ext> /path/to/output.bag" << std::endl;
		return 0;
	}

	int Nf;
	int Nd = std::floor((argc-3)/2);
	std::vector<std::vector<std::string>> dirnames;
	std::vector<std::string> encodings;
	encodings.push_back(sensor_msgs::image_encodings::BGR8);
	encodings.push_back(sensor_msgs::image_encodings::MONO16);

	for( size_t d = 0; d < Nd; d++ ){
		// Vector of paths to image
		std::vector<std::string> filenames = DUtils::FileFunctions::Dir(argv[1+2*d], argv[argc-2], true);
		dirnames.push_back(filenames);

		if( d > 0 && filenames.size() != Nf ){
			std::cout << "Invalid image pairs" << std::endl;
			return -1;
		}

		Nf = filenames.size();
		std::cout << d+1 << "-th dir's Images: " << Nf << std::endl;
	}

	// Output bag
	rosbag::Bag bag_out(argv[argc-1], rosbag::bagmode::Write);
	ros::Time t = ros::Time::now();

	for( size_t i = 0; i < Nf; i++ ){
		if(!ros::ok())
			break;

		for( size_t d = 0; d < Nd; d++ ){
			cv::Mat im = cv::imread(dirnames[d][i], CV_LOAD_IMAGE_UNCHANGED);

			std::vector<char*> tokens;
			char* token = strtok((char*)dirnames[d][i].c_str(), "/.");
			while( token != NULL ){
				tokens.push_back(token);
				token = strtok(NULL, "/.");
			}

			tokens.pop_back();
			std::string usec = std::string(tokens.back());
			tokens.pop_back();
			std::string sec = std::string(tokens.back());
			std::string ctime = sec+'.'+usec;
			double time = atof(ctime.c_str());

			ros::Time t = ros::Time(time);

			cv_bridge::CvImage cvImage;
			cvImage.image = im;
			cvImage.encoding = encodings[d];
			cvImage.header.stamp = t;

			bag_out.write(argv[2+2*d], ros::Time(t), cvImage.toImageMsg());
		}

		double progress = (double)(i+1)/Nf;
		int barWidth = 70;

		std::cout << "[";
		int pos = barWidth * progress;
		for (int i = 0; i < barWidth; ++i) {
			if (i < pos) std::cout << "=";
			else if (i == pos) std::cout << ">";
			else std::cout << " ";
		}
		std::cout << "] " << int(progress * 100.0) << " %\r";
		std::cout.flush();
	}

	std::cout << std::endl;
	bag_out.close();

	return 0;
}
