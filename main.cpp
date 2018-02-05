#include "common.h"
#include "parser.h"
#include "Thirdparty/DLib/FileFunctions.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rosbag_packer");
	Parser::init(argc, argv);

	if( !Parser::hasOption("-i") || !Parser::hasOption("-o") ){
		std::cout << "Mandatory -i: /Path/to/image/directory/.\n"
			"Mandatory -o: /Path/to/output.bag.\n"
			"Optional -d: /Path/to/depth/directory/.\n"
			"Optional -s: Depth scale (default value: 0.001).\n"
			"Optional -ext: Image extension (default value: .png).\n"
			"Example: rosrun rosbag_packer rosbag_packer -i /path/to/image/directory/ -o /path/to/output.bag" << std::endl;
		return 1;
	}

	ros::start();

	int nImage, nDepth;
	std::vector<std::string> imageFiles, depthFiles;

	if( Parser::hasOption("-i") ){
		imageFiles = DUtils::FileFunctions::Dir(Parser::getOption("-i").c_str(), Parser::getStringOption("-ext", ".png").c_str(), true);

		nImage = imageFiles.size();
		std::cout << "nImage: " << nImage << std::endl;
	}

	if( Parser::hasOption("-d") ){
		depthFiles = DUtils::FileFunctions::Dir(Parser::getOption("-d").c_str(), Parser::getStringOption("-ext", ".png").c_str(), true);

		nDepth = depthFiles.size();
		std::cout << "nDepth: " << nDepth << std::endl;
	}

	sensor_msgs::CameraInfo rgb_info, depth_info;
	
	rgb_info.distortion_model = "plumb_bob";
	rgb_info.D = {1081.3720703125, 0.0, 959.5, 0.0, 1081.3720703125, 539.5, 0.0, 0.0, 1.0};
	rgb_info.K = {1081.3720703125, 0.0, 959.5, 0.0, 1081.3720703125, 539.5, 0.0, 0.0, 1.0};
	rgb_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	rgb_info.P = {1081.3720703125, 0.0, 959.5, 0.0, 0.0, 1081.3720703125, 539.5, 0.0, 0.0, 0.0, 1.0, 0.0};
	rgb_info.header.frame_id = "/openni_rgb_optical_frame";
	
	depth_info.distortion_model = "plumb_bob";
	depth_info.D = {1081.3720703125, 0.0, 959.5, 0.0, 1081.3720703125, 539.5, 0.0, 0.0, 1.0};
	depth_info.K = {1081.3720703125, 0.0, 959.5, 0.0, 1081.3720703125, 539.5, 0.0, 0.0, 1.0};
	depth_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	depth_info.P = {1081.3720703125, 0.0, 959.5, 0.0, 0.0, 1081.3720703125, 539.5, 0.0, 0.0, 0.0, 1.0, 0.0};
	depth_info.header.frame_id = "/openni_rgb_optical_frame";


	// Output bag
	rosbag::Bag bag_out(Parser::getOption("-o"), rosbag::bagmode::Write);
	ros::Time t = ros::Time::now();

	// Write RGB image
	for( size_t i = 0; i < nImage; i++ ){
		if(!ros::ok())
			break;

		cv::Mat im = cv::imread(imageFiles[i], CV_LOAD_IMAGE_UNCHANGED);

		// Grab ros::Time
		std::vector<char*> tokens;
		char* token = strtok((char*)imageFiles[i].c_str(), "/.");
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

		// Assign values
		cv_bridge::CvImage cvImage;
		cvImage.image = im;
		cvImage.encoding = sensor_msgs::image_encodings::BGR8;
		cvImage.header.stamp = t;
		cvImage.header.seq = i;
		cvImage.header.frame_id = "/openni_rgb_optical_frame";

		rgb_info.height = im.rows;
		rgb_info.width = im.cols;
		rgb_info.header.stamp = t;
		rgb_info.header.seq = i;

		// Wirte ROS bag
		bag_out.write("/camera/rgb/image_color", ros::Time(t), cvImage.toImageMsg());
		bag_out.write("/camera/rgb/camera_info", ros::Time(t), rgb_info);

		// Update progress bar
		double progress = (double)(i+1)/(nImage+nDepth);
		int barWidth = 70;

		std::cout << "[";
		int pos = barWidth * progress;
		for (int i = 0; i < barWidth; ++i) {
			if (i < pos) std::cout << "=";
			else if (i == pos) std::cout << ">";
			else std::cout << " ";
		}
		std::cout << "] " << int(progress * 100.0) << " %     \r";
		std::cout.flush();
	}

	// Write DEPTH image
	for( size_t i = 0; i < nDepth; i++ ){
		if(!ros::ok())
			break;

		cv::Mat im = cv::imread(depthFiles[i], CV_LOAD_IMAGE_UNCHANGED);

		// Compensate depth scale
		im.convertTo(im,CV_32FC1);
		im *= Parser::getFloatOption("-s", 0.001);

		// Grab ros::Time
		std::vector<char*> tokens;
		char* token = strtok((char*)depthFiles[i].c_str(), "/.");
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

		// Assign values
		cv_bridge::CvImage cvImage;
		cvImage.image = im;
		cvImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
		cvImage.header.stamp = t;
		cvImage.header.seq = i;
		cvImage.header.frame_id = "/openni_rgb_optical_frame";

		depth_info.height = im.rows;
		depth_info.width = im.cols;
		depth_info.header.stamp = t;
		depth_info.header.seq = i;

		// Write ROS bag 
		bag_out.write("/camera/depth/image", ros::Time(t), cvImage.toImageMsg());
		bag_out.write("/camera/depth/camera_info", ros::Time(t), depth_info);

		// Update progress bar
		double progress = (double)(i+nImage+1)/(nImage+nDepth);
		int barWidth = 70;

		std::cout << "[";
		int pos = barWidth * progress;
		for (int i = 0; i < barWidth; ++i) {
			if (i < pos) std::cout << "=";
			else if (i == pos) std::cout << ">";
			else std::cout << " ";
		}
		std::cout << "] " << int(progress * 100.0) << " %     \r";
		std::cout.flush();
	}

	std::cout << std::endl;
	bag_out.close();

	return 0;
}
