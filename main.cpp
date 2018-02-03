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
			"Optional -ti: /Image/topic (default value: /camera/image_raw .\n"
			"Optional -td: /Depth/topic (default value: /camera/depth).\n"
			"Optional -ext: Image extension (default value: .png).\n"
			"Example: rosrun rosbag_packer rosbag_packer -i /path/to/image/directory/ -o /path/to/output.bag" << std::endl;
		return 1;
	}

	ros::start();

	int nframe;
	std::vector<std::vector<std::string>> dirnames;

	std::vector<std::string> encodings;
	encodings.push_back(sensor_msgs::image_encodings::BGR8);
	encodings.push_back(sensor_msgs::image_encodings::TYPE_32FC1);
	
	std::vector<std::string> topics;
	topics.push_back(Parser::getStringOption("-ti", "/camera/image_raw"));
	topics.push_back(Parser::getStringOption("-td", "/camera/depth"));

	if( Parser::hasOption("-i") ){
		std::vector<std::string> filenames = DUtils::FileFunctions::Dir(Parser::getOption("-i").c_str(), Parser::getStringOption("-ext", ".png").c_str(), true);
		dirnames.push_back(filenames);

		nframe = filenames.size();
		std::cout << "nImages: " << nframe << std::endl;
	}

	if( Parser::hasOption("-d") ){
		std::vector<std::string> filenames = DUtils::FileFunctions::Dir(Parser::getOption("-d").c_str(), Parser::getStringOption("-ext", ".png").c_str(), true);
		dirnames.push_back(filenames);

		if( dirnames[0].size() != dirnames[1].size() ){
			std::cerr << "Invalid pairs: images and depths do not match." << std::endl;
			return 1;
		}
	}

	// Output bag
	rosbag::Bag bag_out(Parser::getOption("-o"), rosbag::bagmode::Write);
	ros::Time t = ros::Time::now();

	for( size_t i = 0; i < nframe; i++ ){
		if(!ros::ok())
			break;

		for( size_t d = 0; d < dirnames.size(); d++ ){
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
			
			if( d == 1 ) // depth
				cvImage.image = im * Parser::getFloatOption("-s", 0.001);
			else
				cvImage.image = im;

			cvImage.encoding = encodings[d];
			cvImage.header.stamp = t;

			bag_out.write(topics[d], ros::Time(t), cvImage.toImageMsg());
		}

		double progress = (double)(i+1)/nframe;
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
