#include <opencv2/opencv.hpp>

#include <ros/console.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"

#include <yaml-cpp/yaml.h>


std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) 
{
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "CSI_CAMERA_NODE");

    ros::NodeHandle node;

    YAML::Node RSM_config  = YAML::LoadFile("../config/rsm_uart.yaml");  
	std::string camera_counter_topic_name = RSM_config["camera_counter_topic_name"].as<std::string>();
    std::string camera_data_path = RSM_config["camera_data_path"].as<std::string>();

    ros::Publisher camera_counter_pub = node.advertise<std_msgs::UInt32>(camera_counter_topic_name, 1000);

    std_msgs::UInt32 camera_counter_msg;

    int capture_width = 1280;
    int capture_height = 720;
    int display_width = 1280;
    int display_height = 720;
    int framerate = 3;
    int flip_method = 0;

    std::string pipeline = gstreamer_pipeline(capture_width,
	capture_height,
	display_width,
	display_height,
	framerate,
	flip_method);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";
 
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if(!cap.isOpened()) 
    {
	    std::cout<<"Failed to open camera."<<std::endl;
	    return (-1);
    }

    // cv::namedWindow("CSI Camera", cv::WINDOW_AUTOSIZE);
    cv::Mat img;

    uint32_t counter = 0;
    while(ros::ok())
    {
    	if (!cap.read(img)) 
        {
		    std::cout<<"Capture read error"<<std::endl;
		    break;
	}

        camera_counter_msg.data = counter++;
        
        std::string path = camera_data_path + "/" +  std::to_string(counter) + "_" + std::to_string(ros::Time::now().toNSec()) + ".jpg";

        cv::imwrite(path, img);
        camera_counter_pub.publish(camera_counter_msg);


#ifdef DEBUG
	//    cv::imshow("CSI Camera",img);
	    int keycode = cv::waitKey(10) & 0xff; 
            if (keycode == 27) break;

#endif
    }

    cap.release();

#ifdef DEBUG
    cv::destroyAllWindows();
#endif

    return 0;
}
