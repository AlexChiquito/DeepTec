#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <deepTec_control/LineArrayStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/String.h>
#include <opencv2/core/core.hpp>

class lineController{
	public:
	lineController();
	~lineController();
	
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber im_sub_;
	ros::Subscriber lines_sub_;
	ros::Subscriber scan_sub_;
	ros::Publisher action_pub_;

	float m;
	float x_reference;
	float m_reference;
	bool obstacle; 
	
	float computeCross(float x1, float x2, float y1, float y2);
	void actionCompute(float x);
	void scanCB(const sensor_msgs::LaserScan msg);
	void linesCB(const deepTec_control::LineArrayStamped msg);
};
