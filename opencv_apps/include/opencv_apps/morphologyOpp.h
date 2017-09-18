#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>

#include <opencv2/core/core.hpp>


class morphologyOpp {
public:
	morphologyOpp();
	~morphologyOpp();

	ros::NodeHandle nh_, pnh_;	
	image_transport::ImageTransport it_;
	image_transport::Publisher img_pub_;
	image_transport::Publisher erosion_img_pub_;
	image_transport::Publisher dilation_img_pub_;
	image_transport::Publisher cerradura_img_pub_;
	image_transport::Publisher apertura_img_pub_;
	image_transport::Subscriber cam_sub_;

	cv::Mat  Erosion(cv::Mat input_img, cv::Mat element);
	cv::Mat  Dilation(cv::Mat input_img, cv::Mat element);
	cv::Mat	 GaussBlur(cv::Mat input_img);
	void Apply(const sensor_msgs::ImageConstPtr& msg, int selector);
	void cameraCB(const sensor_msgs::ImageConstPtr& msg);
	
	cv::Mat src, erosion_out, dilation_out, cerradura_out, apertura_out;
	
	int erosion_elem;
	int erosion_size;
	int dilation_elem;
	int dilation_size;	

};


