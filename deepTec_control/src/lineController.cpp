#include "deepTec_control/lineController.h"

lineController::lineController():it_(nh_){
	ROS_INFO("Line follower");
	lines_sub_ = nh_.subscribe<deepTec_control::LineArrayStamped>("/hough_lines/lines",1, &lineController::linesCB,this);
	action_pub_ = nh_.advertise<std_msgs::String>("/command", 1);
}

lineController::~lineController(){

}

void lineController::linesCB(const deepTec_control::LineArrayStamped msg){
	//ROS_INFO_STREAM("SIZE OF: " <<msg.lines.size());
	if (msg.lines.size()!=0){
		float x1 = msg.lines[0].pt1.x;
		float x2 = msg.lines[0].pt2.x;
		float y1 = -msg.lines[0].pt1.y;
		float y2 = -msg.lines[0].pt2.y;
		ROS_INFO_STREAM("x1: "<<x1<<" y1: "<<y1<<" x2: "<<x2<<" y2: "<<y2);
		actionCompute(computeCross(x1,x2,y1,y2));
	}
}


float lineController::computeCross(float x1, float x2, float y1, float y2){
	float m = (y2 - y1) / (x2 - x1);
	float y = -300;
	float result = (m*x1 - y1 + y) / m;
	ROS_INFO_STREAM("m: "<<m);
	ROS_INFO_STREAM("result: " << result);		
}

void lineController::actionCompute(float x){
	ROS_INFO_STREAM("Computing....");
}

int main(int argc, char **argv){
	ros::init(argc, argv, "lineController");
	lineController nh_;
	ros::Rate rate(12);
	while(ros::ok()){
		ros::spinOnce();
	}	
	return 0;
}


