#include "deepTec_control/lineController.h"

lineController::lineController():it_(nh_){
	ROS_INFO("Line follower");
	
	lines_sub_ = nh_.subscribe<deepTec_control::LineArrayStamped>("/hough_lines/lines", 1, &lineController::linesCB, this);
	scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, &lineController::scanCB, this);

	action_pub_ = nh_.advertise<std_msgs::Int32>("/command", 1);
	
	x1 = 0.0;
	x2 = 0.0;
	y1 = 0.0;
	y2 = 0.0;

	x_reference = 980;
	m_reference = -1.30;
}

lineController::~lineController(){

}

void lineController::linesCB(const deepTec_control::LineArrayStamped msg){
	//ROS_INFO_STREAM("SIZE OF: " <<msg.lines.size());
	if (msg.lines.size()!=0){
		x1 = msg.lines[0].pt1.x;
		x2 = msg.lines[0].pt2.x;
		y1 = -msg.lines[0].pt1.y;
		y2 = -msg.lines[0].pt2.y;
		ROS_INFO_STREAM("x1: "<<x1<<" y1: "<<y1<<" x2: "<<x2<<" y2: "<<y2);
	}
	if (!obstacle){
		actionCompute(computeCross(x1,x2,y1,y2));
	}
}

void lineController::scanCB(const sensor_msgs::LaserScan msg){
	int i;
	double min_range = 0.05; //Limites de rango para saber si un dato es o no usable
	double max_range = 10.0;
	double r;
	int scan_size;
	scan_size = msg.ranges.size();
	std_msgs::Int32 action_msg;
	for(i = 400; i < 700; i++){ 
		r = msg.ranges[i];
		if(r > min_range && r < max_range)
			if (r < 4){
				obstacle = true;
				action_msg.data = 1;
				action_pub_.publish(action_msg);	
			}else{
				obstacle = false;
				action_msg.data = 0;
				action_pub_.publish(action_msg);
			}
			
	}	
}

float lineController::computeCross(float x1, float x2, float y1, float y2){
	m = (y2 - y1) / (x2 - x1);
	float y = -500;
	float result = (m*x1 - y1 + y) / m;
	ROS_INFO_STREAM("m: "<<m);
	ROS_INFO_STREAM("result: " << result);
	return result;		
}

void lineController::actionCompute(float x){
	std_msgs::Int32 msg;
	//m_reference = -1.3       x_reference = 980
	if (m>(m_reference + 0.5) && m < 0){  		//pendiente mayor que la referencia y menor que 0
		ROS_INFO_STREAM("derecha");
		msg.data = 3;
	}else if (m < (m_reference - 0.5) || m > 0){   	//pendiente menor que la referencia o mayor que 0
		ROS_INFO_STREAM("izquierda");
		msg.data = 4;
	}else{
		ROS_INFO_STREAM("derecho");
		msg.data = 2;
	}

	if (x < (x_reference - 50)){		
		ROS_INFO_STREAM("izquierda!");
		msg.data = 4;		
	}else if(x > (x_reference + 50)){
		ROS_INFO_STREAM("derecha");
		msg.data = 3;
	}else{
		ROS_INFO_STREAM("pa delante");
		msg.data = 2;
	}
	ROS_INFO_STREAM("Computing....");

	action_pub_.publish(msg);
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


