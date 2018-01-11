#include "opencv_apps/morphologyOpp.h"
#define erosize 3
#define dilsize 3

morphologyOpp::morphologyOpp():
	it_(nh_),
	pnh_("~")
{
	ROS_INFO("Operaciones morfologicas by Alex... :T");

	//Subscripciones...
	cam_sub_ = it_.subscribe("/zed/rgb/image_rect_color", 1, &morphologyOpp::cameraCB, this);
	lines_sub_ = it_.subscribe("/hough_lines/image", 1, &morphologyOpp::linesCB, this);
	//Publishers...
	img_pub_ = it_.advertise("morphologyOpp/output", 1);
	erosion_img_pub_ = it_.advertise("morphologyOpp/erosion", 1);
	dilation_img_pub_ = it_.advertise("morphologyOpp/dilation", 1);
	cerradura_img_pub_ = it_.advertise("morphologyOpp/cerradura", 1);
	apertura_img_pub_ = it_.advertise("morphologyOpp/apertura", 1);
}

morphologyOpp::~morphologyOpp(){

}

void morphologyOpp::Apply(const sensor_msgs::ImageConstPtr& msg, int selector){
	cv::Mat input_img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8) ->image;

	cv::Size s = input_img.size();
	cv::Rect ROI(0,s.height/10,s.width,s.height*0.9);
	cv::Mat cropped = input_img(ROI);
	cv::Mat src_gray;
	cv::cvtColor( cropped, src_gray, cv::COLOR_BGR2GRAY );
	//creacion de elemento estructurante	
	cv::Mat element_ero = cv::getStructuringElement(cv::MORPH_RECT, 
							cv::Size( 2*erosize + 1, 2*erosize + 1),
							cv::Point( erosize, erosize) );

	cv::Mat element_dil = cv::getStructuringElement(cv::MORPH_RECT,
							cv::Size( 2*dilsize + 1, 2*dilsize + 1),
							cv::Point (dilsize, dilsize) );
	//dilatacion TODO:switch de selector
	//TODO: probar con diferentes formas de elementos estructurantes
//	dilation_out = Dilation(src_gray, element_dil);

	//erotion TODO:switch de selector
	//TODO: probar con diferentes formas de elementos estructurantes
//	erosion_out = Erosion(src_gray, element_ero);	

	//TODO:Apertura y cerradura
	//Apertura: Primero erosion y luego dilatacion
//	apertura_out = Dilation(Erosion(src_gray, element_ero), element_dil);

	//Cerradura: Primero dilatacion y luego erosion
	cerradura_out = Erosion(Dilation(src_gray, element_dil), element_ero);	
	//aplicar suavizado Gaussiano

//	dilation_out = GaussBlur(dilation_out);
//	erosion_out = GaussBlur(erosion_out);
//	apertura_out = GaussBlur(apertura_out);
	cerradura_out = GaussBlur(cerradura_out);

	//Convertir a mensaje de cv a mensaje de ros
//	sensor_msgs::ImagePtr msg_dil = cv_bridge::CvImage(std_msgs::Header(), "mono8", dilation_out).toImageMsg();
////	sensor_msgs::ImagePtr msg_ero = cv_bridge::CvImage(std_msgs::Header(), "mono8", erosion_out).toImageMsg();
//	sensor_msgs::ImagePtr msg_ape = cv_bridge::CvImage(std_msgs::Header(), "mono8", apertura_out).toImageMsg();
	sensor_msgs::ImagePtr msg_cer = cv_bridge::CvImage(std_msgs::Header(), "mono8", cerradura_out).toImageMsg();

	//Publish outputs
//	erosion_img_pub_.publish(msg_ero);
//	dilation_img_pub_.publish(msg_dil);
	cerradura_img_pub_.publish(msg_cer);
//	apertura_img_pub_.publish(msg_ape);
		
}

void morphologyOpp::cameraCB(const sensor_msgs::ImageConstPtr& msg){
	Apply(msg, 1);
}

void morphologyOpp::linesCB(const sensor_msgs::ImageConstPtr& msg){
	cv::Mat input_img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8) ->image;
	cv::Mat bgr[3];
	int i, j;
	cv::split(src, bgr);
	cv::Mat blueIm = bgr[0];
	cv::Size s = blueIm.size();
	int blueHistogram[s.width];
	for (i=0;i<=s.width;i++){
		for (j=0;j<s.height;j++){
			blueHistogram[i]+=(blueIm.at<char>(j,i)/255);
		}
	}
	ROS_INFO_STREAM("Del primer for paso");
	cv::Mat histImage(s,CV_8UC1,0);
	for (i=1;i<s.width;i++){
		cv::line(histImage, cv::Point(blueHistogram[i-1],i-1), cv::Point(blueHistogram[i],i), cv::Scalar(255),2,8,0);
	}
	ROS_INFO_STREAM("de aqui tambien paso");

  	if(!histImage.empty()){
        	cv::imshow("histo", histImage);
    	}
}

cv::Mat morphologyOpp::Erosion(cv::Mat input_img, cv::Mat element){
	cv::Mat erosion_tout;
	erode(input_img, erosion_tout, element);
	return erosion_tout;
}

cv::Mat morphologyOpp::Dilation(cv::Mat input_img, cv::Mat element){
	cv::Mat dilation_tout;
	dilate(input_img, dilation_tout, element);
	return dilation_tout;
}

cv::Mat morphologyOpp::GaussBlur(cv::Mat input_img){
	cv::Mat GB_temp;
	GaussianBlur(input_img, GB_temp, cv::Size(5,5),0,0,0);
	return GB_temp;
}


int main(int argc, char **argv){
	ros::init(argc, argv, "morphologyOpp");
	morphologyOpp nh;
	ros::Rate rate(12);
	while(ros::ok()){
		ros::spinOnce();
	}	
	return 0;
}






