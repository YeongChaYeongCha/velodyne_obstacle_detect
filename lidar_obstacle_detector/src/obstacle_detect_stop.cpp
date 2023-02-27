#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

int totall_object_num=0;

void BoundingBox_Callback(const jsk_recognition_msgs::BoundingBoxArray& object){
	/*
	object_num=object.size();
	object_x_pos=object.boxes.pose.position.x;
	object_y_pos=object.boxes.pose.position.y;
	
	
	ROS_INFO("object X : %f  |  object Y : %f", object_x_pos, object_y_pos);
	*/
	int object_num=0;
	totall_object_num=object.boxes.size();
	
	float object_x_pos[totall_object_num]={0, };
	float object_y_pos[totall_object_num]={0, };
	ROS_INFO("------------------------------");
	ROS_INFO("------------------------------");
	ROS_INFO("------------------------------");
	ROS_INFO("------------------------------");
	//ROS_INFO("Object Number : %d", totall_object_num);
	//ROS_INFO("1.	x : %f	|	y : %f", object.boxes[0].pose.position.x, object.boxes[1].pose.position.y);
	
	for(int i=0; i<totall_object_num; i++){
		object_x_pos[i]=object.boxes[i].pose.position.x;
		object_y_pos[i]=object.boxes[i].pose.position.y;
		
		if((object_x_pos[i]<=0.0 && object_x_pos[i]>=-4.0) && abs(object_y_pos[i])<=2.0){
		//if((object_y_pos>0.0 && object_y_pos<=2.0) && (abs(object_x_pos)<1.0)){
			object_num++;			
			ROS_INFO("%d. X : %f	|	Y :%f", object_num, object_x_pos[i], object_y_pos[i]);
		}
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "object_detect_emergency_stop");
	ros::NodeHandle nh;
	
	ros::Subscriber sub_object=nh.subscribe("/obstacle_detector/jsk_bboxes", 1, BoundingBox_Callback);
	
	ros::spin();
	
	return 0;
}
