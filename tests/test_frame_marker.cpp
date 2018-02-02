#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include "XBotGUI/interaction/frame_marker.h"

int main(int argc, char** argv)
{
    std::cout<<std::endl<<" -- Testing Frame Marker -- "<<std::endl<<std::endl;

    ros::init(argc,argv,"test_frame_marker");

    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",10);
    
    XBot::frameMarker fm;
    
    double scale = 0.1;
    double roll=0.0;
    double pitch=0.0;
    double yaw=0.0;
    double x=0.0;
    double y=0.0;
    double z=0.0;

    visualization_msgs::Marker object;
    object.type = visualization_msgs::Marker::CUBE;
    object.color.a=1;
    object.header.frame_id="/world";
    object.scale.x=0.5 * scale;
    object.scale.y=0.25 * scale;
    object.scale.z=0.125 * scale;
    object.ns="obj";
    object.id=0;
    object.color.r=1;
    object.color.g=1;
    object.color.b=1;

    ros::Time start = ros::Time::now();
    ros::Rate loop(10);
    bool enable = true;
    int sign=1;
    
    while(ros::ok())
    {
        if(ros::Time::now()-start>ros::Duration(2))
	{
	    enable=!enable;
	    sign = (sign>0)?-1:1;
	    start = ros::Time::now();
	}

	x += sign*0.1;
	y += sign*0.1;
	z += sign*0.1;
	roll += 0.1;
	pitch += 0.1;
	yaw += 0.1;
	tf::Quaternion q;
	q.setRPY(roll,pitch,yaw);

	object.pose.position.x = x;
	object.pose.position.y = y;
	object.pose.position.z = z;
	object.pose.orientation.x=q.getX();
	object.pose.orientation.y=q.getY();
	object.pose.orientation.z=q.getZ();
	object.pose.orientation.w=q.getW();

	fm.update_markers("world",object.pose,enable);

	marker_pub.publish(object);
	
	std::vector<visualization_msgs::Marker> axes;
	fm.get_markers(axes);
	for(auto axis:axes)
	{
	    marker_pub.publish(axis);
	}

	ros::spinOnce();
	
	loop.sleep();
    }
    
    return 0;
}