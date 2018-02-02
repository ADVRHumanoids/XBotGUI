#include "XBotGUI/interaction/frame_marker.h"

XBot::frameMarker::frameMarker(double scale)
{
    Mx.setRPY(0,0,0);
    My.setRPY(0,0,M_PI/2);
    Mz.setRPY(0,-M_PI/2,0);
    
    x_axis.color.a=1;
    x_axis.scale.x=1.0 * scale;
    x_axis.scale.y=0.1 * scale;
    x_axis.scale.z=0.1 * scale;
    x_axis.id=0;
    x_axis.ns="frame";
    x_axis.color.r=1;
    
    y_axis.color.a=1;
    y_axis.scale.x=1.0 * scale;
    y_axis.scale.y=0.1 * scale;
    y_axis.scale.z=0.1 * scale;
    y_axis.id=1;
    y_axis.ns="frame";
    y_axis.color.g=1;
    
    z_axis.color.a=1;
    z_axis.scale.x=1.0 * scale;
    z_axis.scale.y=0.1 * scale;
    z_axis.scale.z=0.1 * scale;
    z_axis.id=2;
    z_axis.ns="frame";
    z_axis.color.b=1;
}

void XBot::frameMarker::get_markers(std::vector< visualization_msgs::Marker >& markers)
{
    markers.clear();

    markers.push_back(x_axis);
    markers.push_back(y_axis);
    markers.push_back(z_axis);
}

void XBot::frameMarker::update_markers(const std::string& fixed_frame, const geometry_msgs::Pose& pose, bool enable)
{
      x_axis.header.frame_id=fixed_frame;
      y_axis.header.frame_id=fixed_frame;
      z_axis.header.frame_id=fixed_frame;

      x_axis.pose.position = pose.position;
      y_axis.pose.position = pose.position;
      z_axis.pose.position = pose.position;

      tf::Matrix3x3 M;
      tf::Quaternion q;
      q.setX(pose.orientation.x);
      q.setY(pose.orientation.y);
      q.setZ(pose.orientation.z);
      q.setW(pose.orientation.w);

      M.setRotation(q);

      tf::Matrix3x3 temp_M;
      tf::Quaternion temp_q;

      temp_M = M*Mx;
      temp_M.getRotation(temp_q);
      
      x_axis.pose.orientation.x=temp_q.getX();
      x_axis.pose.orientation.y=temp_q.getY();
      x_axis.pose.orientation.z=temp_q.getZ();
      x_axis.pose.orientation.w=temp_q.getW();
      
      temp_M = M*My;
      temp_M.getRotation(temp_q);
      
      y_axis.pose.orientation.x=temp_q.getX();
      y_axis.pose.orientation.y=temp_q.getY();
      y_axis.pose.orientation.z=temp_q.getZ();
      y_axis.pose.orientation.w=temp_q.getW();
      
      temp_M = M*Mz;
      temp_M.getRotation(temp_q);
      
      z_axis.pose.orientation.x=temp_q.getX();
      z_axis.pose.orientation.y=temp_q.getY();
      z_axis.pose.orientation.z=temp_q.getZ();
      z_axis.pose.orientation.w=temp_q.getW();
      
      if(enable)
      {
	  x_axis.action = visualization_msgs::Marker::ADD;
	  y_axis.action = visualization_msgs::Marker::ADD;
	  z_axis.action = visualization_msgs::Marker::ADD;
      }
      else
      {
	  x_axis.action = visualization_msgs::Marker::DELETE;
	  y_axis.action = visualization_msgs::Marker::DELETE;
	  z_axis.action = visualization_msgs::Marker::DELETE;
      }
}

XBot::frameMarker::~frameMarker()
{

}