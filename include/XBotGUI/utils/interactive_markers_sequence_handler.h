/*
 * Copyright (C) 2017 Centro di Ricerca "E. Piaggio" (Universita' di Pisa), IIT-ADVR
 * Author: Alessandro Settimi, Arturo Laurenzi, Luca Muratore
 * email:  ale.settimi@gmail.com, arturo.laurenzi@iit.it, luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef XBOTGUI_INTERACTIVE_MARKERS_SEQUENCE_HANDLER
#define XBOTGUI_INTERACTIVE_MARKERS_SEQUENCE_HANDLER

#include "ros/ros.h"
#include <interactive_markers/interactive_marker_server.h>
#include "XBotGUI/print_utils.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

class interactive_markers_sequence_handler
{
public:
  interactive_markers_sequence_handler(std::string server_topic ,std::string client_topic, double scale=1.0);
  void set_fixed_frame(std::string fixed_frame);
  void set_active_object(int index);
  ~interactive_markers_sequence_handler();

private:
  ros::NodeHandle node;
  tf::TransformListener tf_;
  ros::Subscriber sub;
  interactive_markers::InteractiveMarkerServer* server;
  visualization_msgs::InteractiveMarker interactive_marker;
  int active_object=0;
  
  void update_position(const visualization_msgs::Marker &marker_);
};

#endif