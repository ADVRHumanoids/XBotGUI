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

#ifndef XBOTGUI_INTERACTIVE_MARKERS_HANDLER
#define XBOTGUI_INTERACTIVE_MARKERS_HANDLER

#include "ros/ros.h"
#include <interactive_markers/interactive_marker_server.h>

class interactive_markers_handler
{
public:
  interactive_markers_handler(std::string server_topic ,std::string client_topic, double scale=1.0, int objects_count_=1);
  void set_fixed_frame(std::string fixed_frame);
  ~interactive_markers_handler();

private:
  ros::NodeHandle node;
  ros::Subscriber sub;
  interactive_markers::InteractiveMarkerServer* server;
  std::vector<visualization_msgs::InteractiveMarker*> interactive_markers;
  int objects_count;
  
  void update_position(const visualization_msgs::Marker &marker_);
};

#endif