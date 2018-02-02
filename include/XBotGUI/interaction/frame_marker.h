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

#ifndef XBOTGUI_FRAME_MARKER_H
#define XBOTGUI_FRAME_MARKER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

namespace XBot
{
class frameMarker
{
public:
    frameMarker(double scale=0.1);
    ~frameMarker();

    void update_markers(const std::string& fixed_frame, const geometry_msgs::Pose& pose, bool enable);
    void get_markers(std::vector<visualization_msgs::Marker>& markers);

private:
    visualization_msgs::Marker x_axis;
    visualization_msgs::Marker y_axis;
    visualization_msgs::Marker z_axis;

    tf::Matrix3x3 Mx;
    tf::Matrix3x3 My;
    tf::Matrix3x3 Mz;
};
};

#endif