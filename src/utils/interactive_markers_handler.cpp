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

#include "XBotGUI/utils/interactive_markers_handler.h"

interactive_markers_handler::interactive_markers_handler(std::string server_topic ,std::string client_topic, double scale, int objects_count_):objects_count(objects_count_)
{
    sub = node.subscribe(client_topic,1,&interactive_markers_handler::update_position,this);
    server = new interactive_markers::InteractiveMarkerServer(server_topic);
    
    for(int i=0;i<objects_count;i++)
    {
	visualization_msgs::InteractiveMarker* int_marker = new visualization_msgs::InteractiveMarker();
	int_marker->header.frame_id="/base_link";
	int_marker->scale=scale;
	interactive_markers.push_back(int_marker);
    }
}

void interactive_markers_handler::set_fixed_frame(std::string fixed_frame)
{
    for(int i=0;i<objects_count;i++)
    {
	interactive_markers.at(i)->header.frame_id=fixed_frame;
    }
}

void interactive_markers_handler::update_position(const visualization_msgs::Marker& marker_)
{
    for(int i=0;i<objects_count;i++)
    {
        if(marker_.id!=i) continue; //this implies ids starting from 0, increasing with the number of objects

	interactive_markers.at(i)->controls.clear();

	visualization_msgs::InteractiveMarkerControl box_control;
	box_control.always_visible = (marker_.action!=visualization_msgs::Marker::DELETE);
	box_control.markers.push_back( marker_);

	interactive_markers.at(i)->controls.push_back( box_control );

	if(marker_.action!=visualization_msgs::Marker::DELETE)
	{
	    visualization_msgs::InteractiveMarkerControl control;
	    
	    control.orientation.w = 1;
	    control.orientation.x = 1;
	    control.orientation.y = 0;
	    control.orientation.z = 0;
	    control.name = "rotate_x";
	    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	    interactive_markers.at(i)->controls.push_back(control);
	    control.name = "move_x";
	    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	    interactive_markers.at(i)->controls.push_back(control);
	    
	    control.orientation.w = 1;
	    control.orientation.x = 0;
	    control.orientation.y = 1;
	    control.orientation.z = 0;
	    control.name = "rotate_y";
	    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	    interactive_markers.at(i)->controls.push_back(control);
	    control.name = "move_y";
	    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	    interactive_markers.at(i)->controls.push_back(control);
	    
	    control.orientation.w = 1;
	    control.orientation.x = 0;
	    control.orientation.y = 0;
	    control.orientation.z = 1;
	    control.name = "rotate_z";
	    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	    interactive_markers.at(i)->controls.push_back(control);
	    control.name = "move_z";
	    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	    interactive_markers.at(i)->controls.push_back(control);
	}
	
	interactive_markers.at(i)->pose=marker_.pose;
	interactive_markers.at(i)->name=std::to_string(marker_.id);
	interactive_markers.at(i)->description=(marker_.action!=visualization_msgs::Marker::DELETE)?std::to_string(marker_.id):"";

	server->insert(*interactive_markers.at(i));

    }

    server->applyChanges();
}

interactive_markers_handler::~interactive_markers_handler()
{
    for(int i=0;i<objects_count;i++)
    {
	delete interactive_markers.at(i);
    }
    delete server;
}