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
#include "XBotGUI/interaction/im_single_widget.h"
#include <tf/transform_datatypes.h>

XBot::widgets::im_single_widget::im_single_widget(rviz::ToolManager* tool_manager_, std::string name_, std::map< std::string, XBot::object_properties > objects_)
: QWidget(), name(name_), interactive_marker_widget(tool_manager_,name_,object_combo)
{
   interactive_marker_widget.generate_objects(objects_); //also fills object_combo

   main_layout.addWidget(&object_combo);
   main_layout.addWidget(&interactive_marker_widget);
   
   setLayout(&main_layout);
   
   connect(&object_combo, SIGNAL(currentIndexChanged(int)), this, SLOT(on_object_combo_changed()));
   
   im_handler = new interactive_markers_handler(name+"_server",name+"_client",0.5,object_combo.count());
}

void XBot::widgets::im_single_widget::on_object_combo_changed()
{
    interactive_marker_widget.object_combo_changed();
}

XBot::widgets::im_single_widget::~im_single_widget()
{
    delete im_handler;
}