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
#include "XBotGUI/interaction/im_sequence_widget.h"
#include "XBotGUI/print_utils.h"
#include <tf/transform_datatypes.h>

XBot::widgets::im_sequence_widget::im_sequence_widget(rviz::ToolManager* tool_manager_, std::string name_, std::map< std::string, XBot::object_properties > objects_)
: QWidget(), name(name_), interactive_marker_widget(tool_manager_,name_,object_combo,true)
{
   for(auto object_:objects_)
   {
      objects[object_.first] = object_.second;
      objects_names[next_marker_index] = object_.first;
      next_marker_index++;
   }
   next_marker_index=0;

   add_button.setText("Add");
   delete_last_button.setText("Delete Last");
   delete_all_button.setText("Delete All");
   
   cmd_layout.addWidget(&object_combo);
   cmd_layout.addWidget(&add_button);
   cmd_layout.addWidget(&delete_last_button);
   cmd_layout.addWidget(&delete_all_button);
   main_layout.addLayout(&cmd_layout);
   main_layout.addWidget(&interactive_marker_widget);
   
   setLayout(&main_layout);
   
   connect(&object_combo, SIGNAL(currentIndexChanged(int)), this, SLOT(on_object_combo_changed()));
   connect(&add_button, SIGNAL(clicked(bool)), this, SLOT(on_add_button_clicked()));
   connect(&delete_last_button, SIGNAL(clicked(bool)), this, SLOT(on_delete_last_button_clicked()));
   connect(&delete_all_button, SIGNAL(clicked(bool)), this, SLOT(on_delete_all_button_clicked()));
   
   im_handler = new interactive_markers_sequence_handler(name+"_server",name+"_client",0.5);
}

void XBot::widgets::im_sequence_widget::set_fixed_frame(std::string frame)
{
    im_handler->set_fixed_frame(frame);
    interactive_marker_widget.set_fixed_frame(frame);
}

void XBot::widgets::im_sequence_widget::on_object_combo_changed()
{
    interactive_marker_widget.object_combo_changed();
    im_handler->set_active_object(object_combo.currentIndex());
}

void XBot::widgets::im_sequence_widget::on_add_button_clicked()
{
    objects.at(objects_names.at(next_marker_index)).id=active_markers;
    interactive_marker_widget.add_object(objects.at(objects_names.at(next_marker_index)));
    QString number;
    number.setNum(active_markers);
    object_combo.addItem(number);
    object_combo.setCurrentIndex(active_markers);

    active_markers++;
    next_marker_index++;
    if(next_marker_index==objects.size()) next_marker_index=0;
    
    im_handler->set_active_object(object_combo.currentIndex());
}

void XBot::widgets::im_sequence_widget::on_delete_last_button_clicked()
{
    interactive_marker_widget.delete_last_object();
    object_combo.removeItem(object_combo.count()-1);
    active_markers--;
    
    if(active_markers>0)
    {
	object_combo.setCurrentIndex(active_markers-1);
	im_handler->set_active_object(object_combo.currentIndex());
    }
    else
    {
        im_handler->set_active_object(-1);
    }
}

void XBot::widgets::im_sequence_widget::on_delete_all_button_clicked()
{
    while(object_combo.count()>0)
    {
	on_delete_last_button_clicked();
    }

    active_markers=0;
    next_marker_index=0;
}

XBot::widgets::im_sequence_widget::~im_sequence_widget()
{
    delete im_handler;
}