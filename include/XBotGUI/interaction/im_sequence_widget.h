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

#ifndef XBOTGUI_IM_SEQUENCE_WIDGET_H
#define XBOTGUI_IM_SEQUENCE_WIDGET_H

#include <ros/ros.h>
#include <rviz/tool_manager.h>
#include <std_msgs/String.h>
#include <QBoxLayout>
#include <QComboBox>
#include <QPushButton>
#include "XBotGUI/interaction/interactive_marker_widget.h"
#include "XBotGUI/utils/interactive_markers_sequence_handler.h"
#include <atomic>

namespace XBot
{
namespace widgets
{
class im_sequence_widget: public QWidget
{
Q_OBJECT
public:
    im_sequence_widget(rviz::ToolManager* tool_manager_, std::string name_, std::map<int,object_properties> objects_);
    ~im_sequence_widget();

    void set_fixed_frame(std::string frame);

private Q_SLOTS:
    void on_object_combo_changed();
    void on_add_button_clicked();
    void on_delete_last_button_clicked();
    void on_delete_all_button_clicked();

private:
    QComboBox object_combo;
    QPushButton add_button;
    QPushButton delete_last_button;
    QPushButton delete_all_button;
    QHBoxLayout cmd_layout;
    QVBoxLayout main_layout;

    std::string name;
    ros::NodeHandle nh;
    im_widget interactive_marker_widget;
    interactive_markers_sequence_handler* im_handler;

    int active_markers=0;
    int next_marker_index=0;
    std::map<int,object_properties> objects;
};
};
};

#endif