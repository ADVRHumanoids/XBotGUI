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

#ifndef XBOTGUI_MODULE_H
#define XBOTGUI_MODULE_H

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <QBoxLayout>
#include <QPushButton>
#include <QWidget>
#include "XBotGUI/utils/command_widget.h"
#include "XBotGUI/utils/pose_command_widget.h"
#include "XBotGUI/utils/goal_command_widget.h"
#include "XBotGUI/utils/sequence_command_widget.h"
#include "XBotGUI/utils/cmd_service_widget.h"
#include "XBotGUI/utils/empty_service_widget.h"
#include "XBotGUI/utils/grasp_widget.h"
#include "XBotGUI/utils/string_command_widget.h"
#include "XBotGUI/utils/click_command_widget.h"
#include "XBotGUI/utils/status_widget.h"
#include "XBotGUI/utils/led_status_widget.h"
#include "XBotGUI/utils/postural_command_widget.h"
#include "XBotGUI/utils/traj_utils_move_reset_widget.h"
#include <rviz/tool_manager.h>
#include <urdf_parser/urdf_parser.h>

namespace XBot
{
namespace widgets
{
class module: public QWidget
{
Q_OBJECT
public:
    module(boost::shared_ptr<urdf::ModelInterface const> urdf, std::string name_, std::vector<std::vector<std::map<std::string,std::string>>> command_blocks_, std::vector<std::vector<std::map<std::string,std::string>>> status_blocks, std::vector<std::string> module_dependencies, rviz::ToolManager* tool_manager_=NULL);
    ~module();

    QPushButton* get_switch_button();
    void set_fixed_frame(std::string frame);

    void status_changed(std::string name);

private Q_SLOTS:
    void on_switch_button_clicked();
    void status_timer_body();

private:
    void start_info(bool error, std::string plugin_name, bool called);
    void stop_info(bool error, std::string plugin_name, bool called);

    std::string name;
    ros::NodeHandle nh;
    std::vector<ros::ServiceClient> switch_client;
    std_srvs::SetBool switch_service;

    QPushButton switch_button;
    status_widget status_wid;

    std::vector<command_widget*> command_widgets;

    QVBoxLayout main_layout;
    QHBoxLayout basic_layout;
    std::vector<QHBoxLayout*> h_layout;

    std::vector<led_status_widget*> led_status_widgets;
    QHBoxLayout led_layout;

    std::string last_status;
    QTimer status_timer;
    void status_callback(const std_msgs::String& status);
    ros::Subscriber status_sub;
};
};
};

#endif