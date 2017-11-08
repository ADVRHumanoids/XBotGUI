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

#include <XBotGUI/utils/click_command_widget.h>

XBot::widgets::click_command_widget::click_command_widget(rviz::ToolManager* tool_manager_, std::string topic_name_, std::string command_name_):
topic_name(topic_name_), command_name(command_name_), tool_manager(tool_manager_)
{
    click_tool = tool_manager->addTool("rviz/PublishPoint");
    click_tool->getPropertyContainer()->subProp("Topic")->setValue(topic_name.c_str());

    click_sub = nh.subscribe(topic_name.c_str(),1,&click_command_widget::click_callback,this);

    click_button.setCheckable(true);
    click_button.setText(QString::fromStdString("Click: "+command_name));

    main_layout.addWidget(&click_button);

    connect(&click_button,SIGNAL(clicked()),this,SLOT(on_click_button_clicked()));
    
    setLayout(&main_layout);
}

void XBot::widgets::click_command_widget::on_click_button_clicked()
{
    if(click_button.isChecked())
    {
	tool_manager->setCurrentTool(click_tool);
	click_button.setText("ABORT Clicking");
    }
    else
    {
	tool_manager->setCurrentTool(tool_manager->getDefaultTool());
	click_button.setText(QString::fromStdString("Click: "+command_name));
    }
}

void XBot::widgets::click_command_widget::click_callback(const geometry_msgs::PointStamped& point)
{
    click_button.setChecked(false);
    on_click_button_clicked();
}