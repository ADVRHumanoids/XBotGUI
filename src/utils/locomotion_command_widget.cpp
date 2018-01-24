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

#include <XBotGUI/utils/locomotion_command_widget.h>

void fix_icon_size(QPushButton& wid)
{
    wid.setIconSize( QSize( wid.size().width(), wid.size().height() ));
}

XBot::widgets::locomotion_command_widget::locomotion_command_widget(std::string service_name_): service_name(service_name_)
{
    locomotion_client = nh.serviceClient<ADVR_ROS::advr_locomotion>(service_name.c_str());

    std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
    std::string path_to_icons = robotology_root+"/external/XBotGUI/resources/";

    QPalette palette;
    top_frame.setFrameShape(QFrame::HLine);
    top_frame.setLineWidth(2);
    palette.setColor(top_frame.foregroundRole(),Qt::gray);
    top_frame.setPalette(palette);
    mid_frame.setFrameShape(QFrame::VLine);
    mid_frame.setLineWidth(2);
    palette.setColor(mid_frame.foregroundRole(),Qt::gray);
    mid_frame.setPalette(palette);
    bottom_frame.setFrameShape(QFrame::HLine);
    bottom_frame.setLineWidth(2);
    palette.setColor(bottom_frame.foregroundRole(),Qt::gray);
    bottom_frame.setPalette(palette);

    step_label.setText("Step Lenght: ");
    step_edit.setMinimumSize(30,30);
    step_edit.setText(QString::number(0.0, 'f', 2));
    step_layout.addWidget(&step_label);
    step_layout.addWidget(&step_edit);

    walk_forward_button.setFixedSize(50,50);
    walk_forward_button.setIcon(QIcon(QString::fromStdString(path_to_icons+"up.png")));
    fix_icon_size(walk_forward_button);
    walk_backward_button.setFixedSize(50,50);
    walk_backward_button.setIcon(QIcon(QString::fromStdString(path_to_icons+"down.png")));
    fix_icon_size(walk_backward_button);
    walk_left_button.setFixedSize(50,50);
    walk_left_button.setIcon(QIcon(QString::fromStdString(path_to_icons+"left.png")));
    fix_icon_size(walk_left_button);
    walk_right_button.setFixedSize(50,50);
    walk_right_button.setIcon(QIcon(QString::fromStdString(path_to_icons+"right.png")));
    fix_icon_size(walk_right_button);
    walk_edit.setFixedSize(50,50);
    walk_edit.setText(QString::number(0.0, 'f', 2));
    walk_layout.addWidget(&walk_forward_button,0,1);
    walk_layout.addWidget(&walk_left_button,1,0);
    walk_layout.addWidget(&walk_edit,1,1);
    walk_layout.addWidget(&walk_right_button,1,2);
    walk_layout.addWidget(&walk_backward_button,2,1);
    
    turn_left_button.setFixedSize(50,50);
    turn_left_button.setIcon(QIcon(QString::fromStdString(path_to_icons+"turn_left.png")));
    fix_icon_size(turn_left_button);
    turn_right_button.setFixedSize(50,50);
    turn_right_button.setIcon(QIcon(QString::fromStdString(path_to_icons+"turn_right.png")));
    fix_icon_size(turn_right_button);
    turn_edit.setFixedSize(50,50);
    turn_edit.setText(QString::number(0.0, 'f', 2));
    turn_layout.addWidget(&turn_left_button);
    
    turn_layout.addWidget(&turn_edit);
    turn_layout.addWidget(&turn_right_button);
    
    commands_layout.addLayout(&walk_layout);
    commands_layout.addWidget(&mid_frame);
    commands_layout.addLayout(&turn_layout);
    units_label.setText("Remember! [m] and [deg]");;
    main_layout.addLayout(&step_layout);
    main_layout.addWidget(&top_frame);
    main_layout.addLayout(&commands_layout);
    main_layout.addWidget(&bottom_frame);
    main_layout.addWidget(&units_label);

    connect(&walk_forward_button,SIGNAL(clicked()),this,SLOT(on_walk_forward_button_clicked()));
    connect(&walk_backward_button,SIGNAL(clicked()),this,SLOT(on_walk_backward_button_clicked()));
    connect(&walk_left_button,SIGNAL(clicked()),this,SLOT(on_walk_left_button_clicked()));
    connect(&walk_right_button,SIGNAL(clicked()),this,SLOT(on_walk_right_button_clicked()));
    connect(&turn_left_button,SIGNAL(clicked()),this,SLOT(on_turn_left_button_clicked()));
    connect(&turn_right_button,SIGNAL(clicked()),this,SLOT(on_turn_right_button_clicked()));

    setLayout(&main_layout);

    thread_waiting.store(false);
}

void XBot::widgets::locomotion_command_widget::service_thread_body()
{
    ADVR_ROS::advr_locomotion srv;
    srv.request.step_length = step_edit.text().toDouble();
    srv.request.command_type = command;
    if(command == "turn_left" || command == "turn_right")
    {
	srv.request.command_value = turn_edit.text().toDouble();
    }
    else
    {
	srv.request.command_value = walk_edit.text().toDouble();
    }
      
    if(!locomotion_client.call(srv))
    {
	ROS_ERROR_STREAM("Error calling "<<service_name<<" service");
    }
    thread_waiting.store(false);
}

void XBot::widgets::locomotion_command_widget::on_walk_forward_button_clicked()
{
    command = "walk_forward";
    call_service();
}

void XBot::widgets::locomotion_command_widget::on_walk_backward_button_clicked()
{
    command = "walk_backward";
    call_service();
}

void XBot::widgets::locomotion_command_widget::on_walk_left_button_clicked()
{
    command = "walk_left";
    call_service();
}

void XBot::widgets::locomotion_command_widget::on_walk_right_button_clicked()
{
    command = "walk_right";
    call_service();
}

void XBot::widgets::locomotion_command_widget::on_turn_left_button_clicked()
{
    command = "turn_left";
    call_service();
}

void XBot::widgets::locomotion_command_widget::on_turn_right_button_clicked()
{
    command = "turn_right";
    call_service();
}

void XBot::widgets::locomotion_command_widget::call_service()
{
    std::thread service_thread(&locomotion_command_widget::service_thread_body,this);
    thread_waiting.store(true);

    while(thread_waiting.load())
    {
	ros::spinOnce();
    }

    if(service_thread.joinable()) service_thread.join();   
}