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

#include <XBotGUI/utils/grasp_widget.h>

XBot::widgets::grasp_widget::grasp_widget()
{
    grasp_client = nh.serviceClient<ADVR_ROS::advr_grasp_control_srv>("grasp_control");

    grasp_button.setText("Grasp");
    
    left_label.setText("Left");

    left_slider.setOrientation(Qt::Horizontal);
    left_slider.setTickInterval(1);
    left_slider.setTickPosition(QSlider::NoTicks);
    left_slider.setMinimum(0);
    left_slider.setMaximum(100);
    left_slider.setValue(0);

    left_slider_slot();

    right_label.setText("Right");

    right_slider.setOrientation(Qt::Horizontal);
    right_slider.setTickInterval(1);
    right_slider.setTickPosition(QSlider::NoTicks);
    right_slider.setMinimum(0);
    right_slider.setMaximum(100);
    right_slider.setValue(0);

    right_slider_slot();

    left_layout.addWidget(&left_label);
    left_layout.addWidget(&left_slider);
    left_layout.addWidget(&right_edit);

    right_layout.addWidget(&right_label);
    right_layout.addWidget(&right_slider);
    right_layout.addWidget(&right_edit);

    main_layout.addLayout(&left_layout);
    main_layout.addLayout(&right_layout);
    main_layout.addWidget(&grasp_button);

    connect(&grasp_button,SIGNAL(clicked()),this,SLOT(on_grasp_button_clicked()));
    connect(&left_slider, SIGNAL(valueChanged(int)), this, SLOT(left_slider_slot()));
    connect(&right_slider, SIGNAL(valueChanged(int)), this, SLOT(right_slider_slot()));

    setLayout(&main_layout);

    thread_waiting.store(false);
}

void XBot::widgets::grasp_widget::left_slider_slot()
{
    left_edit.setText(QString::number(left_slider.value(),'f',1));
}

void XBot::widgets::grasp_widget::right_slider_slot()
{
    right_edit.setText(QString::number(right_slider.value(),'f',1));
}

void XBot::widgets::grasp_widget::service_thread_body()
{
    ADVR_ROS::advr_grasp_control_srv srv;
    srv.request.left_grasp = left_edit.text().toDouble();
    srv.request.right_grasp = right_edit.text().toDouble();

    if(grasp_client.call(srv))
    {
        if(!srv.response.success) ROS_ERROR_STREAM("Service grasp_control returned FAILURE");
    }
    else
    {
	ROS_ERROR_STREAM("Error calling grasp_control service");
    }
    thread_waiting.store(false);
}

void XBot::widgets::grasp_widget::on_grasp_button_clicked()
{
    std::thread service_thread(&grasp_widget::service_thread_body,this);
    thread_waiting.store(true);

    while(thread_waiting.load())
    {
	ros::spinOnce();
    }

    if(service_thread.joinable()) service_thread.join();
    
}