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
#include "XBotGUI/interaction/pi.h"

XBot::widgets::pi::pi(): QWidget()
{
    render_panel_ = new rviz::RenderPanel();
    render_panel_->setMinimumHeight(30);
    render_panel_->setMinimumWidth(30);
    
    visualization_manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( visualization_manager_->getSceneManager(), visualization_manager_ );
    visualization_manager_->initialize();
    visualization_manager_->startUpdate();

    tool_manager_ = visualization_manager_->getToolManager();

    visualization_tabs.addTab(render_panel_,"Render");
    
    control_layout.addWidget(&modules_tabs);
    control_layout.addWidget(&im_tabs);
    control_layout.setStretch(0,1);
    control_layout.setStretch(1,1);

    view_layout.addWidget(&visualization_tabs);
    view_layout.addLayout(&control_layout);
    view_layout.setStretch(0,2);
    view_layout.setStretch(1,1);
    
    frame_label.setText("Fixed Frame:");
    display_label.setText("Display:");
    display_toggle.setCheckable(true);

    buttons_layout.addWidget(&frame_label);
    buttons_layout.addWidget(&frame_combo);
    QFrame* myFrame = new QFrame();
    QPalette palette;
    myFrame->setFrameShape(QFrame::VLine);
    myFrame->setLineWidth(2);
    palette.setColor(myFrame->foregroundRole(),Qt::gray);
    myFrame->setPalette(palette);
    buttons_layout.addWidget(myFrame);
    buttons_layout.addWidget(&display_label);
    buttons_layout.addWidget(&display_combo);
    buttons_layout.addWidget(&display_toggle);
    
    main_layout.addLayout(&buttons_layout);
    main_layout.addLayout(&view_layout);

    setLayout(&main_layout);

    connect(&display_combo, SIGNAL(currentIndexChanged(int)), this, SLOT(on_display_combo_changed()));
    connect(&frame_combo, SIGNAL(currentIndexChanged(int)), this, SLOT(on_frame_combo_changed()));
    connect(&display_toggle, SIGNAL(clicked(bool)), this, SLOT(on_display_toggle_clicked()));
}

void XBot::widgets::pi::set_robot_name(std::string robot_name_)
{
    robot_name=robot_name_;
}

void XBot::widgets::pi::add_interactive_marker(std::string name,std::map<std::string,object_properties> objects)
{
    im_widgets[name] =  new im_widget(tool_manager_,name,objects);
    im_tabs.addTab(im_widgets.at(name),QString::fromStdString(name));
    std::map<std::string,std::string> properties;
    add_display(name,"rviz/InteractiveMarkers",properties);
    displays.at(name)->setTopic(("/"+name+"_server/update").c_str(),"visualization_msgs/InteractiveMarkerUpdate");
}

void XBot::widgets::pi::add_module(std::string name, std::map<std::string,std::string> commands)
{
    if(modules.count(name)) return;

    modules[name] = new module(name,commands);
    modules_tabs.addTab(modules.at(name),name.c_str());
}

void XBot::widgets::pi::on_display_toggle_clicked()
{
    if(display_toggle.isChecked())
    {
	displays.at(display_combo.currentText().toStdString())->setEnabled(true);
	display_toggle.setText("Disable");
	displays_enable.at(display_combo.currentText().toStdString()) = true;
    }
    else
    {
	displays.at(display_combo.currentText().toStdString())->setEnabled(false);
	display_toggle.setText("Enable");
	displays_enable.at(display_combo.currentText().toStdString()) = false;
    }
}

void XBot::widgets::pi::on_display_combo_changed()
{
    if(displays_enable.at(display_combo.currentText().toStdString()))
    {
	display_toggle.setChecked(true);
	display_toggle.setText("Disable");
    }
    else
    {
	display_toggle.setChecked(false);
	display_toggle.setText("Enable");
    }
}

void XBot::widgets::pi::on_frame_combo_changed()
{
    visualization_manager_->setFixedFrame(frame_combo.currentText().toStdString().c_str());
}

void XBot::widgets::pi::add_frames(std::vector< std::string > names)
{
    for(auto name:names)
    {
	frame_combo.addItem(QString::fromStdString(name));
    }

    frame_combo.setCurrentIndex(0);
    visualization_manager_->setFixedFrame(frame_combo.currentText().toStdString().c_str());
}

void XBot::widgets::pi::add_display(std::string name, std::string type, std::map< std::string, std::string > properties)
{
    displays[name] =  visualization_manager_->createDisplay( type.c_str(), name.c_str(), true );
    displays_enable[name] = true;
    display_combo.addItem(QString::fromStdString(name));

    if(type=="rviz/Image")
    {
	visualization_tabs.addTab(displays.at(name)->getAssociatedWidget(),"Camera");
    }

    if(type=="rviz/RobotModel")
    {
	displays.at(name)->subProp("Robot Description")->setValue(("/xbotcore/" + robot_name + "/robot_description").c_str());
    }

    for(auto property:properties)
    {
	displays.at(name)->subProp(property.first.c_str())->setValue(property.second.c_str());
    }
}

XBot::widgets::pi::~pi()
{
    delete visualization_manager_;
    delete render_panel_;

    for(auto module:modules)
    {
	delete module.second;
    }
}