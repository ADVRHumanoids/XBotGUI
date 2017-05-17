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

#ifndef XBOTGUI_RENDER_H
#define XBOTGUI_RENDER_H

#include <ros/ros.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <QBoxLayout>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QTabWidget>
#include "XBotGUI/interaction/module.h"

namespace XBot
{
namespace widgets
{
class render: public QWidget
{
Q_OBJECT
public:
    render();
    ~render();

    void add_display(std::string name, std::string type, std::map<std::string,std::string> properties);
    void add_frames(std::vector<std::string> names);
    void add_module(std::string name, std::map<std::string,std::string> commands);

private Q_SLOTS:
    void on_display_combo_changed();
    void on_frame_combo_changed();
    void on_display_toggle_clicked();

private:
    rviz::VisualizationManager* visualization_manager_=NULL;
    rviz::RenderPanel* render_panel_=NULL;

    std::map<std::string,rviz::Display*> displays;
    std::map<std::string,bool> displays_enable;

    QLabel frame_label;
    QComboBox frame_combo;
    QLabel display_label;
    QComboBox display_combo;
    QPushButton display_toggle;

    std::map<std::string, module*> modules;

    QTabWidget modules_tabs;

    QHBoxLayout buttons_layout;
    QHBoxLayout view_layout;
    QVBoxLayout main_layout;

};
};
};

#endif