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

#ifndef XBOTGUI_JOINT_H
#define XBOTGUI_JOINT_H

#include <QWidget>
#include <QLabel>
#include <QSlider>
#include <QBoxLayout>
#include <urdf_parser/urdf_parser.h>

#define RAD2DEG 180.0/3.1415
#define DEG2RAD 3.1415/180.0

namespace XBot
{
namespace widgets
{
class joint: public QWidget
{
Q_OBJECT
public:
    joint(std::string name_,boost::shared_ptr<urdf::ModelInterface const> urdf);
    ~joint();

private Q_SLOTS:
    void slider_slot();

private:
    QLabel title;
    QSlider slider;
    QVBoxLayout main_layout;

    std::string name;
};
};
};

#endif