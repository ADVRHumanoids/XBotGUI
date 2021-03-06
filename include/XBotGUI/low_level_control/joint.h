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
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFrame>
#include <QTimer>
#include <urdf_parser/urdf_parser.h>
#include <XBotInterface/RobotInterface.h>
#include <atomic>

#define RAD2DEG 180.0/3.1415
#define DEG2RAD 3.1415/180.0

namespace XBot
{
namespace widgets
{
class QBoxedLabel: public QFrame
{
public:
    QBoxedLabel();
    ~QBoxedLabel();

    QLabel label;
    QHBoxLayout layout;
};

class joint: public QFrame
{
Q_OBJECT
public:
    joint(std::string name_, boost::shared_ptr <const urdf::Joint > URDFjoint_, XBot::ControlMode control_mode_);
    ~joint();
    void set(double q_sense);
    void get(double& q_sense);

    void enable(bool enable_);

private Q_SLOTS:
    void slider_slot();
    void slider_action();
    void timer_slot();

private:
    QLabel title;
    QBoxedLabel min;
    QBoxedLabel max;
    QBoxedLabel current;
    QSlider slider;
    QHBoxLayout labels_layout;
    QLabel control_mode_label;
    QVBoxLayout main_layout;

    std::string name;
    XBot::ControlMode control_mode;
    boost::shared_ptr <const urdf::Joint > URDFjoint;

    bool initialized = false;
    std::atomic_int desired_q;
    std::atomic_bool disable_sense;
    QTimer sense_timer;
};
};
};

#endif