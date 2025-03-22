#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/adi.hpp"
#include "pros/misc.hpp"

extern Drive chassis;




// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');
inline pros::MotorGroup intake ({-20,2});
inline pros::Motor intakebottom (2);
inline pros::Motor intaketop (-20);
inline pros::Motor Lift (-9);
inline pros::adi::DigitalOut Mogo_mech('B');
inline pros::adi::DigitalOut Rdoinker('C');
inline pros::adi::DigitalOut Ldoinker('A');
inline pros::adi::DigitalOut eject('D');
inline pros::Optical color(18);
inline pros::Rotation lb(-11);
inline pros::Controller Master(pros::E_CONTROLLER_MASTER);
inline pros::adi::DigitalOut ring_rush('H');
inline pros::adi::DigitalOut scoop('G');

inline std::string team_color;
inline std::string blue = "blue";
inline std::string red = "red";
inline std::string none = "none";
inline std::string belt_state;
inline std::string on = "on";
inline std::string off = "off";


