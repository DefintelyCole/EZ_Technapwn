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
inline pros::Motor intakebottom (20);
inline pros::Motor intaketop (2);
inline pros::Motor Lift (-9);
inline pros::adi::DigitalOut Mogo_mech('H');
inline pros::adi::DigitalOut scoop('G');
inline pros::adi::DigitalOut eject('E');
inline pros::Optical color(13);
inline pros::Rotation lb(11);
inline pros::Controller Master(pros::E_CONTROLLER_MASTER);
inline pros::adi::DigitalOut ring_rush('D');

inline std::string team_color;
inline std::string blue = "blue";
inline std::string red = "red";
inline std::string none = "none";


