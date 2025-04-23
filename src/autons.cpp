#include "autons.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// std::string team_color;
// std::string blue = "blue";
// std::string red = "red";

// void colorsort() {
   
//   if (team_color == red){
//     if(color.get_hue() > 185){
//       pros::delay(1);
//       eject.set_value(1);
//       pros::delay(300);
//       eject.set_value(0);
//     }
//   }
//   else if (team_color == blue) {
//     if(color.get_hue() < 40){
//       pros::delay(1);
//       eject.set_value(1);
//       pros::delay(300);
//       eject.set_value(0);
//     }
   
//   }
//   else{
//     eject.set_value(0);
//   }


// }


// pros::Task ColorSortTask([]{
//         while (true) {
//             colorsort();
//             pros::delay(10);
//         }
//     });

// These are out of 127
const int DRIVE_SPEED = 90;
const int TURN_SPEED = 80;
const int SWING_SPEED = 90;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(25.0, 0.0, 15.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(15.0, 0.05, 200.0, 15.0);   // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 5_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

   chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}


void RedWP() {
  
  team_color = red;
  
  chassis.odom_xyt_set(0_in, 0_in, 190_deg);
  nextstate();
  nextstate();
  pros::delay(600);
  chassis.pid_drive_set(-2, 100, true);
  nextstate();
  nextstate();
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-97,80);
  chassis.pid_wait();
  chassis.pid_drive_set(-21, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-13, 50, true);
  chassis.pid_wait_quick_chain();
  Mogo_mech.set_value(1);
  chassis.pid_wait_quick_chain();
  pros::delay(100);
  intake.move(127);
  chassis.pid_turn_set(30,80);
  chassis.pid_wait();
  chassis.pid_drive_set(15, 100, true);
  chassis.pid_wait();
  chassis.pid_turn_set(0,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10, 80, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(20,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-23, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(24, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-78,127);
  chassis.pid_wait();
  chassis.pid_drive_set(45, 70, true);
  Rdoinker.set_value(1);
  pros::delay(2500);
  chassis.pid_turn_set(135,127);
  chassis.pid_wait();
  chassis.pid_drive_set(50, 127, true);
  pros::delay(600);
  Rdoinker.set_value(0);
  pros::delay(900);
  nextstate();
  nextstate();
//   chassis.pid_turn_set(-160,80);
//   chassis.pid_wait_quick_chain();
//   chassis.pid_drive_set(43, 100, true);
//   chassis.pid_wait_quick_chain();
//   Rdoinker.set_value(1);
//   chassis.pid_drive_set(-15, 100, true);
//   chassis.pid_wait_quick_chain();
//   chassis.pid_turn_set(125,80);
//   pros::delay(200);
//   Rdoinker.set_value(0);
//   chassis.pid_wait_quick_chain();
//   chassis.pid_drive_set(18, 100, true);
//   chassis.pid_wait_quick_chain();
}

void RedSigWP() {
  
  team_color = red;
  
  chassis.odom_xyt_set(0_in, 0_in, 190_deg);
  nextstate();
  nextstate();
  pros::delay(600);
  chassis.pid_drive_set(-2, 100, true);
  nextstate();
  nextstate();
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-100,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-21, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-13, 50, true);
  chassis.pid_wait_quick_chain();
  Mogo_mech.set_value(1);
  chassis.pid_wait_quick_chain();
  pros::delay(100);
  intake.move(127);
  chassis.pid_turn_set(30,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(17, 100, true);
  chassis.pid_wait();
  chassis.pid_turn_set(0,100);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(12, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(30,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-70, 127, true);
  pros::delay(900);
  Mogo_mech.set_value(0);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-60,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-34, 60, true);
  chassis.pid_wait_quick_chain();
  Mogo_mech.set_value(1);
  pros::delay(100);
  // nextstate();
  // nextstate();
  chassis.pid_turn_set(180,127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20, 110, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(190,127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-40, 127, true);
  pros::delay(400);
}

  void Red6Ring() {
  
  team_color = red;
  
  chassis.odom_xyt_set(0_in, 0_in, 180_deg);
  Ldoinker.set_value(1);
  pros::delay(200);
  chassis.pid_drive_set(-5, 60, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-96,80);
  chassis.pid_wait();
  chassis.pid_drive_set(-27, 60, true);
  chassis.pid_wait_quick_chain();
  Mogo_mech.set_value(1);
  pros::delay(300);
  intake.move(127);
  chassis.pid_turn_set(135,80);
  pros::delay(1000);
  Ldoinker.set_value(0);
  chassis.pid_wait();
  chassis.pid_drive_set(18, 100, true);
  intaketop.move(0);
  chassis.pid_wait();
  Ldoinker.set_value(1);
  chassis.pid_drive_set(-15, 80, true);
  chassis.pid_wait_quick();
  intake.move(127);
  chassis.pid_turn_set(60,80);
  chassis.pid_wait_quick_chain();
  Ldoinker.set_value(0);
  chassis.pid_turn_set(50,80);
  chassis.pid_wait();
  chassis.pid_drive_set(12, 100, true);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::RIGHT_SWING,0,90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20, 100, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-115,80);
  chassis.pid_wait();
  // chassis.pid_swing_set(ez::RIGHT_SWING,-90,90);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_swing_set(ez::RIGHT_SWING,180,90);
  // chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(15, 100, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-160,80);
  chassis.pid_wait();
  chassis.pid_drive_set(80, 100, true);
  pros::delay(1000);
  intakebottom.move(0);
  chassis.pid_wait();
  chassis.pid_turn_set(45,80);
  chassis.pid_wait();
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(174,80);
  // chassis.pid_wait();
  // scoop.set_value(0);
  // chassis.pid_drive_set(80, 100, true);
  // pros::delay(600);
  // ring_rush.set_value(1);
  // pros::delay(400);
  // ring_rush.set_value(0);
  // chassis.pid_wait();
  }

  void Red4Ring() {
     
  team_color = red;

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  Rdoinker.set_value(1);
  pros::delay(200);
  chassis.pid_swing_set(ez::LEFT_SWING,-82_deg,80);
  chassis.pid_wait_quick_chain();
  pros::delay(400);
  chassis.pid_drive_set(-21, 50, true);
  chassis.pid_wait_quick_chain();
  pros::delay(100);
  Mogo_mech.set_value(1);
  intake.move(127);
  pros::delay(200);
  chassis.pid_turn_set(20,80);
  chassis.pid_wait_quick_chain();
  Rdoinker.set_value(0);
  pros::delay(100);
  chassis.pid_turn_set(43,80);
  chassis.pid_wait_quick_chain();
  pros::delay(200);
  intaketop.move(0);
  chassis.pid_drive_set(18, 100, true);
  pros::delay(200);
  chassis.pid_wait_quick_chain();
  Rdoinker.set_value(1);
  intakebottom.move(0);
  pros::delay(200);
  chassis.pid_drive_set(-15, 100, true);
  chassis.pid_wait_quick_chain();
  intake.move(127);
  chassis.pid_turn_set(160,70);
  chassis.pid_wait_quick_chain();
  Rdoinker.set_value(0);
  pros::delay(200);
  chassis.pid_turn_set(180,70);
  chassis.pid_wait_quick_chain();
  pros::delay(200);
  chassis.pid_drive_set(15, 100, true);
  chassis.pid_wait_quick_chain();
  Rdoinker.set_value(1);
  chassis.pid_turn_set(250,70);
  chassis.pid_wait_quick_chain();
  Rdoinker.set_value(0);
  chassis.pid_drive_set(40, 80, true);
  Ldoinker.set_value(1);
  pros::delay(2000);
  chassis.pid_turn_set(45,127);
  chassis.pid_wait_quick_chain();
  Ldoinker.set_value(0);
  chassis.pid_turn_set(40,127);
  pros::delay(200);
  chassis.pid_drive_set(15, 100, true);
  chassis.pid_wait_quick_chain();

}

void Red4RingWP() {
  
  team_color = red;

  chassis.odom_xyt_set(0_in, 0_in, 293_deg);
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  pros::delay(200);
  nextstate();
  nextstate();
  chassis.pid_wait();
  pros::delay(100);
  chassis.pid_drive_set(-22, 100, true);
  pros::delay(700);
  chassis.pid_drive_set(-15, 50, true);
  chassis.pid_wait();
  pros::delay(100);
  Mogo_mech.set_value(1);
  nextstate();
  pros::delay(100);
  chassis.pid_turn_set(48,80);
  pros::delay(600);
  chassis.pid_drive_set(18, 100, true);
  pros::delay(800);
  scoop.set_value(1);
  pros::delay(200);
  chassis.pid_drive_set(-40, 100, true);
  pros::delay(200);
  intake.move(127);
  chassis.pid_wait();
  scoop.set_value(0);
  pros::delay(200);
  chassis.pid_swing_set(ez::LEFT_SWING,165_deg,80);
  chassis.pid_wait();
  chassis.pid_drive_set(8, 100, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-125,80);
  pros::delay(400);
  scoop.set_value(1);
  chassis.pid_drive_set(36, 100, true);
  pros::delay(1700);
  chassis.pid_turn_set(90,80);
  pros::delay(1300);
  chassis.pid_drive_set(17, 100, true);
  pros::delay(1500);
  Mogo_mech.set_value(0);
  chassis.pid_drive_set(5, 100, true);
  pros::delay(400);
  chassis.pid_turn_set(25,80);
  scoop.set_value(0);
  pros::delay(500);
  nextstate();
  nextstate();
  chassis.pid_drive_set(35, 100, true);
}


void RedGoal(){
  team_color = red;
  
  chassis.odom_xyt_set(0_in, 0_in, 63_deg);
  Rdoinker.set_value(1);
  intakebottom.move(127);
  chassis.pid_drive_set(36_in, 127, true);
  pros::delay(800);
  intakebottom.move(15);
  pros::delay(500);
  chassis.pid_drive_set(-24_in, 127, true);
  pros::delay(800);
  pros::delay(300);
  chassis.pid_turn_set(192,80);
  pros::delay(150);
  Rdoinker.set_value(0);
  chassis.pid_wait();
  chassis.pid_drive_set(-21_in, 65, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7_in, 40, true);
  chassis.pid_wait_quick_chain();
  Mogo_mech.set_value(1);
  pros::delay(200);
  intake.move(127);
  pros::delay(1300);
  Mogo_mech.set_value(0);
  intake.move(-127);
  chassis.pid_turn_set(-15,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-20_in, 55, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-5_in, 40, true);
  chassis.pid_wait_quick_chain();
  Mogo_mech.set_value(1);
  pros::delay(200);
  chassis.pid_turn_set(-115,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(40_in, 90, true);
  pros::delay(1600);
  intake.move(127);
  pros::delay(200);
  chassis.pid_drive_set(-17_in, 90, true);
  chassis.pid_wait_quick_chain();
  pros::delay(200);
  chassis.pid_drive_set(17_in, 90, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-17_in, 90, true);
  chassis.pid_wait_quick_chain();
}


void RedRush() {

team_color = red;

chassis.odom_xyt_set(0_in, 0_in, 75_deg);
Ldoinker.set_value(1);
chassis.pid_drive_set(36_in, 127, true);
intakebottom.move(127);
chassis.pid_wait_quick_chain();
chassis.pid_wait();
intaketop.move(127);
pros::delay(300);
intaketop.move(0);
chassis.pid_turn_set(45,90);
chassis.pid_wait();
chassis.pid_drive_set(-23_in, 50, true);
chassis.pid_wait_quick_chain();
Mogo_mech.set_value(1);
Ldoinker.set_value(0);
pros::delay(300);
chassis.pid_turn_set(11,90);
chassis.pid_wait();
intake.move(127);
chassis.pid_drive_set(25, 50, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(-65,127);
chassis.pid_wait();
chassis.pid_drive_set(45, 80, true);
pros::delay(500);
Rdoinker.set_value(1);
pros::delay(1400);
chassis.pid_turn_set(-90,127);
chassis.pid_wait();
chassis.pid_drive_set(-5, 50, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(180,127);
chassis.pid_wait();
Rdoinker.set_value(0);
chassis.pid_turn_set(194,127);
chassis.pid_wait();
pros::delay(200);
chassis.pid_drive_set(110, 80, true);



}

void BlueWP() {

  team_color =  blue;

  chassis.odom_theta_flip();
  chassis.odom_xyt_set(0_in, 0_in, 170_deg);
  nextstate();
  nextstate();
  pros::delay(600);
  chassis.pid_drive_set(-2, 100, true);
  nextstate();
  nextstate();
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-95,80);
  chassis.pid_wait();
  chassis.pid_drive_set(-21, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-13, 50, true);
  chassis.pid_wait_quick_chain();
  Mogo_mech.set_value(1);
  chassis.pid_wait_quick_chain();
  pros::delay(100);
  intake.move(127);
  chassis.pid_turn_set(30,80);
  chassis.pid_wait();
  chassis.pid_drive_set(17, 100, true);
  chassis.pid_wait();
  chassis.pid_turn_set(0,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10, 80, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(20,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-23, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(24, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-78,127);
  chassis.pid_wait();
  chassis.pid_drive_set(45, 70, true);
  Ldoinker.set_value(1);
  pros::delay(2500);
  chassis.pid_turn_set(135,127);
  chassis.pid_wait();
  chassis.pid_drive_set(50, 127, true);
  pros::delay(600);
  Ldoinker.set_value(0);
  pros::delay(900);
  nextstate();
  nextstate();

  // chassis.odom_xyt_set(0_in, 0_in, 114_deg);
  // chassis.pid_drive_set(8_in, 70, true);
  // pros::delay(200);
  // nextstate();
  // nextstate();
  // nextstate();
  // chassis.pid_wait();
  // pros::delay(200);
  // chassis.pid_drive_set(-21, 100, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-16, 50, true);
  // chassis.pid_wait_quick_chain();
  // pros::delay(100);
  // Mogo_mech.set_value(1);
  // nextstate();
  // chassis.pid_wait_quick_chain();
  // pros::delay(100);
  // intake.move(127);
  // chassis.pid_turn_set(35,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(6, 100, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(3,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(8, 80, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(25,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-23, 100, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(0,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(24, 100, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(-73,127);
  // chassis.pid_wait();
  // chassis.pid_drive_set(45, 70, true);
  // pros::delay(900);
  // nextstate();
  // nextstate();
  // pros::delay(1000);
  // chassis.pid_turn_set(135,127);
  // chassis.pid_wait();
  // chassis.pid_drive_set(58, 100, true);

}




void BlueRush() {

team_color = blue;

chassis.odom_theta_flip();
chassis.odom_xyt_set(0_in, 0_in, 285_deg);
Rdoinker.set_value(1);
chassis.pid_drive_set(36_in, 127, true);
intakebottom.move(127);
chassis.pid_wait_quick_chain();
chassis.pid_wait();
intaketop.move(127);
pros::delay(300);
intaketop.move(0);
chassis.pid_turn_set(40,90);
chassis.pid_wait();
chassis.pid_drive_set(-23_in, 50, true);
chassis.pid_wait_quick_chain();
Mogo_mech.set_value(1);
pros::delay(300);
chassis.pid_turn_set(20,90);
chassis.pid_wait();
Rdoinker.set_value(0);
chassis.pid_turn_set(10,90);
chassis.pid_wait();
intake.move(127);
chassis.pid_drive_set(25, 50, true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(-55,127);
chassis.pid_wait();
chassis.pid_drive_set(45, 80, true);
pros::delay(1000);
nextstate();
nextstate();
pros::delay(1400);
chassis.pid_turn_set(178,127);
chassis.pid_wait();
Rdoinker.set_value(0);
chassis.pid_drive_set(60, 100, true);
pros::delay(3000);
intake.move(0);


}


void BlueSigWP() {
  
  team_color = blue;

  chassis.odom_theta_flip();
  chassis.odom_xyt_set(0_in, 0_in, 170_deg);
  nextstate();
  nextstate();
  pros::delay(600);
  chassis.pid_drive_set(-2, 100, true);
  nextstate();
  nextstate();
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-100,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-21, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-13, 50, true);
  chassis.pid_wait_quick_chain();
  Mogo_mech.set_value(1);
  chassis.pid_wait_quick_chain();
  pros::delay(100);
  intake.move(127);
  chassis.pid_turn_set(30,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(17, 100, true);
  chassis.pid_wait();
  chassis.pid_turn_set(0,100);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(12, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(30,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-70, 127, true);
  pros::delay(900);
  Mogo_mech.set_value(0);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-60,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-30, 60, true);
  chassis.pid_wait_quick_chain();
  Mogo_mech.set_value(1);
  pros::delay(100);
  // nextstate();
  // nextstate();
  chassis.pid_turn_set(195,127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20, 110, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(205,127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-45, 127, true);
  pros::delay(400);
}


void Blue6Ring() {

  team_color =  blue;

  chassis.odom_theta_flip();
  chassis.odom_xyt_set(0_in, 0_in, 180_deg);
  Rdoinker.set_value(1);
  pros::delay(200);
  chassis.pid_drive_set(-5, 60, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-93,80);
  chassis.pid_wait();
  chassis.pid_drive_set(-27, 60, true);
  chassis.pid_wait_quick_chain();
  Mogo_mech.set_value(1);
  pros::delay(300);
  intake.move(127);
  chassis.pid_turn_set(135,80);
  pros::delay(1000);
  Rdoinker.set_value(0);
  chassis.pid_wait();
  chassis.pid_drive_set(18, 100, true);
  intaketop.move(0);
  chassis.pid_wait();
  Rdoinker.set_value(1);
  chassis.pid_drive_set(-15, 80, true);
  chassis.pid_wait_quick();
  intake.move(127);
  chassis.pid_turn_set(60,80);
  chassis.pid_wait_quick_chain();
  Rdoinker.set_value(0);
  chassis.pid_turn_set(50,80);
  chassis.pid_wait();
  chassis.pid_drive_set(12, 100, true);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::RIGHT_SWING,0,90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20, 100, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-115,80);
  chassis.pid_wait();
  chassis.pid_drive_set(15, 100, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-160,80);
  chassis.pid_wait();
  chassis.pid_drive_set(80, 100, true);
  pros::delay(1000);
  intakebottom.move(0);
  chassis.pid_wait();
  chassis.pid_turn_set(45,80);
  chassis.pid_wait();
}

void Blue4Ring() {

  team_color = blue;

  chassis.odom_theta_flip();

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  Ldoinker.set_value(1);
  pros::delay(200);
  chassis.pid_swing_set(ez::LEFT_SWING,-83_deg,80);
  chassis.pid_wait_quick_chain();
  pros::delay(400);
  chassis.pid_drive_set(-21, 50, true);
  chassis.pid_wait_quick_chain();
  pros::delay(100);
  Mogo_mech.set_value(1);
  intake.move(127);
  pros::delay(200);
  chassis.pid_turn_set(20,80);
  chassis.pid_wait_quick_chain();
  Ldoinker.set_value(0);
  pros::delay(300);
  chassis.pid_turn_set(40,80);
  chassis.pid_wait_quick_chain();
  pros::delay(200);
  intaketop.move(0);
  chassis.pid_drive_set(21, 100, true);
  pros::delay(200);
  intakebottom.move(0);
  chassis.pid_wait_quick_chain();
  Ldoinker.set_value(1);
  pros::delay(200);
  chassis.pid_drive_set(-13, 100, true);
  chassis.pid_wait_quick_chain();
  intake.move(127);
  chassis.pid_turn_set(160,70);
  chassis.pid_wait_quick_chain();
  Ldoinker.set_value(0);
  pros::delay(200);
  chassis.pid_turn_set(180,70);
  chassis.pid_wait_quick_chain();
  pros::delay(200);
  chassis.pid_drive_set(15, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(250,70);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(45, 100, true);
  pros::delay(2300);
  // chassis.pid_drive_set(-25, 127, true);
  // chassis.pid_wait_quick_chain();
  Rdoinker.set_value(1);
  // chassis.pid_drive_set(25, 100, true);
  // chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(45,70);
  chassis.pid_wait_quick_chain();
  Mogo_mech.set_value(0);
  pros::delay(200);
  chassis.pid_drive_set(5, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-90,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-20, 100, true);
  chassis.pid_wait_quick_chain();
  }

void Blue3Ring() {
  
  team_color =  blue;

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_turn_set(90,80);
  chassis.pid_wait();
  chassis.pid_turn_set(0,80);
  chassis.pid_wait();
  chassis.pid_drive_set(24_in, 65, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90,80);
  chassis.pid_wait();
  chassis.pid_drive_set(10_in, 65, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-10_in, 65, true);
  chassis.pid_wait();
  chassis.pid_turn_set(0,80);
  chassis.pid_wait();
  chassis.pid_drive_set(24_in, 65, true);
  chassis.pid_wait();
  chassis.pid_turn_set(45,80);
  chassis.pid_wait();
  chassis.pid_drive_set(24_in, 65, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-24_in, 65, true);
  chassis.pid_wait();
  chassis.pid_turn_set(0,80);
  chassis.pid_wait();
  chassis.pid_drive_set(-486_in, 65, true);
  chassis.pid_wait();

  // chassis.pid_wait();
  // chassis.pid_turn_set(180,80);
  // chassis.pid_wait();
  // chassis.pid_turn_set(0,80);
  // chassis.pid_wait();
  // chassis.pid_turn_set(270,80);
  // chassis.pid_wait();
  // chassis.pid_turn_set(0,80);
  // chassis.pid_wait_quick_chain();

}

void BlueGoal(){
  team_color = blue;
  
  chassis.odom_theta_flip();
  chassis.odom_xyt_set(0_in, 0_in, 297_deg);
  Ldoinker.set_value(1);
  intakebottom.move(127);
  chassis.pid_drive_set(36_in, 127, true);
  pros::delay(800);
  intakebottom.move(15);
  pros::delay(500);
  chassis.pid_drive_set(-24_in, 127, true);
  pros::delay(800);
  pros::delay(300);
  chassis.pid_turn_set(192,80);
  pros::delay(1);
  Ldoinker.set_value(0);
  chassis.pid_wait();
  chassis.pid_drive_set(-21_in, 65, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7_in, 40, true);
  chassis.pid_wait_quick_chain();
  Mogo_mech.set_value(1);
  pros::delay(200);
  intake.move(127);
  pros::delay(1300);
  Mogo_mech.set_value(0);
  intake.move(-127);
  chassis.pid_turn_set(-15,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-20_in, 55, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-8_in, 40, true);
  chassis.pid_wait_quick_chain();
  Mogo_mech.set_value(1);
  pros::delay(200);
  intake.move(-127);
  chassis.pid_turn_set(-115,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(50_in, 90, true);
  pros::delay(1600);
  Rdoinker.set_value(1);
  chassis.pid_turn_set(45,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(35_in, 90, true);
  pros::delay(1600);






}

// . . .
// Make your own autonomous functions here!
// . . .

void Skills() {
  default_constants();
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  nextstate();
  nextstate();
  nextstate();
  pros::delay(800);
  chassis.pid_drive_set(-11,50);
  chassis.pid_wait();
  chassis.pid_turn_set(180,80);
  chassis.pid_wait();
  chassis.pid_drive_set(-20,50);
  chassis.pid_wait();
  Mogo_mech.set_value(1);
  nextstate();
  pros::delay(200);
  chassis.pid_turn_set(90,80);
  chassis.pid_wait_quick_chain();
  intake.move(127);
  chassis.pid_drive_set(20,100);
  chassis.pid_wait();
  chassis.pid_turn_set(45,80);
  chassis.pid_wait();
  chassis.pid_drive_set(40,100);
  pros::delay(1000);
  nextstate();
  chassis.pid_wait();
  // chassis.pid_turn_set(-100,80);
  // chassis.pid_wait();
  // chassis.pid_drive_set(30,100);
  // chassis.pid_wait();
  chassis.pid_turn_set(0,80);
  chassis.pid_wait();
  chassis.pid_drive_set(20,80);
  intake.move(0);
  pros::delay(100);
  intake.move(127);
  pros::delay(100);
  intake.move(0);
  pros::delay(100);
  intake.move(127);
  pros::delay(100);
  chassis.pid_wait();
  intaketop.move(-15);
  nextstate();
  pros::delay(1200);
  chassis.pid_drive_set(-15,80);
  nextstate();
  nextstate();
  chassis.pid_wait();
  chassis.pid_turn_set(-85,80);
  intake.move(127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(30,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(30,40);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(45,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(15,50);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(110,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-15,50);
  chassis.pid_wait_quick_chain();
  pros::delay(500);
  intake.move(-127);
  Mogo_mech.set_value(0);
  chassis.pid_drive_set(10,50);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(30,100);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-30,100);
  pros::delay(1000);
  chassis.odom_xyt_set(0_in, 0_in, 90_deg);
  chassis.pid_drive_set(15,100);
  chassis.pid_wait();
  chassis.pid_turn_set(0,80);
  chassis.pid_wait();
  chassis.pid_drive_set(-25,60);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-12,40);
  chassis.pid_wait_quick_chain();
  Mogo_mech.set_value(1);
  pros::delay(200);
  chassis.pid_turn_set(90,80);
  chassis.pid_wait_quick_chain();
  intake.move(127);
  chassis.pid_drive_set(20,100);
  chassis.pid_wait();
  chassis.pid_turn_set(135,80);
  chassis.pid_wait();
  chassis.pid_drive_set(40,100);
  pros::delay(1000);
  nextstate();
  chassis.pid_wait();
  chassis.pid_turn_set(180,80);
  chassis.pid_wait();
  chassis.pid_drive_set(20,80);
  intake.move(0);
  pros::delay(100);
  intake.move(127);
  pros::delay(100);
  intake.move(0);
  pros::delay(100);
  intake.move(127);
  pros::delay(100);
  chassis.pid_wait();
  intaketop.move(-15);
  nextstate();
  pros::delay(1200);
  chassis.pid_drive_set(-15,80);
  nextstate();
  nextstate();
  chassis.pid_wait();
  chassis.pid_turn_set(-90,80);
  intake.move(127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(30,100);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(30,50);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(135,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(15,50);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(70,80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-15,50);
  chassis.pid_wait_quick_chain();
  pros::delay(500);
  intake.move(-127);
  Mogo_mech.set_value(0);

  


  
























  // chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  // nextstate();
  // nextstate();
  // nextstate();
  // pros::delay(700);
  // chassis.pid_drive_set(-12, 100, true);
  // pros::delay(600);
  // chassis.pid_turn_set(180,80);
  // pros::delay(400);
  // chassis.pid_drive_set(-23, 55, true);
  // pros::delay(800);
  // Mogo_mech.set_value(1);
  // nextstate();
  // pros::delay(400);
  // chassis.pid_turn_set(105,80);
  // chassis.pid_wait_quick_chain();
  // intake.move(127);
  // chassis.pid_drive_set(20, 110, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(25,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(20, 110, true);
  // chassis.pid_wait_quick_chain();
  // intaketop.move(127);
  // chassis.pid_turn_set(90,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(12, 110, true);
  // nextstate();
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-18, 110, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(0,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(15, 60, true);
  // chassis.pid_wait_quick_chain();
  // intake.move(-127);
  // nextstate();
  // pros::delay(500);
  // intake.move(127);
  // pros::delay(1000);
  // chassis.pid_drive_set(-18, 110, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(-90,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(50, 60, true);
  // chassis.pid_wait_quick_chain();


  // chassis.pid_turn_set(105,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-15, 110, true);
  // chassis.pid_wait_quick_chain();
  // intake.move(-127);
  // pros::delay(200);
  // Mogo_mech.set_value(0);
  // pros::delay(400);
  // chassis.pid_drive_set(3, 110, true);
  // intake.move(127);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(0,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-25, 127, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(90,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-35, 80, true);
  // pros::delay(1500);
  // chassis.odom_xyt_set(0_in, 0_in, 90_deg);
  // chassis.pid_drive_set(15.5, 110, true);
  // pros::delay(700);
  // chassis.pid_turn_set(0,80);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-39, 45, true);
  // chassis.pid_wait();
  // Mogo_mech.set_value(1);
  // pros::delay(200);
  // chassis.pid_turn_set(90,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(20, 110, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(155,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(38, 110, true);
  // chassis.pid_wait_quick_chain();
  // pros::delay(400);
  // chassis.pid_turn_set(-75,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(24, 110, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(-90,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(50, 40, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-24, 110, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(-125,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(15, 110, true);
  // chassis.pid_wait_quick_chain();
  // pros::delay(400);
  // chassis.pid_turn_set(65,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-20, 110, true);
  // chassis.pid_wait();
  // intake.move(-127);
  // pros::delay(200);
  // Mogo_mech.set_value(0);
  // pros::delay(500);
  // chassis.pid_turn_set(80,80);
  // intake.move(-127);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(65, 110, true);
  // intaketop.move(0);
  // intakebottom.move(127);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(0,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(23, 60, true);
  // pros::delay(500);
  // intaketop.move(127);
  // pros::delay(200);
  // intaketop.move(0);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(-142,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-28, 50, true);
  // pros::delay(1300);
  // Mogo_mech.set_value(1);
  // chassis.pid_wait_quick_chain();
  // pros::delay(300);
  // intake.move(127);
  // chassis.pid_turn_set(-50,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(30, 70, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(0,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(30, 80, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(-115,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-32, 127, true);
  // Mogo_mech.set_value(0);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(15, 127, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(155,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(150, 127, true);
  // pros::delay(3500); 
  // intake.move(-127);
  // chassis.pid_turn_set(130,80);
  // chassis.pid_wait_quick_chain();
  // nextstate();
  // nextstate();
  // nextstate();
  // chassis.pid_drive_set(-35, 127, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-30, 75, true);
  // chassis.pid_wait_quick_chain();
  // pros::delay(600);
  // nextstate();
  // chassis.pid_drive_set(2, 40, true);
  // chassis.pid_wait_quick_chain();






























  // chassis.pid_drive_chain_constant_set(3_in);
  // chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  // nextstate();
  // nextstate();
  // nextstate();
  // pros::delay(700);
  // chassis.pid_drive_set(-12, 100, true);
  // pros::delay(600);
  // chassis.pid_turn_set(180,80);
  // pros::delay(400);
  // chassis.pid_drive_set(-23, 55, true);
  // pros::delay(800);
  // Mogo_mech.set_value(1);
  // nextstate();
  // pros::delay(400);
  // chassis.pid_turn_set(105,80);
  // chassis.pid_wait_quick_chain();
  // intake.move(127);
  // chassis.pid_drive_set(20, 110, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(25,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(35, 110, true);
  // pros::delay(800);
  // intaketop.move(0);
  // chassis.pid_wait_quick_chain();
  // pros::delay(400);
  // intaketop.move(127);
  // chassis.pid_turn_set(-105,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(18, 110, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(-90,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(45, 60, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-24, 110, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(-55,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(15, 110, true);
  // chassis.pid_wait_quick_chain();
  // pros::delay(400);
  // chassis.pid_turn_set(105,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-15, 110, true);
  // chassis.pid_wait_quick_chain();
  // intake.move(-127);
  // pros::delay(200);
  // Mogo_mech.set_value(0);
  // pros::delay(400);
  // chassis.pid_drive_set(3, 110, true);
  // intake.move(127);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(0,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-25, 127, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(90,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-35, 80, true);
  // pros::delay(1500);
  // chassis.odom_xyt_set(0_in, 0_in, 90_deg);
  // chassis.pid_drive_set(15.5, 110, true);
  // pros::delay(700);
  // chassis.pid_turn_set(0,80);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-39, 45, true);
  // chassis.pid_wait();
  // Mogo_mech.set_value(1);
  // pros::delay(200);
  // chassis.pid_turn_set(90,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(20, 110, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(155,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(38, 110, true);
  // chassis.pid_wait_quick_chain();
  // pros::delay(400);
  // chassis.pid_turn_set(-75,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(24, 110, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(-90,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(50, 40, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-24, 110, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(-125,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(15, 110, true);
  // chassis.pid_wait_quick_chain();
  // pros::delay(400);
  // chassis.pid_turn_set(65,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-20, 110, true);
  // chassis.pid_wait();
  // intake.move(-127);
  // pros::delay(200);
  // Mogo_mech.set_value(0);
  // pros::delay(500);
  // chassis.pid_turn_set(80,80);
  // intake.move(-127);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(65, 110, true);
  // intaketop.move(0);
  // intakebottom.move(127);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(0,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(23, 60, true);
  // pros::delay(500);
  // intaketop.move(127);
  // pros::delay(200);
  // intaketop.move(0);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(-142,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-28, 50, true);
  // pros::delay(1300);
  // Mogo_mech.set_value(1);
  // chassis.pid_wait_quick_chain();
  // pros::delay(300);
  // intake.move(127);
  // chassis.pid_turn_set(-50,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(30, 70, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(0,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(30, 80, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(-115,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-32, 127, true);
  // Mogo_mech.set_value(0);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(15, 127, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(155,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(150, 127, true);
  // pros::delay(3500); 
  // intake.move(-127);
  // chassis.pid_turn_set(130,80);
  // chassis.pid_wait_quick_chain();
  // nextstate();
  // nextstate();
  // nextstate();
  // chassis.pid_drive_set(-35, 127, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-30, 75, true);
  // chassis.pid_wait_quick_chain();
  // pros::delay(600);
  // nextstate();
  // chassis.pid_drive_set(2, 40, true);
  // chassis.pid_wait_quick_chain();
  

  // chassis.pid_drive_set(90, 100, true);
  // intaketop.move(0);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(-148,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-17, 60, true);
  // chassis.pid_wait_quick_chain();
  // Mogo_mech.set_value(1);
  // pros::delay(300);
  // intake.move(127);
  // chassis.pid_turn_set(180,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(60, 70, true);
  // chassis.pid_wait_quick_chain();
  // scoop.set_value(1);
  // chassis.pid_turn_set(125,80);
  // chassis.pid_wait_quick_chain();
  // intakebottom.move(0);
  // chassis.pid_drive_set(20, 80, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(75,80);
  // pros::delay(400);
  // chassis.pid_turn_set(-45,80);
  // chassis.pid_wait_quick_chain();
  // Mogo_mech.set_value(0);
  // scoop.set_value(0);
  // intake.move(-127);
  // chassis.pid_drive_set(-10, 127, true);
  // chassis.pid_wait_quick_chain();
  // intake.move(127);
  // chassis.pid_drive_set(20, 127, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(25,80);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(150, 127, true);
  // pros::delay(3500); 
  // intake.move(-127);
  // chassis.pid_drive_set(-10, 127, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(50,80);
  // chassis.pid_wait_quick_chain();
  // nextstate();
  // nextstate();
  // chassis.pid_drive_set(-30, 127, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-15, 80, true);
  // chassis.pid_wait_quick_chain();
  // pros::delay(600);
  // nextstate();
  // chassis.pid_drive_set(10, 50, true);
  // chassis.pid_wait_quick_chain();

  





  


}

void skills2() {
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  // nextstate();
  // nextstate();
  // chassis.pid_wait();
  // pros::delay(300);
  chassis.pid_odom_set({{0_in,5_in, 0_deg}, fwd, 60},
                       true);
  chassis.pid_wait();
  // Mogo_mech.set_value(1);
  // nextstate();
  // pros::delay(100);
  // intake.move(127);
  // chassis.pid_odom_set({{-23_in, 23_in, 47_deg}, fwd, 115},
  //                      true);
  // chassis.pid_wait();
  // chassis.pid_odom_set({{0_in, 59_in, 0_deg}, fwd, 115},
  //                      true);
  // chassis.pid_wait();
  // chassis.pid_odom_set({{23_in, 47_in, 256_deg}, fwd, 115},
  //                      true);
  // chassis.pid_wait();
  // chassis.pid_odom_set({{66_in, 47_in, 270_deg}, fwd, 60},
  //                      true);
  // chassis.pid_wait();
  // chassis.pid_odom_set({{-41_in, 47_in, 333_deg}, rev, 100},
  //                      true);
  // chassis.pid_wait();
  // chassis.pid_odom_set({{-48_in, 62_in, 333_deg}, fwd, 100},
  //                      true);
  // chassis.pid_wait();
  // chassis.pid_odom_set({{-65_in, -65_in, 138_deg}, rev, 85},
  //                      true);
  // chassis.pid_wait();
  // Mogo_mech.set_value(0);


}