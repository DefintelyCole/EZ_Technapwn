#include "main.h"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////





// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-21, -7, -6},     // Left Chassis Ports (negative port will reverse it!)
    {3, 4, 5},  // Right Chassis Ports (negative port will reverse it!)

    12,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
ez::tracking_wheel horiz_tracker(8, 2, 1.625);  // This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel vert_tracker(19, 2, 1.25);   // This tracking wheel is parallel to the drive wheels

const int numStates = 3;
int states[numStates] = {0, -2000, -14000};
int currState = 0;
int target = 0;


void nextstate() {
    currState += 1;
    if (currState == numStates) {
        currState = 0;
    }
    target = states[currState];
}






void liftControl() {
    double kp = 0.01;
    double error = target - lb.get_position();
    double velocity = kp * error;
    Lift.move(velocity);
}


void colorsort() {
   
  if (team_color == red){
    if(color.get_hue() > 185){
      pros::delay(1);
      eject.set_value(1);
      pros::delay(300);
      eject.set_value(0);
    }
  }
  else if (team_color == blue) {
    if(color.get_hue() < 40){
      pros::delay(1);
      eject.set_value(1);
      pros::delay(300);
      eject.set_value(0);
    }
   
  }
  else{
    eject.set_value(0);
  }


}







void initialize() {

  lb.set_position(0);
  color.set_led_pwm(100);
  Lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  ez::ez_template_print();
  pros::delay(500); 
 
 
 
 


  pros::Task liftControlTask([]{
        while (true) {
            liftControl();
            pros::delay(10);
        }
    });


    pros::Task ColorSortTask([]{
        while (true) {
            colorsort();
            pros::delay(10);
        }
    });
   

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"RedWP\n\nscores ring on alliance stake then grabs goal and puts 3 rings on", RedWP},
      {"Red3Ring\n\nputs a rng on alliance stake then grabs red ring ontop of double stack grabs goal and scores 2 rings on it", Red3Ring},
      {"BlueWP\n\nMirror of RedWP", BlueWP},
      {"Blue3Ring\n\nMirror of Red3Ring", Blue3Ring},
      {"Skills\n\nscores 6 rings on 2 goals in corner and pushes 2 other goals in to corners", Skills},
      {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      {"Combine all 3 movements", combining_movements},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
      {"RedWP\n\nscores ring on alliance stake then grabs goal and puts 3 rings on", RedWP},
      
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    // chassis.opcontrol_tank();  // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .
    Lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


          // contols for intake
         if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move(127);
         } else if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
           intake.move(-127);
         } else {
           intake.move(0);
         }




          // controls for mogo mech
          if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
          Mogo_mech.set_value(true);
         } else if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
          Mogo_mech.set_value(false);
         }




          // controls for release on claw
         if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
          scoop.set_value(true);
         } else {
          scoop.set_value(false);
         }


 


          


         if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
              team_color = red;
         
         } else if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
              team_color = blue;
             
         }


         
     if (Master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
      nextstate();
    }
     pros::delay(20);


   

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}