#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.h"
#include "pros/motors.hpp"
#include <thread>
//DO CONFIG FUNTIME HERE

//LEFT MOTORS
constexpr int SP_DRIVE_L1 = -2;
constexpr int SP_DRIVE_L2 = -9;
constexpr int SP_DRIVE_L3 = 20; //Upside Down

//RIGHT MOTORS
constexpr int SP_DRIVE_R1 = 10;
constexpr int SP_DRIVE_R2 = 1;
constexpr int SP_DRIVE_R3 = -7; // Upside Down

constexpr int SP_INTAKE_LOWER = 19;
constexpr int SP_INTAKE_UPPER = 18;



ASSET(RightNOAWP1_txt);
ASSET(RightNOAWP2_txt);
ASSET(RightNOAWP3_txt);

//intake
pros::MotorGroup intake_group({SP_INTAKE_LOWER, SP_INTAKE_UPPER});
pros::Motor bottomIntake(SP_INTAKE_LOWER); //for individual control in auton or whatever
pros::Motor topIntake(SP_INTAKE_UPPER); 

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motor_group({SP_DRIVE_L1, SP_DRIVE_L2, SP_DRIVE_L3},
	pros::MotorGearset::blue
);
pros::MotorGroup right_motor_group({SP_DRIVE_R1, SP_DRIVE_R2, SP_DRIVE_R3},
	pros::MotorGearset::blue
);

pros::ADIDigitalOut scraperControl('A');

pros::ADIDigitalOut descoreControl('H');

lemlib::Drivetrain drivetrain(
	&left_motor_group, // left motor group
    &right_motor_group, // right motor group
    12.5, // 12.5 inch track width (distance from left to right wheels)
    lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
    450, // drivetrain rpm is 600 * 36/48
    2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(14);
// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(8);
// vertical tracking wheel encoder
// pros::adi::Encoder vertical_encoder('C', 'D', true);

pros::Rotation vertical_encoder(9);

// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -5.75);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -2.5);

// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(
	10, // proportional gain (kP)
    0, // integral gain (kI)
    3, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
	2, // proportional gain (kP)
    0, // integral gain (kI)
    10, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

void initialize() {
  scraperControl.set_value(false);
  descoreControl.set_value(false); 
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}



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
  // scraperControl.set_value(true);
  // descoreControl.set_value(false); 
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

// void antiJamBottomIntake() {
//   while (true) {
//     if (bottomIntake.get_actual_velocity() < 5 && bottomIntake.get_efficiency() > 0) {
//     bottomIntake.move(-127); 
//     } else {
//         bottomIntake.move(127);
//     }
//     std::this_thread::sleep_for(std::chrono::milliseconds(50)); // prevent CPU overload
//   }
  
// //   bottomIntake;
// }

void right_awp(){
  scraperControl.set_value(true);
  bottomIntake.move(127); 
    // std::thread intakeThread(antiJamBottomIntake);
  chassis.setPose(46.921, 11.123, 290);
  bottomIntake.move(127); 

    // chassis.turnToPoint(20.695, 21.922, 500);
    chassis.turnToPoint(20.695, 21.922, 3000);
    chassis.moveToPoint(20.695, 21.922, 3000);
bottomIntake.move(127); 
    // chassis.turnToPoint(51.076, 46.518, 500);
    chassis.turnToPoint(42.483, 46.635, 3000);
    chassis.moveToPoint(42.483, 46.635, 3000);
    scraperControl.set_value(false);
bottomIntake.move(127); 
    // chassis.turnToPoint(67.555, 46.798, 500);
    chassis.turnToPoint(57.54, 46.272, 3000);
    chassis.moveToPoint(57.54, 46.272, 3000);
    bottomIntake.move(127); 
    pros::delay(3000);

    // chassis.turnToPoint(23.348, 47.135, 500, {.forwards = false});
    // chassis.turnToPoint(30.329, 47.135, 3000, {.forwards = false});
    chassis.moveToPoint(30.329, 47.123, 3000, {.forwards = false});
    bottomIntake.move(127); 
    scraperControl.set_value(true);
    // intakeThread.detach();
    topIntake.move(127); 
}

void left_awp() {
  scraperControl.set_value(true);
  bottomIntake.move(127);
  // vex::task bottomIntakeTask = task(antiJamIntakeBottomIntake);
    // std::thread intakeThread(antiJamBottomIntake); 
    // bottomIntake.move(127);
    chassis.setPose(46.921, -11.123, 250);
    chassis.moveToPoint(20.695, -21.922, 5000);
    chassis.moveToPoint(51.076, -46.518, 5000);
    scraperControl.set_value(false);
    chassis.moveToPoint(67.555, -46.798, 5000);
    pros::delay(3000);
    
    chassis.moveToPoint(23.348, -47.135, 5000);
    scraperControl.set_value(true);
    topIntake.move(127);
    
}

/*chassis.moveTo(0, 0, 5000);
chassis.moveTo(1.178, 28.338, 5000);
chassis.moveTo(34.682, 8.201, 5000);
chassis.moveTo(40.58, -7.188, 5000);
chassis.moveTo(25.778, 34.468, 5000);
*/
void autonomous() {
  //   scraperControl.set_value(false);
  //   chassis.setPose(54.442,13.437,270);
  //   bottomIntake.move(127);
  //   chassis.follow(RightNOAWP1_txt, 15, 2000);
  //   // put down scraper
  //   scraperControl.set_value(true);
  //   chassis.follow(RightNOAWP2_txt, 15, 2000);
  //   scraperControl.set_value(false);
  //   //put up scraper
  //   chassis.follow(RightNOAWP3_txt, 15, 2000);
  //   topIntake.move(127);

    right_awp();
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
  // loop forever
  int intakeToggle = 1;
  bool scraperToggle = true;
  bool descoreToggle = false;
  bool toggleR2 = true;
  bool toggleX = true; 
  bool toggleL2 = true;
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // pros::lcd::print(3, "Yay: %f", controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1));
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
          bottomIntake.move(127);
          topIntake.move(intakeToggle*127);
        } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
          bottomIntake.move(-127);
          topIntake.move(intakeToggle * -127);
        } else {
          intake_group.brake();
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            if (toggleX) {
              toggleX = false;
              scraperToggle = !scraperToggle;
              scraperControl.set_value(scraperToggle); 
            }
        } else {
          toggleX = true;
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            if (toggleL2) {
              toggleL2 = false;
              intakeToggle = 0 - intakeToggle; 
            }
        } else {
          toggleL2 = true;
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            if (toggleR2) {
              toggleR2 = false;
              descoreToggle = !descoreToggle;
              descoreControl.set_value(descoreToggle); 
            }
        } else {
          toggleR2 = true; 
        }
        //set up macro later
        
        // move the robot
        chassis.arcade(leftY, rightX);

        // delay to save resources
        pros::delay(20);
    }
}
