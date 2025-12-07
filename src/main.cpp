#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-6, -5, -8},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({7, 9, 20}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(14);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(4.5, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            4, // derivative gain (kD)
                                            0, // anti windup
                                            1, // small error range, in inches
                                            75, // small error range timeout, in milliseconds
                                            2, // large error range, in inches
                                            150, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(4, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             20, // derivative gain (kD)
                                             0, // anti windup
                                             1, // small error range, in degrees
                                             50, // small error range timeout, in milliseconds
                                             2, // large error range, in degrees
                                             200, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

pros::Motor Intake(19);
pros::Motor Scoring(-3);

pros::ADIDigitalOut descore('F');
pros::ADIDigitalOut matchload('G');
pros::ADIDigitalOut middle('H');

bool descorestate = false;
bool matchloadstate = false;
bool middlestate = false;

void toggle_descore(){
    descorestate = !descorestate;
    descore.set_value(descorestate);
}

void toggle_matchload(){
    matchloadstate = !matchloadstate;
    matchload.set_value(matchloadstate);
}

void toggle_middle(){
    middlestate = !middlestate;
    middle.set_value(middlestate);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */


void redRight() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 14, 1000, {.minSpeed = 1, .earlyExitRange = 1});
    chassis.swingToHeading(25,lemlib::DriveSide::RIGHT, 1000, {.minSpeed = 1, .earlyExitRange = 1});
    Intake.move_velocity(200);
    // Scoring.move_velocity(-50);
    chassis.moveToPoint(8, 27, 1500, {.maxSpeed = 50});
    chassis.waitUntil(7);
    toggle_matchload();
    chassis.turnToHeading(-47, 1000);
    toggle_matchload();
    chassis.moveToPoint(-4, 33.9, 1000, {.maxSpeed = 70});
    chassis.waitUntil(7);
    Intake.move_velocity(-200);
    Scoring.move_velocity(-50);
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.moveToPoint(1,27,1500, {.forwards = false});
    chassis.waitUntil(5);
    Intake.move_velocity(200);
    chassis.turnToHeading(190, 1000, {.minSpeed = 1, .earlyExitRange = 5});
    chassis.moveToPoint(4.5, 0, 1000, {.minSpeed = 1, .earlyExitRange = 1});
    chassis.turnToHeading(-270, 1000, {.minSpeed = 1, .earlyExitRange = 1});
    chassis.moveToPoint(30.5, 3, 1500);
    chassis.turnToHeading(-180, 1000);
    toggle_matchload();
    chassis.moveToPoint(30.5, -6, 1000, {.minSpeed = 70});
    chassis.waitUntilDone();
    chassis.moveToPoint(30.5, -1, 1000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.moveToPoint(30.5, -4, 1000);
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.moveToPoint(30.5, -1, 1000, {.forwards = false, .maxSpeed = 70});

    chassis.moveToPoint(30.8, 20, 1500, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntil(15);
    Scoring.move_velocity(200);
}
void redLeft() {

}

void autonomous() {
    redRight();
    // redLeft();
}


/**
 * Runs in driver control
 */

void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);

        //buttons for controller

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
        {
			toggle_descore();

        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
			toggle_middle();
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        {
			toggle_matchload();
        }

        // Control Intake using shoulder buttons (L1/L2)
        
    

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
			Intake.move_velocity(200);
	
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
			Intake.move_velocity(-200);
			Scoring.move_velocity(-50);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
			Intake.move_velocity(200);
			Scoring.move_velocity(200);
        }
        else
        {
			Scoring.move_velocity(0);
			Intake.move_velocity(0);
        }




        // delay to save resources
        pros::delay(10);
    }
}
