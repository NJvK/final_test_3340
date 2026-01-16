#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <algorithm>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-11, 12, -13},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({-14, 15, 16}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(1);

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
lemlib::ControllerSettings linearController(5, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            6, // derivative gain (kD)
                                            0, // anti windup
                                            1, // small error range, in inches
                                            75, // small error range timeout, in milliseconds
                                            2, // large error range, in inches
                                            150, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(3.7, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             25.5, // derivative gain (kD)
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

pros::Motor intakeb(-19);
pros::Motor intakef(20);

pros::ADIDigitalOut descore('B');
pros::ADIDigitalOut matchload('A');

bool descore_state = false;
bool matchload_state = false;

void set_both(int32_t voltage) {
    intakef.move_voltage(voltage);
    intakeb.move_voltage(voltage);
}

void set_intakeb(int32_t voltage) { intakeb.move_voltage(voltage); }

void set_intakef(int32_t voltage) { intakef.move_voltage(voltage); }

void toggle_matchload() {
    matchload_state = !matchload_state;
    matchload.set_value(matchload_state);
}

void toggle_descore() {
    descore_state = !descore_state;
    descore.set_value(descore_state);
}


// direction: "x" or "y"
// distance: positive or negative inches
// timeout: ms
// settings: lemlib::MoveToPointSettings (same struct used in moveToPoint)


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

 void exit_condition(lemlib::Pose target, double exitDist){
    chassis.waitUntil(fabs(chassis.getPose().distance(target)) - exitDist);
    chassis.cancelMotion();
}

void redRight() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 14, 1000, {.minSpeed = 1, .earlyExitRange = 1});
    chassis.swingToHeading(25,lemlib::DriveSide::RIGHT, 1000, {.minSpeed = 1, .earlyExitRange = 1});
    Intake.move_velocity(200);
    // Scoring.move_velocity(-50);
    chassis.moveToPoint(8, 27, 1500, {.maxSpeed = 50});
    chassis.waitUntil(7);
    toggle_matchload();
    // toggle amtchlod

    chassis.moveToPoint(8, 24, 1000);
    set_intakef(12000);
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
    set_both(12000);
}

void left_quals() {
    chassis.moveToPoint(-5, 16, 1000, {.minSpeed = 20, .earlyExitRange = 3});
    set_intakef(12000);

    chassis.moveToPoint(-24, 33, 2000, {.maxSpeed = 70, .minSpeed = 10, .earlyExitRange = 2});
    chassis.waitUntil(21);
    toggle_matchload();

    chassis.moveToPoint(-7.3, 20, 1000, {.forwards = false, .maxSpeed = 90});
    chassis.turnToHeading(-130, 1000);

    chassis.moveToPoint(5, 33, 2500, {.forwards = false, .minSpeed = 90});
    chassis.waitUntil(10);
    set_both(-12000);
    pros::delay(200);
    set_intakef(12000);
    set_intakeb(0);
    pros::delay(1000);

    chassis.moveToPoint(-31, 1, 2000, {.minSpeed = 10, .earlyExitRange = 2});
    chassis.turnToHeading(-180, 1000, {.minSpeed = 10, .earlyExitRange = 5});

    chassis.moveToPoint(-30.5, -12, 1000);
    pros::delay(500);

    chassis.moveToPoint(-30.5, 16, 1000, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.moveToPoint(30.5, -1, 1000, {.forwards = false, .maxSpeed = 70});

    chassis.moveToPoint(30.8, 20, 1500, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntil(15);
    Scoring.move_velocity(200);
}
void redLeft() {
    // moveDistance(chassis, 10, 1000);

    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 13, 1000, {.minSpeed = 1, .earlyExitRange = 1});
    chassis.swingToHeading(-32,lemlib::DriveSide::LEFT, 1000, {.minSpeed = 1, .earlyExitRange = 1});
    Intake.move_velocity(200);
    // Scoring.move_velocity(-50);
    chassis.moveToPoint(-6, 25, 1500, {.maxSpeed = 50});
    chassis.waitUntil(7);
    toggle_matchload();
    chassis.turnToHeading(-135, 1000);
    toggle_matchload();
    Intake.move_velocity((0));
    toggle_middle();
    chassis.moveToPoint(6, 32, 1500, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntil(7);

    Intake.move_velocity(10000);
    Scoring.move_velocity(50);
    chassis.waitUntilDone();
    pros::delay(1000);
    // chassis.moveToPoint(-1,27,1500, {.forwards = false});
    // chassis.waitUntil(5);
    // Intake.move_velocity(200);
    // chassis.turnToHeading(-190, 1000, {.minSpeed = 1, .earlyExitRange = 5});
    chassis.moveToPoint(-28, 1, 2000);
    chassis.turnToHeading(-180, 1000);
    chassis.moveToPoint(-26.5, -9, 1500);
    // chassis.turnToHeading(180, 1000);
    toggle_matchload();
    chassis.waitUntilDone();
    pros::delay(800);
    chassis.moveToPoint(-26.5, -3, 1000, {.forwards = false, .maxSpeed = 20});
    chassis.moveToPoint(-26.5, 15, 1000, {.forwards = false});
    Scoring.move_velocity(200);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-30.5, -1, 1000, {.forwards = false, .maxSpeed = 70});
    // chassis.waitUntilDone();
    // pros::delay(200);
    // chassis.moveToPoint(-30.5, -4, 1000);
    // chassis.waitUntilDone();
    // pros::delay(200);
    // chassis.moveToPoint(-30.5, -1, 1000, {.forwards = false, .maxSpeed = 70});

    // chassis.moveToPoint(-30.8, 20, 1500, {.forwards = false, .maxSpeed = 90});
    // chassis.waitUntil(15);
    // Scoring.move_velocity(200);

}
void soloWinPointright() {
  chassis.setPose(-47.000, -8.000, 180.0);

  //moving to matchload
  chassis.moveToPoint(chassis.getPose().x, -43.00, 1000, {.maxSpeed =100, .minSpeed = 50});
  pros::delay(700);

  chassis.waitUntilDone();
  //turning to matchload
  chassis.turnToHeading(270, 650, {.minSpeed = 10, .earlyExitRange = 50});
  chassis.waitUntilDone();;
  //smashhhh
  chassis.moveToPoint(-70, -43.3, 900, { .maxSpeed = 90, .minSpeed = 60});
  chassis.waitUntilDone();
  pros::delay(300);

  //moving to score
  chassis.moveToPose(-19.000, -47.000, 270.0, 1000, {.forwards = false, .lead = 0.1,  .maxSpeed = 120, .minSpeed = 80});
  exitcondition({-23.000, -47.000, 270.0}, 1);
  chassis.turnToHeading(270, 200);
  pros::delay(1200);

  //6 balls route
  //getting 3
  chassis.moveToPose(-18.000, -19.000, 52.0, 1300, {.lead = 0.1, .maxSpeed = 90, .minSpeed = 50});
  chassis.waitUntil(24);
  exitcondition({-18.000, -19.000, 52.0}, 2);
  
  //getting other 3
  chassis.moveToPose(chassis.getPose().x, 28.500, 358.0, 1500, {.maxSpeed = 90, .minSpeed = 50});
  chassis.waitUntil(15);
  chassis.waitUntil(42);
  chassis.waitUntilDone();

  //midgoal
  chassis.turnToHeading(315, 600);
  chassis.moveToPose(-3.272, 2.344, 315.0, 1200, {.forwards = false,.lead = 0.1, .minSpeed = 85});
  pros::delay(200);
  chassis.waitUntilDone();
  chassis.turnToHeading(315, 200);
  pros::delay(600);

  //matchload
  chassis.moveToPoint(-43.000, 45.000, 1400, {.minSpeed = 80});
  pros::delay(200);
  chassis.moveToPose(-70.000, 45.000, 270.0, 1300, {.lead = 0.1,.maxSpeed = 80, .minSpeed = 50,});
  chassis.waitUntilDone();
  chassis.turnToHeading(270, 309);

  //scoring
  chassis.moveToPose(-19.000, 46.800, 270.0, 1000, {.forwards = false, .lead = 0.4, .maxSpeed = 120, .minSpeed = 70});
  exitcondition({-22.000, 47.800, 270.0}, 1);
  
}
void moveForward() {
    chassis.setPose(0, 0, 0);
    pros::delay(4000);
    chassis.moveToPoint(0, 10, 1000);
}
void skills() {

}
void autonomous() {
    // redRight();
    redLeft();
    // soloWinPointright();
    // moveForward();
}


    /**
     * Runs in driver control
     */

void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        return;
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);

            // buttons for controller

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) { toggle_descore(); }
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) { toggle_matchload(); }

            // Control Intake using shoulder buttons (L1/L2)

            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                set_intakef(12000);

            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                intakef.move_velocity(-200);
                intakeb.move_velocity(-50);
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                set_both(12000);
            } else {
                set_both(0);
            }

        // delay to save resources
        pros::delay(10);
    }
}
