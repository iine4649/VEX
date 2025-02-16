#include "main.h"
#include "pros/adi.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
\
#define LF_PORT 1
#define LB_PORT 16

#define RF_PORT 10
#define RB_PORT 18
#define IU_PORT 13
#define ID_PORT 20
// controller


// motor groups
pros::Controller controller1(pros::E_CONTROLLER_MASTER);
pros::Motor Intake(-8,pros::MotorGearset::blue);
pros::MotorGroup leftMotors({-5, -20,18},pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({-10, 7,2}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)
pros::Motor lbL(-12);
pros::Motor lbR(13);
pros::Rotation lbRot(17);

// motors
pros::adi::Pneumatics BackC('A',false);
pros::adi::Pneumatics doinkerL('C',false);
pros::adi::Pneumatics doinkerR('B',false);
pros::adi::Pneumatics intakeR('D',false);
pros::Optical americanPolice(15);
/*
pros::Motor IntakeU(13);
pros::Motor IntakeD(20);*/
// Inertial Sensor on port 10
pros::Imu imu(14);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(-1);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(4);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -2.72);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -0.051);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.817, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            12, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             100, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
    bool redRace = 1;
    bool blueRace = 0;
    double intakeVel = 127;
 void activeRacism(){
    if(redRace){
        if(0 < americanPolice.get_hue() && americanPolice.get_hue() < 20){
            pros::delay(50);
            intakeVel = -127;
            //Intake.brake();
            pros::delay(90);
            intakeVel = 127;
        }
    }
    if(blueRace){
        if(150 < americanPolice.get_hue() && americanPolice.get_hue() < 200){
            pros::delay(50);
            intakeVel = -127;
            //Intake.brake();
            pros::delay(90);
            intakeVel = 127;
        }
    }
 }
 
 int currState = 0;

 bool special = 0;
 void specialState(){
    if(special){
        special = 0;
        currState = 0;
    } else {
        special = 1;
        currState = 3;
    }
 }
 const int numStates = 3;
 //make sure these are in centidegrees (1 degree = 100 centidegrees)
 int states[4] = {500, 3500, 18000, 22500};
 int target = 0;
 
 void nextState() {
     currState += 1;
     if (currState >= numStates) {
         currState = 0;
     }
 }

 
 void liftControl() {
     target = states[currState];
     double kp = 0.008;
     double kd = 0.0005; 
     double ki = 0.001;
     double error = target - lbRot.get_position();
     double preverror; 
     double derivative = error - preverror;
     preverror = error;
     double totalerror;
     totalerror += error;
     double velocity = kp * error + kd * derivative + ki * totalerror;
     lbL.move(velocity);
     lbR.move(velocity);
 }

 
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    pros::Task liftControlTask([]{
        while (true) {
            liftControl(); 
            activeRacism();    // infinite loop
                // print measurements from the adi encoder
            pros::lcd::print(0, "Horizontal Encoder: %i", horizontalEnc.get_position());
                // print measurements from the rotation sensor
            pros::lcd::print(1, "Vertical Encoder: %i", verticalEnc.get_position());
            pros::lcd::print(3, "IntakeVel: %i", intakeVel);
            pros::delay(10);
        }
    });
/*
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x003a57), LV_PART_MAIN);
*/
/*Create a white label, set its text and align it to the center*/
  /*  lv_obj_t * label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Hello world");
    lv_obj_set_style_text_color(label, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);*/
    
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
ASSET(firstmove_txt);
ASSET(second_txt);
/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() 
    {/*
     blueRace = 1;
     pros::Task liftControlTask([]{
        while (true) {
            activeRacism();
            pros::delay(10);
        }
    });*/
    /*chassis.setPose(0, 0, 0);
    chassis.moveToPoint(5.011, 34.006, 300);
    chassis.moveToPoint(1.321, 46.548, 300);*/

    /*
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    //pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    //pros::lcd::print(4, "pure pursuit finished!");
    */
    // set position to x:0, y:0, heading:0
    chassis.setPose(-62.712, 36.074, 90);
    
    Intake.move(127);
    chassis.follow(firstmove_txt,5,5500);
    chassis.waitUntilDone();
    doinkerL.extend();
    pros::delay(1000);
    Intake.brake();
    // lookahead distance: 15 inches
    // timeout: 2000 ms
    //bluenegativeAuton
    /*chassis.moveToPose(-14.5,35.937,90,1750,{.maxSpeed = 127, .minSpeed=126, .earlyExitRange=3});
    chassis.waitUntilDone();
    doinkerL.extend();
    
    chassis.moveToPose(-12,50.403,80,30000,{.maxSpeed = 127, .minSpeed=2,.earlyExitRange=30});
    chassis.waitUntilDone();
    pros::delay(2000);
    chassis.swingToHeading(90,DriveSide::RIGHT,1500,{.direction = AngularDirection::CW_CLOCKWISE,.maxSpeed = 84, .minSpeed = 60 });
    chassis.waitUntilDone();*/
    /*chassis.moveToPose(-15,24,100,1750,{.forwards=false,.minSpeed=125});
    chassis.waitUntilDone();
    pros::delay(5000);*/
    chassis.moveToPose(-23,20,0,1750,{.forwards=false,.maxSpeed = 100, .minSpeed=80});
    //chassis.setPose(0, 0, 0);
    //chassis.follow(second_txt, 20, 4000,false);
    //chassis.moveToPose(21.)
    chassis.waitUntilDone();
    BackC.extend();
    Intake.move(127);
    doinkerL.retract();
    //chassis.turnToHeading(5,100,{.maxSpeed = 86, .minSpeed = 60});
    //chassis.waitUntilDone();
    doinkerR.extend();
    chassis.swingToHeading(300,DriveSide::LEFT,500,{.direction = AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed = 84, .minSpeed = 60 });
    chassis.waitUntilDone();
    chassis.moveToPose(-24,50,14.7,1750,{.forwards = true});
    
    
}

/**
 * Runs in driver control
 */
void opcontrol() {
    bool backBool = 0;
    bool doinker = 0;
    bool IntakeP = 0;
    bool IntakeU = 0;
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        if(controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            if(backBool){
                BackC.retract();
                backBool = 0;
            }else if(!backBool){
                BackC.extend();
                backBool = 1;

            }
        }
        if(controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            if(doinker){
                doinkerR.retract();
                //doinkerL.retract();
                doinker = 0;
            }else if(!doinker){
                doinkerR.extend();
                //doinkerL.extend();

                doinker = 1;

            }
        }
        if(controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            if(IntakeP){
                doinkerL.retract();
                IntakeP = 0;
            }else if(!IntakeP){
                doinkerL.extend();
                IntakeP = 1;

            }
        }
        if(controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
            if(IntakeU){
                intakeR.retract();
                IntakeU = 0;
            }else if(!IntakeU){
                intakeR.extend();
                IntakeU = 1;

            }
        }
        //doinker A 
        if(controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            Intake.move(intakeVel);
        }else if (controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            Intake.move(intakeVel);
        }else{
        Intake.brake();
        }
        
        if (controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			nextState();
		}
        if (controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			specialState();
		}


        // delay to save resources
        pros::delay(20);
    }
}