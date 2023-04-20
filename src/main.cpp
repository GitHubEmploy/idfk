#include "main.h"

#define FLYWHEEL_TARGET_RPM 200 // adjust this value to set the desired flywheel speed
pros::Motor flywheel(1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

enum class FlywheelState {
    idle,
    auton,
    driver
};

FlywheelState flywheel_state = FlywheelState::idle;

std::shared_ptr<OdomChassisController> chassis =
        ChassisControllerBuilder()
                .withMotors(
                        {-10, 19, -20}, // Left motors are 1 & 2 (reversed)
                        {11, -12, 13}    // Right motors are 3 & 4
                )
                .withGains(
                        {1.7, 0.00005, 0.000000005}, // Distance controller gains
                        {0.9, 0.125, 0.06}  // Turn controller gains
                )
                // green gearset, 4 inch wheel diameter, 11.5 inch wheel track
                .withDimensions(AbstractMotor::gearset::green, {{4_in, 10.25_in}, imev5BlueTPR})
                .withOdometry() // use the same scales as the chassis (above)
                .buildOdometry(); // build an odometry chassis

std::shared_ptr<AsyncMotionProfileController> profileController =
        AsyncMotionProfileControllerBuilder()
                .withLimits({
                    1.0, // Maximum linear velocity of the Chassis in m/s
                    2.0, // Maximum linear acceleration of the Chassis in m/s/s
                    10.0 // Maximum linear jerk of the Chassis in m/s/s/s
                })
                .withOutput(chassis)
                .buildMotionProfileController();

void flywheelControlTask () {
    double kp = 0.3; // adjust these values to tune the PIDF controller
    double ki = 0.01;
    double kd = 0.1;
    double kf = 0.07;
    double maxVelocity = flywheel.get_target_velocity();

    double integral = 0;
    double derivative = 0;
    double lastError = 0;

    while (true) {
        switch (flywheel_state) {
            case FlywheelState::idle:
            {
                double error = 0 - flywheel.get_actual_velocity();
                integral += error;
                derivative = error - lastError;
                lastError = error;

                double output = (kp * error) + (ki * integral) + (kd * derivative) + (kf * 0);
                output = (output / maxVelocity) * 127;
                flywheel.move_voltage(output);
            }
                break;
            case FlywheelState::auton:
            {
                double error = 200 - flywheel.get_actual_velocity();
                integral += error;
                derivative = error - lastError;
                lastError = error;

                double output = (kp * error) + (ki * integral) + (kd * derivative) + (kf * 200);
                output = (output / maxVelocity) * 127;
                flywheel.move_voltage(output);
            }
                break;
            case FlywheelState::driver:
            {
                double error = 100 - flywheel.get_actual_velocity();
                integral += error;
                derivative = error - lastError;
                lastError = error;

                double output = (kp * error) + (ki * integral) + (kd * derivative) + (kf * 100);
                output = (output / maxVelocity) * 127;
                flywheel.move_voltage(output);
            }
                break;
        }

        if (flywheel.get_actual_velocity() >= FLYWHEEL_TARGET_RPM) {
            flywheel.move_voltage(kf * maxVelocity);
        }

        pros::delay(20);
    }
}

pros::Task mytask(flywheelControlTask);

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    chassis->turnAngle(90_deg);
    chassis->moveDistance(2_ft); // sqrt(1^2 + 1^2)
    profileController->generatePath(
            {{0_ft, 0_ft, 0_deg}, {3_ft, 3_ft, 0_deg}}, "A");
    profileController->setTarget("A");
    profileController->waitUntilSettled();

}


void opcontrol() { // flywheel_state = FlywheelState::driver;
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	while (true) {
        chassis->getModel()->tank(master.get_analog(ANALOG_LEFT_Y),
                                master.get_analog(ANALOG_RIGHT_Y));

        if (master.get_digital(DIGITAL_R1)) {
            flywheel_state = FlywheelState::driver;
        }
        if (master.get_digital(DIGITAL_R2)) {
            flywheel_state = FlywheelState::idle;
        }


	}
}
