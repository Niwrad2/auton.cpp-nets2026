#include "main.h"
#include <cmath> // added

bool MechDown = false;

// add constants for deadband/tolerance
constexpr double MECH_MIN_OUTPUT = 16.0;           // minimum applied motor output to overcome deadband
constexpr double MECH_POSITION_TOLERANCE = 2.0;    // degrees: consider "on target" when error within this

void Mech_Task(void *param)

{
	Motor Mech(LITTLE_WILL_MECH, v5::MotorGear::green, v5::MotorUnits::degrees);
	Controller master(E_CONTROLLER_MASTER);
	double error;

	while (true)

	{
		if (MechDown)
		{
			error = MECH_ACTIVE_POSITION - Mech.get_position();
		}
		else
		{
			error = MECH_REST_POSITION - Mech.get_position();
		}

		// replace direct move with deadband/tolerance-aware output
		double output = error * MECH_KP;

		if (std::fabs(output) < MECH_MIN_OUTPUT) {
			if (std::fabs(error) <= MECH_POSITION_TOLERANCE) {
				output = 0.0; // close enough: stop moving
			} else {
				// keep enough power to overcome static friction
				output = (output > 0.0) ? MECH_MIN_OUTPUT : -MECH_MIN_OUTPUT;
			}
		}

		Mech.move(output);
		delay(20);
	}
}


void jerk(double forward_time, double Fstrength, double backward_time, double Bstrength, int times)
{

	Motor L1(LEFT1);
	Motor L2(LEFT2);
	Motor L3(LEFT3);

	Motor R1(RIGHT1);
	Motor R2(RIGHT2);
	Motor R3(RIGHT3);

	Motor Mech(LITTLE_WILL_MECH);
	Motor Conveyer(CONVEYER);
	Motor Rollers(ROLLERS);

	while (times > 0)
	{
		

		times -= 1;
	}
}

void health(void* ignore) {
	
}

void good (void* ignore) {


	Motor L1(LEFT1);
	Motor L2(LEFT2);
	Motor L3(LEFT3);

	Motor R1(RIGHT1);
	Motor R2(RIGHT2);
	Motor R3(RIGHT3);

	Motor Mech(LITTLE_WILL_MECH);
	Motor Conveyer(CONVEYER);
	Motor Rollers(ROLLERS);

	Imu imu(INERTIAL);
	Optical optical_sensor(OPTICAL);
	Controller master(E_CONTROLLER_MASTER);
	master.print(0, 0, "%.2f", optical_sensor.get_hue());
	master.print(2, 0, "%.2f", (L1.get_temperature() + L2.get_temperature() + L3.get_temperature()) / 3);
	master.print(2, 14, "%.2f", (R1.get_temperature() + R2.get_temperature() + R3.get_temperature()) / 3);

	delay(15);
}
void initialize() {

	Motor L1(LEFT1);
	Motor L2(LEFT2);
	Motor L3(LEFT3);

	Motor R1(RIGHT1);
	Motor R2(RIGHT2);
	Motor R3(RIGHT3);

	Motor Mech(LITTLE_WILL_MECH);
	Motor Conveyer(CONVEYER);
	Motor Rollers(ROLLERS);

	Imu imu(INERTIAL);
	Optical optical_sensor(OPTICAL);

	Controller master(E_CONTROLLER_MASTER);



	imu.reset(true);
    while (imu.is_calibrating()) {
        pros::delay(20);
    }
	
	master.clear();

	Task MechTask(Mech_Task, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Mech");


	Task HLTEMPANALYSIS(good, (void*)"PROS", TASK_PRIORITY_MIN, TASK_STACK_DEPTH_DEFAULT);
}

void disabled() {

}

void competition_initialize() {}

void opcontrol()
{	
	Motor L1(LEFT1);
	Motor L2(LEFT2);
	Motor L3(LEFT3);

	Motor R1(RIGHT1);
	Motor R2(RIGHT2);
	Motor R3(RIGHT3);

	Motor Mech(LITTLE_WILL_MECH);
	Motor Conveyer(CONVEYER);
	Motor Rollers(ROLLERS);

	Imu imu(INERTIAL);
	Optical optical_sensor(OPTICAL);


	Controller master(E_CONTROLLER_MASTER);



	L1.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	L2.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	L3.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	R1.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	R2.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	R3.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	int leftY, rightY;
	int r1, r2, l1, l2, A, B, SLOW;

	while (true)
	{

	
		leftY = master.get_analog(ANALOG_LEFT_Y);
		rightY = master.get_analog(ANALOG_RIGHT_Y);

		r1 = master.get_digital(DIGITAL_R1);
		r2 = master.get_digital(DIGITAL_R2);
		l1 = master.get_digital(DIGITAL_L1);
		l2 = master.get_digital(DIGITAL_L2);
		A = master.get_digital(DIGITAL_A);
		B = master.get_digital(DIGITAL_B);
		SLOW = master.get_digital(DIGITAL_UP);	
		
		//intake
		if (master.get_digital(DIGITAL_Y)) {
			Conveyer.move(127);

		} else {
			Conveyer.move((r1 - r2) * 127);

		}


		

		if (master.get_digital(DIGITAL_A))
		{
			L1.move(-127);
			L2.move(-127);
			L3.move(-127);

			R1.move(-127);
			R2.move(-127);
			R3.move(-127);
		}
		else 
		{
			L1.move(leftY);
			L2.move(leftY);
			L3.move(leftY);

			R1.move(rightY);
			R2.move(rightY);
			R3.move(rightY);
		}
		

		

		if (master.get_digital_new_press(DIGITAL_B))
		{
			MechDown = !MechDown;
		}
		
		//master.print(0, 0, "%.2f", optical_sensor.get_hue());
		if (master.get_digital(DIGITAL_Y)) {
			Rollers.move(50);

		} else {

			Rollers.move((l1 - l2) * 127);

		}
		

		/*if (optical_sensor.get_hue() < 160) {
			Rollers.move((l1 - l2) * 127);
		} 

		else if 
			(optical_sensor.get_hue() > 160) {
			Rollers.move(-127);
			delay(250);
			Rollers.move(0);
		}
		// 	*/
		// if (Conveyer.get_temperature() > 45) {
		// 	master.rumble(".");
		// }


	}
}


void The_Very_Sigma_Autonomous(int Which_autonomous_do_you_want) {

	if (Which_autonomous_do_you_want == 1) {
		Motor L1(LEFT1);
		Motor L2(LEFT2);
		Motor L3(LEFT3);

		Motor R1(RIGHT1);
		Motor R2(RIGHT2);
		Motor R3(RIGHT3);

		Motor Mech(LITTLE_WILL_MECH);
		Motor Conveyer(CONVEYER);
		Motor Rollers(ROLLERS);

		Imu imu(INERTIAL);
		Optical optical_sensor(OPTICAL);

		L1.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
		L2.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
		L3.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

		R1.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
		R2.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
		R3.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
		Controller master(E_CONTROLLER_MASTER);

	
		//baseGo(20, 20, 0.1, 0, 5, 5000, 0, 80);
		//gyroTurn(90, 3, 0.5, 5000, 0, 90);
		//gyroTurn(180, 3.25, 0.5, 5000, 0, 90);

		Conveyer.move(127);
		baseGo(40, 35, 0.5, 0, 10, 5000, 80);
		MechDown = true;
		gyroTurn(90, 3, 0.5, 5000, 90);
	








	}

	else if (Which_autonomous_do_you_want == 2) {}

}
void autonomous() {
	
	Motor L1(LEFT1);
	Motor L2(LEFT2);
	Motor L3(LEFT3);

	Motor R1(RIGHT1);
	Motor R2(RIGHT2);
	Motor R3(RIGHT3);

	Motor Mech(LITTLE_WILL_MECH);
	Motor Conveyer(CONVEYER);
	Motor Rollers(ROLLERS);

	Imu imu(INERTIAL);
	Optical optical_sensor(OPTICAL);

	L1.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	L2.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	L3.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	R1.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	R2.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	R3.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

//1 = 
//2 = 
//3 = 


	The_Very_Sigma_Autonomous(1);

}




