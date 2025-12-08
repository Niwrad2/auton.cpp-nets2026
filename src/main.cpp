#include "main.h"

bool MechDown = false;

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

		Mech.move(error * MECH_KP);
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

	IMU imu(INERTIAL);

	Task MechTask(Mech_Task, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Mech");

	imu.reset();
    while (imu.is_calibrating()) {
        pros::delay(20);
    }
	
}

void disabled() {}

void competition_initialize() {}

void opcontrol()
{

	Controller master(E_CONTROLLER_MASTER);

	Motor L1(LEFT1);
	Motor L2(LEFT2);
	Motor L3(LEFT3);

	Motor R1(RIGHT1);
	Motor R2(RIGHT2);
	Motor R3(RIGHT3);

	Motor Mech(LITTLE_WILL_MECH);
	Motor Conveyer(CONVEYER);
	Motor Rollers(ROLLERS);

	L1.set_brake_mode(E_MOTOR_BRAKE_COAST);
	L2.set_brake_mode(E_MOTOR_BRAKE_COAST);
	L3.set_brake_mode(E_MOTOR_BRAKE_COAST);

	R1.set_brake_mode(E_MOTOR_BRAKE_COAST);
	R2.set_brake_mode(E_MOTOR_BRAKE_COAST);
	R3.set_brake_mode(E_MOTOR_BRAKE_COAST);

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
		if (r2 > 0)
		{

			Conveyer.move((r1 - r2) * 127);

		}
		else
		{

			Conveyer.move((r1 - r2) * 127);
;
		}

		Rollers.move((l1 - l2) * 127);

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




		
			
		
		
	}
}

void The_Very_Sigma_Autonomous(int Which_autonomous_do_you_want) {

	if (Which_autonomous_do_you_want == 1) {

		baseGo(10, 10, 1, 1, 5, 3000, 50, 100);






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

	IMU imu(INERTIAL);


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




