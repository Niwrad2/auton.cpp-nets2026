#include "main.h"
#include <cmath> // added

bool MechDown = false;

// add constants for deadband/tolerance
double avgtemp(Motor &m1, Motor &m2, Motor &m3)
{
    int NOMs = 0;
    double SUMs = 0;
    double val1s = 1.0 / 1.0;  // Positive infinity
    double val2s = -1.0 / 0.0; // Negative infinity

    if (std::isnan(m1.get_temperature()) || (std::isinf(m1.get_temperature()))) {
        NOMs += 1;
    } else {
        NOMs +=1;
        SUMs += m1.get_temperature();
    }

    if (std::isnan(m2.get_temperature()) || (std::isinf(m2.get_temperature()))) {
        NOMs += 0;
    } else {
        NOMs +=1;
        SUMs += m2.get_temperature();
    }

    if (std::isnan(m3.get_temperature()) || (std::isinf(m3.get_temperature()))) {
        NOMs += 1;
    } else {
        NOMs +=1;
        SUMs += m3.get_temperature();
    }

    if (NOMs == 0) {
        return 0.0; 
    }

    return SUMs / NOMs;

    
}

void Mech_Task(void *param)

{
	Motor Mech(LITTLE_WILL_MECH, v5::MotorGear::green, v5::MotorUnits::degrees);
	Controller master(E_CONTROLLER_MASTER);
	double error;

	int counter_thingy = 0;
	while (true)

	{
		if (MechDown)
		{
			counter_thingy += 1;
			error = MECH_ACTIVE_POSITION - Mech.get_position();
			Mech.move(error * MECH_KP);
		}
		else
		{
			if (counter_thingy == 1) {
				error = 2220 - Mech.get_position();
				Mech.move(error * MECH_KP);
			} else if (counter_thingy >= 2) {
			error = MECH_REST_POSITION - Mech.get_position();
			Mech.move(error * MECH_KP);
			}	
		}


		// replace direct move with deadband/tolerance-aware output


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

void health(double time, double direction, double power) {

	Motor L1(LEFT1);
	Motor L2(LEFT2);
	Motor L3(LEFT3);

	Motor R1(RIGHT1);
	Motor R2(RIGHT2);
	Motor R3(RIGHT3);

	Motor Mech(LITTLE_WILL_MECH);
	Motor Conveyer(CONVEYER);
	Motor Rollers(ROLLERS);

	L1.move(direction * power * 0.88);
	L2.move(direction * power * 0.88);
	L3.move(direction * power * 0.88);
	R1.move(direction * power);
	R2.move(direction * power);
	R3.move(direction * power);
	delay(time);
	L1.move(0);
	L2.move(0);
	L3.move(0);
	R1.move(0);
	R2.move(0);
	R3.move(0);

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



		master.print(2, 0, "%.2f", (avgtemp(L1, L2, L3)));
		master.print(2, 14, "%.2f", (avgtemp(R1, R2, R3)));
	}
}


void The_Very_Sigma_Autonomous(int Which_autonomous_do_you_want) 
{

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

	
		baseGo(46.5, 46.5, 0.3, 0, 5, 1500, 80); //take note of this distance
		gyroTurn(-90, 3, 0.5, 1300, 90);
		gyroTurn(-90, 3, 0.1, 1300, 90);

		//reach loading
		master.rumble("..");
		MechDown = true;
		delay(100);

		Conveyer.move(127);
		baseGo(11, 11, 0.3, 0, 5, 450, 90);
		health(225, 1, 60);
		//loader
		delay(2500); //laoding time
		//baseGo(-5, -4, 0.3, 0, 5, 700, 90);//tuneing shit
		gyroTurn(-90, 3, 0.5, 1000, 90);
		baseGo(-56, -59, 0.3, 0, 5, 900, 90);
		health(300, -1, 127);
		delay(150);
		Conveyer.move(0);
		MechDown = false;

		Rollers.move(127);
		Conveyer.move(127);
		delay(3000);
		Rollers.move(0);
		Conveyer.move(0);
		


		baseGo(10, 10, 0.3, 0, 5, 600, 90);
		gyroTurn(-180, 3, 0.5, 3000, 90);
		baseGo(-30, -30, 0.3, 0, 5, 1000, 80);
		baseGo(5, 5, 0.3, 0, 5, 1000, 80);
		gyroTurn(90, 3, 0.5, 1500, 90);
		gyroTurn(90, 3, 0.1, 1200, 90);
		//delay(200);
		baseGo(15, 25, 0.3, 0, 5, 2000, 99);
		baseGo(40, 45, 0.3, 0, 5, 2000, 99);
		/*gyroTurn(-90, 3, 0.1, 1300, 90);
		baseGo(-16, -16, 0.3, 0, 5, 2000, 99);
		gyroTurn(45, 3, 0.5, 1300, 90);
		gyroTurn(45, 3, 0.5, 1300, 90);
		delay(150);
		baseGo(14, 14, 0.3, 0, 5, 600, 80);    
		gyroTurn(90, 3, 0.5, 1500, 90); 

		baseGo(-24, -24, 0.3, 0, 5, 600, 80);
		Conveyer.move(127);
		health(800, -1, 127); */
	

	






	}

	else if (Which_autonomous_do_you_want == 2) {
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

	
		baseGo(46.5, 46.5, 0.3, 0, 5, 1500, 80); //take note of this distance
		gyroTurn(-90, 3, 0.5, 1300, 90);
		gyroTurn(-90, 3, 0.1, 1300, 90);

		//reach loading
		master.rumble("..");
		MechDown = true;
		delay(100);

		Conveyer.move(127);
		baseGo(11, 11, 0.3, 0, 5, 450, 90);
		health(225, 1, 60);
		//loader
		delay(2500); //laoding time
		//baseGo(-5, -4, 0.3, 0, 5, 700, 90);//tuneing shit
		gyroTurn(-90, 3, 0.5, 1000, 90);
		baseGo(-56, -59, 0.3, 0, 5, 900, 90);
		health(300, -1, 127);
		delay(150);
		Conveyer.move(0);
		MechDown = false;

		Rollers.move(127);
		Conveyer.move(127);
		delay(3000);
		Rollers.move(0);
		Conveyer.move(0);
		


		baseGo(10, 10, 0.3, 0, 5, 600, 90);
		gyroTurn(-180, 3, 0.5, 3000, 90);
		delay(200);
		gyroTurn(-180, 3, 0.5, 3000, 90);
		baseGo(-30, -30, 0.3, 0, 5, 3000, 50);
		baseGo(63.5, 63.5, 0.3, 0, 5, 3000, 80);
		gyroTurn(-90, 3, 0.5, 1000, 90);
		delay(100);
		gyroTurn(-90, 3, 0.1, 2000, 90);
		delay(100);
		gyroTurn(-90, 3, 0.1, 2000, 90);
		Conveyer.move(127);
		health(800, 1, 127);


		
		/*gyroTurn(-90, 3, 0.1, 1300, 90);
		baseGo(-16, -16, 0.3, 0, 5, 2000, 99);
		gyroTurn(45, 3, 0.5, 1300, 90);
		gyroTurn(45, 3, 0.5, 1300, 90);
		delay(150);
		baseGo(14, 14, 0.3, 0, 5, 600, 80);    
		gyroTurn(90, 3, 0.5, 1500, 90); 

		baseGo(-24, -24, 0.3, 0, 5, 600, 80);
		Conveyer.move(127);
		health(800, -1, 127); */
	
	


		

	} 
	else if (Which_autonomous_do_you_want == 3) {
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
		//gyroTurn(90, 3.25, 0.5, 5000, 90);
		baseGo(47, 47, 0.3, 0, 5, 1500, 80); //take note of this distance
		gyroTurn(-90, 3, 0.5, 1300, 90);
		gyroTurn(-90, 3, 0.1, 1300, 90);

		//reach loading
		master.rumble("..");
		MechDown = true;
		delay(100);

		Conveyer.move(127);
		baseGo(11, 11, 0.3, 0, 5, 450, 90);
		health(175, 1, 50);
		//loader
		delay(2500); //laoding time
		//baseGo(-5, -4, 0.3, 0, 5, 700, 90);//tuneing shit
		gyroTurn(-90, 3, 0.5, 1000, 90);
		baseGo(-56, -56, 0.3, 0, 5, 900, 90);
		health(300, -1, 127);
		delay(150);
		Conveyer.move(0);
		MechDown = false;
		//reached long goal



		baseGo(10, 10, 0.3, 0, 5, 600, 90);
		gyroTurn(-45, 3, 0.5, 1000, 90);
		baseGo(-22, -22, 0.3, 0, 5, 1000, 80);
		gyroTurn(-90, 3, 0.5, 1500, 90);
		gyroTurn(-90, 3, 0.1, 1200, 90);
		//delay(200);
		baseGo(-40, -40, 0.3, 0, 5, 2000, 99);
		gyroTurn(-90, 3, 0.1, 1300, 90);
		baseGo(-16, -16, 0.3, 0, 5, 2000, 99);
		gyroTurn(45, 3, 0.5, 1300, 90);
		gyroTurn(45, 3, 0.5, 1300, 90);
		delay(150);
		baseGo(14, 14, 0.3, 0, 5, 600, 80);    
		gyroTurn(90, 3, 0.5, 1500, 90); 

		baseGo(-24, -24, 0.3, 0, 5, 600, 80);
		Conveyer.move(127);
		health(800, -1, 127);

		/*health(1800, -1, 127);
		gyroTurn(90, 3, 2, 400, 90);
		baseGo(6, 4, 0.3, 0, 5, 200, 80);
		delay(20);
		gyroTurn(90, 3, 0.5, 1000, 90);
		delay(20); */
		//health(300, -1, 127);
		MechDown = true;
		Rollers.move(127);
		Conveyer.move(127);
		delay(3000);
		Rollers.move(0);



		MechDown = true;
		//finsih scoring


		gyroTurn(90, 3, 0.5, 1500, 90);
		baseGo(27, 27, 0.3, 0, 5, 700, 90);
		health(200, 1, 70);
		health(200, -1, 50);
		health(340, 1, 50);

		
		delay(2000);


		gyroTurn(90, 3, 0.5, 1500, 90);
		baseGo(-56, -56, 0.3, 0, 5, 900, 90);
		delay(500);

		Rollers.move(127);
		Conveyer.move(127);
		delay(3000);

		Conveyer.move(0);
		Rollers.move(0);

		baseGo(10, 10, 0.3, 0, 5, 600, 90);
		gyroTurn(180, 3, 0.5, 1000, 90);
		baseGo(115, 115, 0.3, 0, 5, 1500, 90);
		gyroTurn(90, 3, 0.5, 1000, 90);
		baseGo(-100, -120, 0.6, 0, 5, 1000, 90);
		/*gyroTurn(45, 3, 0.5, 1000, 90);
		baseGo(10, 10, 0.3, 0, 5, 2000, 80);
		gyroTurn(90, 3, 0.5, 1000, 90); */

	}
}

void autonomous() 
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

	L1.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	L2.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	L3.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	R1.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	R2.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	R3.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

//1 = skills uncomplete
//2 = skills with park
//3 = skills


	The_Very_Sigma_Autonomous(2);

}




