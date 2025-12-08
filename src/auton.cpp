#include "main.h"
#include <cmath>

double TARGL = 0, TARGR = 0, ER, EL, KP, CUR, CUL, LPOW, RPOW;
double startTime = 0, currentTime = 0;
double abscap(double pow, double cap) { return fmax(fmin(pow, cap), -cap); }

double applyMinMax(double pow, double minSpeed, double maxSpeed)
{
    // Clamp to max first
    pow = abscap(pow, maxSpeed);

    // Enforce minimum speed (if not zero)
    if (fabs(pow) > 0 && fabs(pow) < minSpeed)
    {
        pow = (pow > 0) ? minSpeed : -minSpeed;
    }

    return pow;
}

double avgPos(Motor &m1, Motor &m2, Motor &m3)
{
    return (m1.get_position() + m2.get_position() + m3.get_position()) / 3.0;
}

void baseGo(double L, double R, double p_KP, double p_KD, double LEEWAY, double TimeOut, double MinSpeed, double MaxSpeed)
{
    Motor L1(LEFT1, MotorGears::blue, MotorUnits::degrees);
    Motor L2(LEFT2, MotorGears::blue, MotorUnits::degrees);
    Motor L3(LEFT3, MotorGears::blue, MotorUnits::degrees);

    Motor R1(RIGHT1, MotorGears::blue, MotorUnits::degrees);
    Motor R2(RIGHT2, MotorGears::blue, MotorUnits::degrees);
    Motor R3(RIGHT3, MotorGears::blue, MotorUnits::degrees);

    L1.tare_position();
    L2.tare_position();
    L3.tare_position();
    R1.tare_position();
    R2.tare_position();
    R3.tare_position();

    Controller master(E_CONTROLLER_MASTER);

    EL = L / INPERDEG;
    ER = R / INPERDEG;

    double LeftPos = avgPos(L1, L2, L3);
    double RightPos = avgPos(R1, R2, R3);

    double prevEL = 0, prevER = 0;
    double DEL = 0, DER = 0; // D
    double KD = 0.2;

    TARGL = LeftPos + EL;
    TARGR = RightPos + ER;
    startTime = millis();
    currentTime = startTime;

    KP = p_KP;

    while ((fabs(EL) >= LEEWAY || fabs(ER) >= LEEWAY) &&
           (currentTime - startTime < TimeOut))
    {
        currentTime = millis();
        CUL = avgPos(L1, L2, L3);
        CUR = avgPos(R1, R2, R3);

        EL = TARGL - CUL;
        ER = TARGR - CUR;

        DEL = EL - prevEL;
        DER = ER - prevER;
        prevEL = EL;
        prevER = ER;

        double LOUT = EL * KP + DEL * KD; // D
        double ROUT = ER * KP + DER * KD;

        LPOW = applyMinMax(LOUT, MinSpeed, MaxSpeed);
        RPOW = applyMinMax(ROUT, MinSpeed, MaxSpeed);

        L1.move(LPOW); // D
        L2.move(LPOW);
        L3.move(LPOW);

        R1.move(RPOW);
        R2.move(RPOW);
        R3.move(RPOW);

        master.print(2, 0, "L: %.2f", EL);
        printf("L: %.2f", EL);
        delay(20);
    }

    printf("my neck hurts");
    master.print(2, 0, "not running");
    L1.move(0);
    L2.move(0);
    L3.move(0);

    R1.move(0);
    R2.move(0);
    R3.move(0);
}

double target = 0, current = 0, error, power;

void gyroTurn(double Angle, double Kp, double leeway, int Timeout,
              double minSpeed, double maxSpeed)
{
    // Drivetrain motors
    Motor L1(LEFT1, MotorGears::blue, MotorUnits::degrees);
    Motor L2(LEFT2, MotorGears::blue, MotorUnits::degrees);
    Motor L3(LEFT3, MotorGears::blue, MotorUnits::degrees);

    Motor R1(RIGHT1, MotorGears::blue, MotorUnits::degrees);
    Motor R2(RIGHT2, MotorGears::blue, MotorUnits::degrees);
    Motor R3(RIGHT3, MotorGears::blue, MotorUnits::degrees);

    IMU imu(INERTIAL);

    L1.tare_position();
    L2.tare_position();
    L3.tare_position();
    R1.tare_position();
    R2.tare_position();
    R3.tare_position();

    // Starting heading and target heading
    double current = imu.get_rotation(); // current angle in deg
    double target = current + Angle;

    // Timing
    int startTime = millis();
    int currentTime = startTime;

    while ((currentTime - startTime < Timeout) && error > leeway)
    {
        currentTime = millis();

        double currentHeading = imu.get_rotation();
        error = target - currentHeading;

        double power = Kp * error;

        power = abscap(power, maxSpeed);

        // Enforce min power
        if (fabs(power) > 0 && fabs(power) < minSpeed)
        {
            power = (power > 0) ? minSpeed : -minSpeed;
        }

        // Apply turn
        L1.move(power);
        L2.move(power);
        L3.move(power);

        R1.move(-power);
        R2.move(-power);
        R3.move(-power);

        pros::delay(20);
    }

    // Stop motors
    L1.move(0);
    L2.move(0);
    L3.move(0);
    R1.move(0);
    R2.move(0);
    R3.move(0);
}

/*


void baseTurn(double bearing, double kp)
{
    double turnL = bearing * TORAD * DIS;
    double turnR = -bearing * TORAD * DIS;
    baseGo(turnL, turnR, kp);
}


*/