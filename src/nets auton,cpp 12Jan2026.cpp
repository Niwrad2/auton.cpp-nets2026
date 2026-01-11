#include "main.h"
#include <cmath>

double TARGL = 0, TARGR = 0, ER, EL, KP, KD, CUR, CUL, LPOW, RPOW;
double startTime = 0, currentTime = 0;
double abscap(double pow, double cap) { return fmax(fmin(pow, cap), -cap); }


double avgPos(Motor &m1, Motor &m2, Motor &m3)
{
    int NOM = 0;
    double SUM = 0;
    double val1 = 1.0 / 0.0;  // Positive infinity
    double val2 = -1.0 / 0.0; // Negative infinity

    if (std::isnan(m1.get_position()) || (std::isinf(m1.get_position()))) {
        NOM += 0;
    } else {
        NOM +=1;
        SUM += m1.get_position();
    }

    if (std::isnan(m2.get_position()) || (std::isinf(m2.get_position()))) {
        NOM += 0;
    } else {
        NOM +=1;
        SUM += m2.get_position();
    }

    if (std::isnan(m3.get_position()) || (std::isinf(m3.get_position()))) {
        NOM += 0;
    } else {
        NOM +=1;
        SUM += m3.get_position();
    }

    if (NOM == 0) {
        return 0.0; 
    }

    return SUM / NOM;

    
}

void baseGo(double L, double R, double p_KP, double p_KD, double LEEWAY, double TimeOut, double MaxSpeed)
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

    TARGL = LeftPos + EL;
    TARGR = RightPos + ER;
    startTime = millis();
    currentTime = startTime;
    KD = p_KD;
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

        L1.move(abscap(LOUT, MaxSpeed)); // D
        R1.move(abscap(ROUT, MaxSpeed));
        L2.move(abscap(LOUT, MaxSpeed));
        R2.move(abscap(ROUT, MaxSpeed));
        L3.move(abscap(LOUT, MaxSpeed));
        R3.move(abscap(ROUT, MaxSpeed));


        delay(20);
    }

    L1.move(0);
    R1.move(0);

    L2.move(0);
    R2.move(0);

    L3.move(0);
    R3.move(0);
}

// add helper to compute minimal signed angle difference (-180..180)
static double shortestAngleDiff(double target, double current)
{
    double diff = std::fmod(target - current, 360.0);
    if (diff < -180.0) diff += 360.0;
    else if (diff > 180.0) diff -= 360.0;
    return diff;
}

double target = 0, currentPos = 0, error, power, startPos;

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

    Imu imu(INERTIAL);

    L1.tare_position();
    L2.tare_position();
    L3.tare_position();
    R1.tare_position();
    R2.tare_position();
    R3.tare_position();


  
    delay(10);

     while (imu.is_calibrating()) {
        delay(20);
    }


    double currentPos = imu.get_rotation();
    double startPos = imu.get_rotation();
    double target = Angle - startPos;


    int startTime = millis();
    int currentTime = startTime;

    error = target - currentPos;
    
    // The enforced `minSpeed` combined with proportional gain `Kp` creates
    // a minimum achievable error roughly equal to (minSpeed / Kp).
    // Use the larger of the requested `leeway` and that achievable floor
    // so the loop can terminate instead of oscillating around that value.

   while ((currentTime - startTime < Timeout) && (fabs(error) > leeway))
    {
        currentTime = millis();

        double currentPos = imu.get_rotation();
        // compute shortest signed error across 0/360
        error = target - currentPos;

        // compute and clamp (apply min speed)
        double raw = Kp * error;
        double power = abscap(raw, maxSpeed);

        // Apply turn
        L1.move(power); L2.move(power); L3.move(power);
        R1.move(-power); R2.move(-power); R3.move(-power);

        printf("err: %.2f  pow: %.2f  Pos: %.2f\n", error, power, currentPos);

        delay(20);
    }

    // Stop motors at the end
    L1.move(0); L2.move(0); L3.move(0);
    R1.move(0); R2.move(0); R3.move(0);
}

/*


void baseTurn(double bearing, double kp)
{
    double turnL = bearing * TORAD * DIS;
    double turnR = -bearing * TORAD * DIS;
    baseGo(turnL, turnR, kp);
}


*/