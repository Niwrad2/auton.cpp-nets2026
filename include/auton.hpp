#pragma once
#define INPERDEG 0.01701696




#define DIS 4.82258
#define TORAD 0.01745329251

void baseGo(double L, double R, double p_KP, double p_KD, double LEEWAY, double TimeOut, double MaxSpeed);
void gyroTurn(double Angle, double Kp, double leeway, int Timeout, double maxSpeed);