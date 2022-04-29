#include "utilities.h"
#include <iostream>


void inversekinematics(double v_cmd, double w_cmd, double& vr, double& vl, double D) {

	vr = v_cmd + D * w_cmd / 2;
	vl = v_cmd - D * w_cmd / 2;

}

double pid(double kp, double kd, double ki, double e, double vr, double vl) {

	static double ei = 0.0;
	double dt = 0.01;

	if (abs(vr) > 100 || abs(vl) > 100) {

		ei = ei;
	}
	else
	{
		ei = ei + e * dt;
	}

	return kp * e + ki * ei;
}

void poscontroller(double xref, double yref, double x, double y, double theta, double& v_cmd, double& w_cmd) {

	double Kv, Kh, e_pos, e_angle;

	e_pos = sqrt(pow(x - xref, 2) + pow(y - yref, 2));
	e_angle = (atan2(yref - y, xref - x) - theta);

	std::cout << "\nep is " << e_pos << "\neangle is: " << e_angle;

	Kv = 0.5;
	Kh = 4;

	if (abs(e_angle) > 3.14159 / 2) {

		v_cmd = 0;
		w_cmd = Kh * e_angle;

	}
	else {
		v_cmd = Kv * e_pos;
		w_cmd = Kh * e_angle;

	}

	std::cout << "\nv is " << v_cmd << "\nw is " << w_cmd;

}

void purepursuit(double xref, double yref, double x, double y, double theta, double& v_cmd, double& w_cmd, double vr, double vl) {

	double kh = 4, kd = 0, ki = 0.005,kp = 2;
	double e_p, e_ang, d = 1;

	e_p = sqrt(pow(x - xref, 2) + pow(y- yref, 2)) - 1;
	e_ang = (atan2(yref - y, xref - x) - theta);

	if (abs(e_ang) > 3.14159 / 8) {

		v_cmd = 0;
		w_cmd = kh * e_ang;

	}
	else {
		v_cmd = pid(kp, kd, ki, e_p, vr, vl);
		w_cmd = kh * e_ang;

	}

}
