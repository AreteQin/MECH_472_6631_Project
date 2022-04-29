#pragma once


void inversekinematics(double v_cmd, double w_cmd, double& vr, double& vl, double D);

double pid(double kp, double kd, double ki, double e, double vr, double vl);

void poscontroller(double xref, double yref, double x, double y, double theta, double& v_cmd, double& w_cmd);

void purepursuit(double xref, double yref, double x, double y, double theta, double& v_cmd, double& w_cmd, double vr, double vl);
