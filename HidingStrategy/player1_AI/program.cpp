
#include <cstdio>
#include <iostream>
#include <fstream>
#include <vector>
#include <Windows.h>

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#include "image_transfer.h"

// include this header file for computer vision functions
#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "timer.h"

extern robot_system S1;

// define a class to represent objects
// object_id_(1: self robot head, 2 : self robot rear
//            3: enemy head, 4: enemy rear
//            5: obstacles, 6: wheels)
class object {
public:
	object();
	object(double position_x, double position_y);
	// identify the object:
	// *input: rgb image, self robot colour, center of object
	// *return: object_id_
	int identify_object(image input_RGB_image,
		image label_map,
		int self_colour);
	bool calculate_robot_theta(object rear);
	double get_position_x();
	double get_position_y();
	double get_theta();
	int get_id();
	int get_label_value();
private:
	int object_id_, label_value_;
	double position_x_, position_y_, theta_;
};

object::object() {
	position_x_ = 0;
	position_y_ = 0;
	theta_ = 0;
}

object::object(double position_x, double position_y) :
	position_x_(position_x), position_y_(position_y) {};

int object::identify_object(image input_RGB_image,
							image label_map,
							int self_colour) {
	i2byte* p_label = (i2byte*)label_map.pdata;
	ibyte* p = input_RGB_image.pdata;
	int i = (int)position_x_;
	int j = (int)position_y_;
	//std::cout << "label: " << p_label[j * label_map.width + i] << std::endl;
	// calculate the position of pointer of object center
	int wheel_length = 4;
	ibyte* p_center = p + (i + input_RGB_image.width * j) * 3;
	ibyte* p_left = p + (i - wheel_length + input_RGB_image.width * j) * 3;
	ibyte* p_right = p + (i + wheel_length + input_RGB_image.width * j) * 3;
	ibyte* p_up = p + (i + input_RGB_image.width * (j + wheel_length)) * 3;
	ibyte* p_down = p + (i + input_RGB_image.width * (j - wheel_length)) * 3;
	// get the colour of object center
	if (i> input_RGB_image.width-16 || i < 16 ||
		j> input_RGB_image.height - 16 || j<16) {
		std::cout << "out of sight!----------------------" << std::endl;
		return 0;
	}
	ibyte B = *p_center;
	ibyte G = *(p_center + 1);
	ibyte R = *(p_center + 2);
	// get the colour at four directions larger than robot's wheel's size
	ibyte B_left = *p_left;
	ibyte B_right = *p_right;
	ibyte B_down = *p_down;
	ibyte B_up = *p_up;
	// check if is still the same colour at four directions
	// larger than robot's size
	int max_error = 15;
	int left_error = abs(B_left - B);
	int right_error = abs(B_right - B);
	int up_error = abs(B_up - B);
	int down_error = abs(B_down - B);
	//std::cout << "errors: " << left_error <<" "
	//						<< right_error << " "
	//						<< up_error << " "
	//						<< down_error << std::endl;
	if (left_error > max_error || right_error > max_error ||
		up_error > max_error || down_error > max_error) {
		//std::cout << "found a wheel" << std::endl;
		object_id_ = 6;
		label_value_ = p_label[j * label_map.width + i];
		return 6;
	}
	// get the colour at four directions larger than robot's size
	int robot_radius = 15;
	p_left = p + (i - robot_radius + input_RGB_image.width * j) * 3;
	p_right = p + (i + robot_radius + input_RGB_image.width * j) * 3;
	p_up = p + (i + input_RGB_image.width * (j + robot_radius)) * 3;
	p_down = p + (i + input_RGB_image.width * (j - robot_radius)) * 3;
	B_left = *p_left;
	B_right = *p_right;
	B_down = *p_down;
	B_up = *p_up;
	left_error = abs(B_left - B);
	right_error = abs(B_right - B);
	up_error = abs(B_up - B);
	down_error = abs(B_down - B);
	//std::cout << "errors: " << left_error << " "
	//	<< right_error << " "
	//	<< up_error << " "
	//	<< down_error << std::endl;
	// check if is still the same color at four directions
	// larger than robot's size
	if (self_colour == 1) {
		if (left_error > max_error || right_error > max_error ||
			up_error > max_error || down_error > max_error) {
			// check the colour of center to determine is self or enemy
			if (R > 200) {
				if (G > 150) { 
					object_id_ = 3; 
					label_value_ = p_label[j * label_map.width + i + robot_radius];
					return 3; 
				}
				else { 
					object_id_ = 2;
					label_value_ = p_label[j * label_map.width + i + robot_radius]; 
					return 2; }
			}
			else if (B > 200) { object_id_ = 4; 
			label_value_ = p_label[j * label_map.width + i + robot_radius]; 
			return 4; }
			else { object_id_ = 1; 
			label_value_ = p_label[j * label_map.width + i + robot_radius]; 
			return 1; }
		}
	}
	else {
		if (left_error > max_error || right_error > max_error ||
			up_error > max_error || down_error > max_error) {
			// check the colour of center to determine is self or enemy
			if (R > 200) {
				if (G > 150) { object_id_ = 1; label_value_ = p_label[j * label_map.width + i + robot_radius]; return 1; }
				else { object_id_ = 4; label_value_ = p_label[j * label_map.width + i + robot_radius]; return 4; }
			}
			else if (B > 200) { object_id_ = 2; label_value_ = p_label[j * label_map.width + i + robot_radius]; return 2; }
			else { object_id_ = 3; label_value_ = p_label[j * label_map.width + i + robot_radius]; return 3; }
		}
	}
	//std::cout << "found an obstacle" << std::endl;
	object_id_ = 5;
	//label_value_ = p_label[j * label_map.width + i];
	return 5;
}

bool object::calculate_robot_theta(object rear) {
	double rear_x = rear.get_position_x();
	double rear_y = rear.get_position_y();
	double theta;
	/*std::cout << " rear_x: " << rear_x << std::endl;
	std::cout << " rear_y: " << rear_y << std::endl;
	std::cout << " x: " << position_x_ << std::endl;
	std::cout << " y: " << position_y_ << std::endl;
	std::cout << " position_y_ - rear_y: " << (position_y_ - rear_y) << std::endl;
	if ((position_y_ - rear_y) < 0) {
		theta = -atan2((rear_y - position_y_), (position_x_ - rear_x));
	}
	else{
		theta = atan2((position_y_ - rear_y), (position_x_ - rear_x));
	}*/
	theta = atan2((position_y_ - rear_y), (position_x_ - rear_x));
	theta_ = theta;
	return true;
}

double object::get_position_x() {
	return position_x_;
}

double object::get_position_y() {
	return position_y_;
}

double object::get_theta() {
	return theta_;
}

int object::get_id() {
	return object_id_;
}

int object::get_label_value() {
	return label_value_;
}

// get positions of two robots and map which includes all obstacles,
// return the number of found objects
int get_positions_from_image(image rgb, int self_colour,
	std::vector<object> &objects, image &label_map) {
	image temp_image, map;
	temp_image.type = GREY_IMAGE;
	temp_image.width = rgb.width;
	temp_image.height = rgb.height;
	map.type = GREY_IMAGE;
	map.width = rgb.width;
	map.height = rgb.height;
	allocate_image(map);
	allocate_image(temp_image);
	copy(rgb, temp_image);
	map.type = GREY_IMAGE;
	scale(temp_image, map);
	lowpass_filter(map, temp_image);
	threshold(temp_image, map, 180);
	invert(map, temp_image);
	erode(temp_image, map); // denoise
	dialate(map, temp_image);
	//copy(temp_image, rgb);
	//view_rgb_image(rgb);
	//pause();
	// label objects
	int nlabel;
	label_image(temp_image, label_map, nlabel);
	std::cout << "found " << nlabel << " objects" << std::endl;
	double ic[9], jc[9];
	for (int i = 1; i <= nlabel; i++) {
		centroid(temp_image, label_map, i, ic[i], jc[i]);
		object obj(ic[i], jc[i]);
		//std::cout << "object " << i << " position: " << ic[i] << " " << jc[i] << std::endl;
		objects.push_back(obj);
		objects[i-1].identify_object(rgb, label_map, self_colour);
		std::cout << "id: x,y,label: " << objects[i - 1].get_id() << ": " <<
			ic[i] << " " << jc[i] << " " << objects[i - 1].get_label_value()<< std::endl;
		//draw_point_rgb(rgb, ic[i], jc[i], 255, 0, 0);
	}

	free_image(temp_image);
	free_image(map);
	return true;
}

// check whether a pixel is part of obstacls
// return true if it is free
bool check_space(std::vector<object> objects, int i, int j) {
	double obstacle_radius = 50;
	for (int k = 0; k < objects.size(); k++) {
		if (objects[k].get_id() == 5) {
			double distance = sqrt((i - objects[k].get_position_x()) * (i - objects[k].get_position_x())+
				(j - objects[k].get_position_y()) * (j - objects[k].get_position_y()));
			if (distance < obstacle_radius) {
				return false;
			}
		}
	}
	return true;
}

// find out objects represent self and enemy robots from all objects
bool get_robots(std::vector<object> objects, object& self, object& self_rear,
				object& enemy, object& enemy_rear) {
	for (int i = 0; i < objects.size(); i++) {
		if (objects[i].get_id() == 1) {
			self = objects[i];
		}
		if (objects[i].get_id() == 3) {
			enemy = objects[i];
		}
		if (objects[i].get_id() == 2) {
			self_rear = objects[i];
		}
		if (objects[i].get_id() == 4) {
			enemy_rear = objects[i];
		}
	}
	return true;
}

// locate the hiding point according to the enemy position and 
// the specific obstacle position
bool calculate_hiding_point(double self_x, double self_y,
	double enemy_x, double enemy_y,
	double obstacle_x, double obstacle_y,
	double &hiding_point_x, double &hiding_point_y) {
	double dx = obstacle_x - enemy_x;
	double dy = obstacle_y - enemy_y;
	double k = ((self_x - obstacle_x) * dx + (self_y - obstacle_y) * dy)
		/ ((dx * dx) + (dy * dy));
	hiding_point_x = obstacle_x + k * dx;
	hiding_point_y = obstacle_y + k * dy;
	return true;
}

// calculate the distance between two points
double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

bool calculate_expected_position(double self_x, double self_y, double self_theta,
	double enemy_x, double enemy_y, double enemy_theta, std::vector<object> objects,
	double &expected_x, double &expected_y, double &expected_theta) {
	std::vector<double> points_x, points_y;
	for (int i = 0; i < objects.size(); i++) {
		if (objects[i].get_id() == 5) {
			double point_x, point_y;
			calculate_hiding_point(self_x, self_y, enemy_x, enemy_y,
				objects[i].get_position_x(), objects[i].get_position_y(),
				point_x, point_y);
			points_x.push_back(point_x);
			points_y.push_back(point_y);
		}
	}
	double min_distance=2000.0;
	for (int i = 0; i < points_x.size(); i++) {
		double distance_ = distance(points_x[i], points_y[i], self_x, self_y);
		std::cout << "distance: " << distance_ << std::endl;
		if (distance_ < min_distance) {
			min_distance = distance_;
			expected_x = points_x[i];
			expected_y = points_y[i];
			std::cout << "hiding_point: " << points_x[i] << ", " << points_y[i] << std::endl;
			std::cout << "distance: " << distance_ << std::endl;
		}
	}
	if (enemy_theta > 0) {
		expected_theta = enemy_theta - 3.14;
		return true;
	}
	expected_theta = enemy_theta + 3.14;
	return true;
}

int main()
{
	double x0, y0, theta0, max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int N_obs, n_robot;
	double x_obs[50], y_obs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	int mode, level;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;

	// TODO: it might be better to put this model initialization
	// section in a separate function

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width1 = 640;
	height1 = 480;

	// number of obstacles
	N_obs = 2;

	x_obs[1] = 300; // pixels
	y_obs[1] = 200; // pixels
	size_obs[1] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	x_obs[2] = 400; // pixels
	y_obs[2] = 300; // pixels
	size_obs[2] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	// set robot model parameters ////////

	D = 121.0; // distance between front wheels (pixels)

	// position of laser in local robot coordinates (pixels)
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction		
	Lx = 31.0;
	Ly = 0.0;

	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;

	alpha_max = 3.14159 / 2; // max range of laser / gripper (rad)

	// number of robot (1 - no opponent, 2 - with opponent, 3 - not implemented yet)
	n_robot = 2;

	std::cout << "\npress space key to begin program.";
	pause();

	// you need to activate the regular vision library before 
	// activating the vision simulation library
	activate_vision();

	// note it's assumed that the robot points upware in its bmp file

	// however, Lx, Ly, Ax, Ay assume robot image has already been
	// rotated 90 deg so that the robot is pointing in the x-direction
	// -- ie when specifying these parameters assume the robot
	// is pointing in the x-direction.

	// note that the robot opponent is not currently implemented in 
	// the library, but it will be implemented soon.

	activate_simulation(width1, height1, x_obs, y_obs, size_obs, N_obs,
		"robot_A.bmp", "robot_B.bmp", "background.bmp", "obstacle_blue.bmp", D, Lx, Ly,
		Ax, Ay, alpha_max, n_robot);

	// open an output file if needed for testing or plotting
//	ofstream fout("sim1.txt");
//	fout << scientific;

	// set simulation mode (level is currently not implemented)
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	mode = 1;
	level = 1;
	set_simulation_mode(mode, level);

	// set robot initial position (pixels) and angle (rad)
	x0 = 150;
	y0 = 350;
	theta0 = 3;
	set_robot_position(x0, y0, theta0);

	// set initial inputs / on-line adjustable parameters /////////

	// inputs
	// pw_l -- pulse width of left servo (us) (from 1000 to 2000)
	// pw_r -- pulse width of right servo (us) (from 1000 to 2000)
	pw_l = 1500; // pulse width for left wheel servo (us)
	pw_r = 1500; // pulse width for right wheel servo (us)
	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)

	// paramaters
	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)

	// lighting parameters (not currently implemented in the library)
	light = 1.0;
	light_gradient = 1.0;
	light_dir = 1.0;
	image_noise = 1.0;

	// set initial inputs
	set_inputs(pw_l, pw_r, pw_laser, laser,
		light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);

	image rgb;
	int height, width;

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width = 640;
	height = 480;

	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;

	// allocate memory for the images
	allocate_image(rgb);

	wait_for_player();

	// measure initial clock time
	tc0 = high_resolution_time();
	std::cout << "Start" << std::endl;

	while (1) {

		// simulates the robots and acquires the image from simulation
		acquire_image_sim(rgb);

		tc = high_resolution_time() - tc0;

		// change the inputs to move the robot around
		// or change some additional parameters (lighting, etc.)
		// only the following inputs work so far
		// pw_l -- pulse width of left servo (us) (from 1000 to 2000)
		// pw_r -- pulse width of right servo (us) (from 1000 to 2000)
		// pw_laser -- pulse width of laser servo (us) (from 1000 to 2000)
		// -- 1000 -> -90 deg
		// -- 1500 -> 0 deg
		// -- 2000 -> 90 deg
		// laser -- (0 - laser off, 1 - fire laser for 3 s)
		// max_speed -- pixels/s for right and left wheels
		set_inputs(pw_l, pw_r, pw_laser, laser,
			light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);

		//   Image processing --------------------------------------------------

		// The colour of our own robot which should be known, 1 represents A
		// 2 represents B
		int self_colour = 1;
		std::vector<object> objects;
		image label_map;
		label_map.type = LABEL_IMAGE;
		label_map.width = rgb.width;
		label_map.height = rgb.height;
		allocate_image(label_map);

		int nlabel = get_positions_from_image(rgb, self_colour, objects, 
											label_map);

		object self,self_rear,enemy,enemy_rear;
		get_robots(objects,self,self_rear,enemy,enemy_rear);
		self.calculate_robot_theta(self_rear);
		enemy.calculate_robot_theta(enemy_rear);

		double self_position_x, self_position_y, self_position_theta,
			enemy_position_x, enemy_position_y, enemy_position_theta;

		self_position_x = self.get_position_x();
		self_position_y = self.get_position_y();
		self_position_theta = self.get_theta();
		enemy_position_x = enemy.get_position_x();
		enemy_position_y = enemy.get_position_y();
		enemy_position_theta = enemy.get_theta();
		// print the output
		std::cout << "self_position: " << self_position_x << ", "
			<< self_position_y << ", "
			<< self_position_theta 
			<< ", " << std::endl;
		std::cout << "enemy_position: " << enemy_position_x << ", "
			<< enemy_position_y << ", "
			<< enemy_position_theta 
			<< ", " << std::endl;

		//draw_point_rgb(rgb, self_position_x, self_position_y, 0, 0, 255);
		//draw_point_rgb(rgb, enemy_position_x, enemy_position_y, 0, 255, 0);

		//std::cout << "is 100, 100 free? "
			//<< check_space(objects, 320, 400) << std::endl;
		//std::cout << "is 300, 200 free? "
			//<< check_space(objects, 300, 200) << std::endl;

		// Image processing done ---------------------------------------------

		// Hiding Strategy ---------------------------------------------------
		// input: self state, enemy state, all obstacles
		// output: expected state
		double expected_x, expected_y, expected_theta;
		calculate_expected_position(self_position_x, self_position_y, self_position_theta,
			enemy_position_x, enemy_position_y, enemy_position_theta, objects,
			expected_x, expected_y, expected_theta);
		draw_point_rgb(rgb, expected_x, expected_y, 255, 0, 0);
		// Hiding Strategy done ---------------------------------------------------


		// NOTE: only one program can call view_image()
		view_rgb_image(rgb);

		// don't need to simulate too fast
		Sleep(10); // 100 fps max
	}

	// free the image memory before the program completes
	free_image(rgb);

	deactivate_vision();

	deactivate_simulation();

	std::cout << "\ndone.\n";

	return 0;
}
