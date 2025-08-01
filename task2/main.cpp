/*  task1
This file includes the required code to implement task1: compute joint angles for 2dof
planar mechanism.

Author: Shameek Ganguly shameekg@stanford.edu
Date: 8/7/18
*/

#include <iostream>
#include <string>
#include <thread>
#include <math.h>

#include "SaiModel.h"
#include "SaiGraphics.h"

#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

// for handling ctrl+c and interruptions properly
#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }

// namespaces for compactness of code
using namespace std;
using namespace Eigen;

const string world_fname = "resources/task1/world.urdf";
const string robot_fname = "resources/task1/RR.urdf";
const string robot_name = "RRBot";
const string camera_name = "camera_top";
const string ee_link_name = "link1";

/*  ----- TASK2 FLAGS ----- */
bool f_x_minus = false;
bool f_x_plus = false;
bool f_y_minus = false;
bool f_y_plus = false;
bool f_xy_stop = false;

/* ----------------------------- */

// simulation loop
void simulation(shared_ptr<SaiModel::SaiModel> robot);

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load graphics scene
	auto graphics = make_shared<SaiGraphics::SaiGraphics>(world_fname);
	graphics->addUIForceInteraction(robot_name);

	// load robots
	auto robot = make_shared<SaiModel::SaiModel>(robot_fname, false);

	// set initial condition
	Eigen::Vector2d initial_q;
	initial_q << 0.0/180.0*M_PI,
				90.0/180.0*M_PI;
	robot->setQ(initial_q);
	// Eigen::Affine3d ee_trans;
	// robot->transform(ee_trans, ee_link_name);
	// cout << ee_trans.translation().transpose() << endl;
	// cout << ee_trans.rotation() << endl;

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// start the simulation
	thread sim_thread(simulation, robot);
	
	// while window is open:
	// while (graphics->isWindowOpen()) {
	while (!glfwWindowShouldClose(window)) {
		graphics->updateRobotGraphics(robot_name, robot->q());
		graphics->renderGraphicsWorld();
		
	    // poll for events
		glfwSwapBuffers(window);
	    glfwPollEvents();
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(shared_ptr<SaiModel::SaiModel> robot) {
	
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(500); //500Hz timer
	double last_time = timer.elapsedTime(); //secs

	double vxe = 0.0;
	double vye = 0.0;
	double q0 = 0.0;
	double q1 = 0.0;
	double dq0 = 0.0;
	double dq1 = 0.0;

	// TODO: make sure this reads initial q values set in main function
	Eigen::VectorXd robot_q = robot->q();
	Eigen::VectorXd robot_dq = robot->dq();

	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// integrate joint velocity to joint positions
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		robot_q += robot->dq() * loop_dt;
		robot->setQ(robot_q);

		// if (!fTimerDidSleep) {
		// 	cout << "Warning: timer underflow! dt: " << loop_dt << "\n";
		// }

		// update kinematic models
		robot->updateModel();

		// handle keyboard keys
		if (f_xy_stop) {
			vxe = 0.0;
			vye = 0.0;
			f_xy_stop = false;
		}
		if (f_x_minus) {
			vxe -= 0.01;
			f_x_minus = false;
		}
		if (f_x_plus) {
			vxe += 0.01;
			f_x_plus = false;
		}
		if (f_y_minus) {
			vye -= 0.01;
			f_y_minus = false;
		}
		if (f_y_plus) {
			vye += 0.01;
			f_y_plus = false;
		}
		
		// get q0, q1
		q0 = robot->q()(0); // in radians
		q1 = robot->q()(1); // in radians

		// ------------------------------------
		// FILL ME IN: set new joint velocities given vxe, vye, q0 and q1
		// NOTE: These should be entered in radians/second. Not in degrees/second.
		dq0 = 0.0;
		dq1 = 0.0;

		// ------------------------------------

		robot_dq << dq0, dq1;
		robot->setDq(robot_dq);
		
		// ------------------------------------

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
GLFWwindow* glfwInitialize() {
		/*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "Sai.0 - Task 2", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods) {
    // option ESC: exit
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        // exit application
         glfwSetWindowShouldClose(window, 1);
    }

    // up
    if (key == GLFW_KEY_UP && action == GLFW_PRESS) {
    	f_x_minus = true;
    }
    // down
    if (key == GLFW_KEY_DOWN && action == GLFW_PRESS) {
    	f_x_plus = true;
    }
    // left
    if (key == GLFW_KEY_LEFT && action == GLFW_PRESS) {
    	f_y_minus = true;
    }
    // right
    if (key == GLFW_KEY_RIGHT && action == GLFW_PRESS) {
    	f_y_plus = true;
    }
    // space
    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
    	f_xy_stop = true;
    }
}

