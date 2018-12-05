/*  task4
This file includes the required code to implement task4: constrained teleoperation with haptic device.

Author: Shameek Ganguly shameekg@stanford.edu
Date: 12/3/18
*/

#include <iostream>
#include <string>
#include <thread>
#include <math.h>

#include "model/ModelInterface.h"
#include "graphics/GraphicsInterface.h"
#include "graphics/ChaiGraphics.h"
#include "chai3d.h"

#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;
using namespace chai3d;

const string world_fname = "resources/task4/world.urdf";
const string robot_fname = "resources/task4/RR.urdf";
const string robot_name = "RRBot";
const string camera_name = "camera_top";
const string ee_link_name = "link1";

/*  ----- TASK4 Variables ----- */
bool fHapticDeviceEnabled = false;
Eigen::Vector3d haptic_device_velocity = Eigen::Vector3d::Zero();
Eigen::Vector3d F_haptic = Eigen::Vector3d::Zero();
/* ----------------------------- */

// simulation loop
bool fSimulationRunning = false;
void simulation(Model::ModelInterface* robot);
void haptic(cGenericHapticDevicePtr device);

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load graphics scene
	auto graphics_ext = new Graphics::GraphicsInterface(world_fname, Graphics::chai, Graphics::urdf, false);
	Graphics::ChaiGraphics* graphics = dynamic_cast<Graphics::ChaiGraphics*> (graphics_ext->_graphics_internal);

	// load robots
	auto robot = new Model::ModelInterface(robot_fname, Model::rbdl, Model::urdf, false);

	// set initial condition
	robot->_q << 0.0/180.0*M_PI,
				90.0/180.0*M_PI;
	robot->updateModel();
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

	// initialize the haptic device
	auto handler = new cHapticDeviceHandler();
	cGenericHapticDevicePtr hapticDevice;
	handler->getDevice(hapticDevice, 0);
	if (NULL == hapticDevice) {
		cerr << "No haptic device found. " << endl;
		fHapticDeviceEnabled = false;
	} else {
		cout << "Haptic device initialized and ready. " << endl;
		hapticDevice->open();
		hapticDevice->calibrate();
		fHapticDeviceEnabled = true;
		// if the device has a gripper, enable the gripper to simulate a user switch
	    hapticDevice->setEnableGripperUserSwitch(true);
	}

	// set a line to visualize the applied force
	auto haptic_force_line = new cShapeLine();
    haptic_force_line->setShowEnabled(false);
    haptic_force_line->setLineWidth(4.0);
    if (fHapticDeviceEnabled) {
    	graphics->_world->addChild(haptic_force_line);
    }

	thread haptics_thread(haptic, hapticDevice);

    // while window is open:
    Eigen::Vector3d robot_eff_pos;
    robot_eff_pos.setZero();
    while (!glfwWindowShouldClose(window)) {
		// update haptic display line
		robot->position(robot_eff_pos, "link1", Eigen::Vector3d(1.0, 0.0, 0.0));
		haptic_force_line->m_pointA = robot_eff_pos;
		haptic_force_line->m_pointB = chai3d::cVector3d(robot_eff_pos + 0.1*F_haptic);
		haptic_force_line->m_pointB.z(0.0);
		haptic_force_line->setShowEnabled(true);


		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	haptics_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(Model::ModelInterface* robot) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(500); //500Hz timer
	double last_time = timer.elapsedTime(); //secs

	double vhx = 0.0;
	double vhy = 0.0;
	double vhz = 0.0;
	double q0 = 0.0;
	double q1 = 0.0;
	double dq0 = 0.0;
	double dq1 = 0.0;

	double vxe = 0.0;
	double vye = 0.0;

	Eigen::Matrix2d J;
	J.setZero();

	double scale = 50.0;

	double fxh = 0.0;
	double fyh = 0.0;

	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// integrate joint velocity to joint positions
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		robot->_q += robot->_dq*loop_dt;

		// if (!fTimerDidSleep) {
		// 	cout << "Warning: timer underflow! dt: " << loop_dt << "\n";
		// }

		// update kinematic models
		robot->updateModel();
		
		// get q0, q1
		q0 = robot->_q[0]; // in radians
		q1 = robot->_q[1]; // in radians

		// get haptic device velocity
		vhx = haptic_device_velocity[0];
		vhy = haptic_device_velocity[1];
		vhz = haptic_device_velocity[2];
		
		// get vxe and vye from haptic device
		vxe = haptic_device_velocity[0] * scale;
		vye = haptic_device_velocity[1] * scale;
		
		J << -sin(q0) - sin(q0+q1), - sin(q0+q1),
		cos(q0) + cos(q0+q1), cos(q0+q1); 

		Eigen::Vector2d dq_vec = J.inverse() * Eigen::Vector2d(vxe, vye);
		dq0 = dq_vec[0];
		dq1 = dq_vec[1];

		// ------------------------------------
		// FILL ME IN: set force on haptic device, fx and fy
		fxh = 0.0;
		fyh = 0.0;

		// ------------------------------------
		F_haptic << fxh, fyh;

		// ------------------------------------

		robot->_dq << dq0, dq1;
		
		// ------------------------------------

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
void haptic(cGenericHapticDevicePtr device) {
	cVector3d device_vel;
	while (fHapticDeviceEnabled && fSimulationRunning) {
		device->getLinearVelocity(device_vel);
		haptic_device_velocity = device_vel.eigen();
		// cout << haptic_device_velocity << endl;
		device->setForce(F_haptic);
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
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - Task 4", NULL, NULL);
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
}
