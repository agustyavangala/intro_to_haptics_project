/*  task4
This file includes the required code to implement task4: move planar mechanism with haptic device.

Author: Shameek Ganguly shameekg@stanford.edu
Date: 11/26/18
*/

#include <iostream>
#include <string>
#include <thread>
#include <math.h>

#include "SaiModel.h"
#include "SaiGraphics.h"
#include "chai3d.h"

#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

// for handling ctrl+c and interruptions properly
#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }

// namespaces for compactness of code
using namespace std;
using namespace Eigen;
using namespace chai3d;

const string world_fname = "resources/task3/world.urdf";
const string robot_fname = "resources/task3/RR.urdf";
const string robot_name = "RRBot";
const string camera_name = "camera_top";
const string ee_link_name = "link1";

/*  ----- TASK3 Variables ----- */
bool fHapticDeviceEnabled = false;
Eigen::Vector3d haptic_device_velocity = Eigen::Vector3d::Zero();
/*  ----- TASK4 Variables ----- */
Eigen::Vector3d F_haptic = Eigen::Vector3d::Zero();
/* ----------------------------- */

/* ----------------------------- */

// simulation loop
void simulation(shared_ptr<SaiModel::SaiModel> robot);
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

	thread haptics_thread(haptic, hapticDevice);

    // while window is open:
	while (graphics->isWindowOpen()) {
		graphics->updateRobotGraphics(robot_name, robot->q());
		graphics->renderGraphicsWorld();
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	haptics_thread.join();

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

	double vhx = 0.0;
	double vhy = 0.0;
	double vhz = 0.0;
	double q0 = 0.0;
	double q1 = 0.0;
	double dq0 = 0.0;
	double dq1 = 0.0;

	double scale = 10.0;

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
		
		// get q0, q1
		q0 = robot->q()[0]; // in radians
		q1 = robot->q()[1]; // in radians

		// get haptic device velocity
		vhx = haptic_device_velocity[0];
		vhy = haptic_device_velocity[1];
		vhz = haptic_device_velocity[2];
		
		// scale the velocity
		double vxe = vhx * scale;
		double vye = vhy * scale;

		// ------------------------------------
		// Dq0 = cos(q0+q1)vxe+sin(q0+q1)vye/ sin(q1)
		// Dq1 = - ((cos q0 + cos(q0+q1))vxe + (sin(q0)+sin(q0+q1))vye)/sin(q1)
		// FILL ME IN: set new joint velocities given vxe, vye, q0 and q1
		// NOTE: These should be entered in radians/second. Not in degrees/second.
		dq0 = ((cos(q0 + q1) * vxe )+ (sin(q0 + q1) * vye)) / sin(q1);
		dq1 = -((cos(q0) + cos(q0 + q1)) * vxe + (sin(q0) + sin(q0 + q1)) * vye) / sin(q1);		
		// ------------------------------------
		// FILL ME IN: set new joint velocities given vxe, vye, q0 and q1
		// NOTE: These should be entered in radians/second. Not in degrees/second.
		

		// ------------------------------------

		robot_dq << dq0, dq1;
		robot->setDq(robot_dq);
		
		// ------------------------------------

		//  Fhaptic = -p/||p|| * 1/(R- ||p||)^k c^k0 +c1------------------------------------
        // FILL ME IN: set force on haptic device, fx and fy
        //p = [xe, ye, 0] is the 3D position of the robot’s hand
        //||p|| = the distance of the hand from the origin given by square-root of (xe*xe + ye*ye)
        //R = 2 is the radius of the robot’s workspace in distance units.
        //k is an exponent parameter that determines how the force increases as the robot’s hand gets closer to the workspace boundary (Hint: you can try different values  between 1 and 10)
        //c0 and c1 are positive valued parameters that control the scaling of the force. If c0 is too large, the force will decrease quickly as you move the robot’s hand away from the workspace boundary. If c1 is too large, the maximum strength of the force felt (when the robot’s hand is at the workspace boundary) is reduced. (Hint: try values for c0 between 1.0 to 10.0, try values for c1 between 0.01 to 1.0)

        double xe, ye;
		double fxh = 0.0;
		double fyh = 0.0;
        //Xe = f(q0,q1) = cos(q0) + cos(q0+q1)
        //Ye = g(q0,q1) = sin(q0) + sin(q0+q1) 
        xe = cos(q0) + cos(q0+q1);
        ye = sin(q0) + sin(q0+q1);

        double p_norm = sqrt(xe*xe + ye*ye);
        double R = 2.0;
        double k = 3.0;
        double c0 = 0.5;
        double c1 = 0.5;       
        if (p_norm < R) {
            fxh = - (xe/p_norm) * (1.0/pow((R - p_norm), k)) * pow(c0, k) + c1;
            fyh = - (ye/p_norm) * (1.0/pow((R - p_norm), k)) * pow(c0, k) + c1;
        } else {
            //if the robot's hand is outside the workspace, set the force to zero
            fxh = 0.0;
            fyh = 0.0;
        }
    
        // ------------------------------------
        F_haptic << fxh, fyh, 0.0;


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
		device->setForce(F_haptic);
		//cout << haptic_device_velocity << endl << endl;
	}
}
