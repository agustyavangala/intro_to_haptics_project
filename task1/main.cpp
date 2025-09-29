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

/*  ----- TASK1 PARAMETERS ----- */
double ROBOT_JOINT1_POSITION_DEG = 90;
double ROBOT_JOINT2_POSITION_DEG = 90;

/* ----------------------------- */

// simulation loop
void simulation(shared_ptr<SaiModel::SaiModel> robot);

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load graphics scene
	auto graphics = make_shared<SaiGraphics::SaiGraphics>(world_fname);
	graphics->addUIForceInteraction(robot_name);

	// load robots
	auto robot = make_shared<SaiModel::SaiModel>(robot_fname, false);

	// set initial condition
	Eigen::Vector2d initial_q;
	initial_q << ROBOT_JOINT1_POSITION_DEG/180.0*M_PI,
				ROBOT_JOINT2_POSITION_DEG/180.0*M_PI;
	robot->setQ(initial_q);
	robot->updateModel();
	// Eigen::Affine3d ee_trans;
	// robot->transform(ee_trans, ee_link_name);
	// cout << ee_trans.translation().transpose() << endl;
	// cout << ee_trans.rotation() << endl;

	// start the simulation
	fSimulationRunning = true;
	thread sim_thread(simulation, robot);

	// while window is open:
	while (graphics->isWindowOpen()) {
		graphics->updateRobotGraphics(robot_name, robot->q());
		graphics->renderGraphicsWorld();
	}

	// stop simulation and control
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

	// sim->setTimestep(1.0 / sim_freq);

	Eigen::MatrixXd Jbar;
	Eigen::MatrixXd Jv;
	Eigen::MatrixXd Jw;
	Eigen::MatrixXd J0;

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

		// ------------------------------------
		// FILL ME IN: set new joint velocities
		robot_dq << 0.0/180.0*M_PI,
					0.0/180.0*M_PI;
		robot->setDq(robot_dq);

		// ------------------------------------

		// update last time
		last_time = curr_time;
	}
}
