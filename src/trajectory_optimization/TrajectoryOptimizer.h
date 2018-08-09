// TrajectoryOptimizer.h
#ifndef TRAJECTORY_OPTIMIZER_H
#define TRAJECTORY_OPTIMIZER_H

#include "ParametrizedTrajectory.h"
#include "system_model/SystemModel.h"



// class to encapsulate general trajectory optimization capability based on 
// an implicit system model that involves a controller and a simulation interface
class TrajectoryOptimizer {
public:
	// constructor
	TrajectoryOptimizer()
	: _model(NULL), _curr_trajectory(NULL)
	{
		// nothing to do
	}

	// model set. Model is mutable 
	virtual void modelIs (SystemModel* real_model, SystemModel* sim_model);

	// TODO: set partial gradient of function if available

	// one step of gradient descent
	// If input trajectory is NULL, the trajectory stored from the last step is used
	// RETURNs the current objective value. Objective is -1 if no feasible
	// trajectory was found
	virtual double stepGradDescent (const ParametrizedTrajectory& init_traj=NULL);

	// get best trajectory so far
	// RETURNs NULL if no successful trajectory was found so far
	const ParametrizedTrajectory& trajectory () const;

private:
	// real system model. private since we don't want the client to accidentally 
	// change the system model in the middle of the optimization without us realizing
	SystemModel* _real_model;

	// sim system model. this is reset to the initial state several times during the
	// optimization. So it needs to be a mutable.
	SystemModel* _sim_model;

	// most updated trajectory since the system model was updated. 
	// NULL if a successful trajectory has yet to be computed.
	ParametrizedTrajectory _curr_trajectory;
};

#endif //TRAJECTORY_OPTIMIZER_H