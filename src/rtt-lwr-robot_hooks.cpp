#include <rtt-lwr-robot.hpp>
#include <Eigen/Dense>

using namespace cogimon;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

void lwr_robot::updateHook() {
	// check if the component is stopped, then do not execute the updateHook anymore.
	if (!isRunning())
		return;

	for (it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
		it->second->sense(); // use return value to see if there is a connection!

	for (it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
		it->second->getCommand();

	for (it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
		it->second->move();

	// execution action is called to trigger this updateHook again.
	this->trigger();
}

void lwr_robot::stopHook() {
	// redirect the stop signal to the kinematic-chains to handle the actual stop.
	for (it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
		it->second->stop();
}

