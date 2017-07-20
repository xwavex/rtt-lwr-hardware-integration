#include <rtt-lwr-robot.hpp>
#include <Eigen/Dense>

using namespace cogimon;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

void lwr_robot::updateHook() {
   //RTT::log(RTT::Info) << "Start Update" << RTT::endlog();
    if (!isRunning())
        return;


for(it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
        it->second->sense();

for(it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
        it->second->getCommand();

    for(it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
        it->second->move();

    this->trigger();
    //remote->doReceiveData();




   //RTT::log(RTT::Info) << "Done!" << RTT::endlog();
}

void lwr_robot::stopHook() {

for(it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
        it->second->stop();
}

