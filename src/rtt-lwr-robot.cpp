#include <rtt/Operation.hpp>
#include <rtt-lwr-robot.hpp>
#include <string>
#include <fstream>
#include <streambuf>

#include "pid_values_tmp.h"

using namespace cogimon;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

lwr_robot::lwr_robot(const std::string &name) :
		TaskContext(name), is_configured(false) {
//    this->provides("gazebo")->addOperation("WorldUpdateBegin",
//            &lwrSim::WorldUpdateBegin, this, RTT::ClientThread);
//    this->provides("gazebo")->addOperation("WorldUpdateEnd",
//            &lwrSim::WorldUpdateEnd, this, RTT::ClientThread);

	this->addOperation("getModel", &lwr_robot::getModel, this, ClientThread);

	this->addOperation("setControlMode", &lwr_robot::setControlMode, this,
			RTT::ClientThread);

	this->addOperation("getKinematicChains", &lwr_robot::getKinematicChains, this,
			RTT::ClientThread);

	this->addOperation("printKinematicChainInformation",
			&lwr_robot::printKinematicChainInformation, this, RTT::ClientThread);

	this->addOperation("getControlMode", &lwr_robot::getControlMode, this,
			RTT::ClientThread);

	this->addOperation("getAvailableControlMode",
			&lwr_robot::getControlAvailableMode, this, RTT::ClientThread);

	this->provides("joint_info")->addOperation("getJointMappingForPort",
			&lwr_robot::getJointMappingForPort, this, RTT::ClientThread);

//    world_begin = gazebo::event::Events::ConnectWorldUpdateBegin(
//            boost::bind(&lwrSim::WorldUpdateBegin, this));
//    world_end = gazebo::event::Events::ConnectWorldUpdateEnd(
//            boost::bind(&lwrSim::WorldUpdateEnd, this));
}

std::map<std::string, int> lwr_robot::getJointMappingForPort(
		std::string portName) {
	std::map<std::string, int> result;
	// find port in kinematic chain. Ports should be unique so no problem here!
	for (std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it =
			kinematic_chains.begin(); it != kinematic_chains.end(); it++) {
		std::vector<RTT::base::PortInterface*> interface =
				it->second->getAssociatedPorts();
		std::vector<base::PortInterface*>::iterator iiter;

		base::PortInterface* candidatePort = 0;

		for (iiter = interface.begin(); iiter != interface.end(); ++iiter) {
			if ((*iiter)->getName() == portName) {
				candidatePort = *iiter;
				break;
			}
		}

		if (candidatePort) {
			std::vector<std::string> jointNames = it->second->getJointNames();
			// assuming we take the index as stored in the vector...
			for (unsigned int i = 0; i < jointNames.size(); i++) {
				result[jointNames[i]] = i;
			}
			return result;
		}
	}
	return result;
}

std::string lwr_robot::printKinematicChainInformation(
		const std::string& kinematic_chain) {
	std::vector<std::string> chain_names = getKinematicChains();
	if (!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain)
			!= chain_names.end())) {
		log(Warning) << "Kinematic Chain " << kinematic_chain
				<< " is not available!" << endlog();
		return "";
	}

	return kinematic_chains[kinematic_chain]->printKinematicChainInformation();
}

std::string lwr_robot::getControlMode(const std::string& kinematic_chain) {
	std::vector<std::string> chain_names = getKinematicChains();
	if (!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain)
			!= chain_names.end())) {
		log(Warning) << "Kinematic Chain " << kinematic_chain
				<< " is not available!" << endlog();
		return "";
	}

	return kinematic_chains[kinematic_chain]->getCurrentControlMode();
}

std::vector<std::string> lwr_robot::getControlAvailableMode(
		const std::string& kinematic_chain) {
	std::vector<std::string> control_modes;

	std::vector<std::string> chain_names = getKinematicChains();
	if (!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain)
			!= chain_names.end())) {
		log(Warning) << "Kinematic Chain " << kinematic_chain
				<< " is not available!" << endlog();
		control_modes.push_back("");
	} else
		control_modes =
				kinematic_chains[kinematic_chain]->getControllersAvailable();
	return control_modes;
}

std::vector<std::string> lwr_robot::getKinematicChains() {
	std::vector<std::string> chains;
	for (std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it =
			kinematic_chains.begin(); it != kinematic_chains.end(); it++)
		chains.push_back(it->second->getKinematicChainName());
	return chains;
}

bool lwr_robot::setControlMode(const std::string& kinematic_chain,
		const std::string& controlMode) {
	std::vector<std::string> chain_names = getKinematicChains();
	if (!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain)
			!= chain_names.end())) {
		log(Warning) << "Kinematic Chain " << kinematic_chain
				<< " is not available!" << endlog();
		return false;
	}

	return kinematic_chains[kinematic_chain]->setControlMode(controlMode);
}

bool lwr_robot::getModel(const std::string& model_name) {
	if (model) {
		log(Warning) << "Model [" << model_name << "] already loaded !"
				<< endlog();
		return true;
	}
	model = urdf::parseURDFFile(model_name);
	return bool(model);
}

void lwr_robot::updateHook() {
}

bool lwr_robot::configureHook() {
	//this->is_configured = gazeboConfigureHook(model);

	//JOSH STUFF
	if (!bool(model)) {
		return false;
	}
	urdf_joints_ = model->joints_;
	urdf_links_ = model->links_;
	RTT::log(RTT::Info) << "Model name " << model->getName() << RTT::endlog();
	RTT::log(RTT::Info) << "Model has " << urdf_joints_.size() << " joints"
			<< RTT::endlog();
	RTT::log(RTT::Info) << "Model has " << urdf_links_.size() << " links"
			<< RTT::endlog();

	hardcoded_chains chains;
	std::map<std::string, std::pair<std::string,std::vector<std::string>>>::iterator it;
	for (it = chains.map_chains_joints.begin();
			it != chains.map_chains_joints.end(); it++) {
		kinematic_chains.insert(
				std::pair<std::string, boost::shared_ptr<KinematicChain>>(
						it->first,
						boost::shared_ptr<KinematicChain>(//TODO FRI INST
								new KinematicChain(it->first, it->second.second,
										*(this->ports()), new friRemote(49939, it->second.first, this->getActivity()->thread()->getTask())))));
	}
	RTT::log(RTT::Info) << "Kinematic Chains map created!" << RTT::endlog();
	for (std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it =
				kinematic_chains.begin(); it != kinematic_chains.end(); it++) {
			if (!(it->second->initKinematicChain())) {
				RTT::log(RTT::Warning) << "Problem Init Kinematic Chain"
						<< it->second->getKinematicChainName() << RTT::endlog();
				return false;
			}
		}
		RTT::log(RTT::Info) << "Kinematic Chains Initialized!" << RTT::endlog();

		RTT::log(RTT::Warning) << "Done configuring component" << RTT::endlog();
	return is_configured;
}



ORO_CREATE_COMPONENT_LIBRARY()
//ORO_CREATE_COMPONENT(cogimon::lwrSim)
ORO_LIST_COMPONENT_TYPE(cogimon::lwr_robot)

