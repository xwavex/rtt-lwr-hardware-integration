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

	this->addOperation("getKinematicChains", &lwr_robot::getKinematicChains,
			this, RTT::ClientThread);

	this->addOperation("printKinematicChainInformation",
			&lwr_robot::printKinematicChainInformation, this,
			RTT::ClientThread);

	this->addOperation("getControlMode", &lwr_robot::getControlMode, this,
			RTT::ClientThread);

	this->addOperation("getAvailableControlMode",
			&lwr_robot::getControlAvailableMode, this, RTT::ClientThread);
	this->addOperation("loadURDFAndSRDF", &lwr_robot::loadURDFAndSRDF, this,
			RTT::ClientThread);

	this->addOperation("reset_model_configuration",
			&lwr_robot::resetModelConfiguration, this, RTT::ClientThread);

	//assign ip address of this computer
	this->ip_addr = "192.168.0.51";
	addProperty("ip_addr", ip_addr).doc("IP address of the computer");

	_models_loaded = false;
}
/*
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
 }*/

bool lwr_robot::resetModelConfiguration() {
	bool reset = true;
	std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it;
	for (it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
		reset = reset && it->second->resetKinematicChain();
	return reset;
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
	//parse URDF rather than getting model from gazebo
	model = urdf::parseURDFFile(model_name);
	return bool(model);
}

bool lwr_robot::configureHook() {
	//might not need the model stuff for this component or maybe have it in this component and provide to everything else?
	if (!_models_loaded || !bool(model)) {
		RTT::log(RTT::Error)
				<< "URDF and SRDF models has not been passed. Call loadURDFAndSRDF(URDF_path, SRDF_path)"
				<< RTT::endlog();
		return false;
	}
	urdf_joints_ = model->joints_;
	urdf_links_ = model->links_;
	RTT::log(RTT::Info) << "Model name " << model->getName() << RTT::endlog();
	RTT::log(RTT::Info) << "Model has " << urdf_joints_.size() << " joints"
			<< RTT::endlog();
	RTT::log(RTT::Info) << "Model has " << urdf_links_.size() << " links"
			<< RTT::endlog();

	for (unsigned int i = 0; i < _xbotcore_model.get_chain_names().size();
			++i) {

		std::string chain_name = _xbotcore_model.get_chain_names()[i];
		RTT::log(RTT::Info) << chain_name << " :chain" << RTT::endlog();
		std::vector<std::string> enabled_joints_in_chain;
		_xbotcore_model.get_enabled_joints_in_chain(chain_name,
				enabled_joints_in_chain);
		srdf::Model::RTTGazebo temp = _xbotcore_model.getRTTGazebo(chain_name);
		kinematic_chains.insert(
				std::pair<std::string, boost::shared_ptr<KinematicChain>>(
						chain_name,
						boost::shared_ptr<KinematicChain>(
								new KinematicChain(chain_name,
										enabled_joints_in_chain,
										*(this->ports()),
										new friRemote(49939,
												temp.hardware_info_.address_.c_str(),
												ip_addr.c_str(),
												this->getActivity()->thread()->getTask())))));
	}

	RTT::log(RTT::Info) << "Kinematic Chains map created!" << kinematic_chains.size()<< RTT::endlog();

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
	return true;
}

bool lwr_robot::loadURDFAndSRDF(const std::string &URDF_path,
		const std::string &SRDF_path) {
	if (!_models_loaded) {
		std::string _urdf_path = URDF_path;
		std::string _srdf_path = SRDF_path;

		RTT::log(RTT::Info) << "URDF path: " << _urdf_path << RTT::endlog();
		RTT::log(RTT::Info) << "SRDF path: " << _srdf_path << RTT::endlog();

		_models_loaded = _xbotcore_model.init(_urdf_path, _srdf_path);

		for (unsigned int i = 0; i < _xbotcore_model.get_chain_names().size();
				++i) {
			RTT::log(RTT::Info) << "chain #" << i << " "
					<< _xbotcore_model.get_chain_names()[i] << RTT::endlog();
			std::vector<std::string> enabled_joints_in_chain_i;
			_xbotcore_model.get_enabled_joints_in_chain(
					_xbotcore_model.get_chain_names()[i],
					enabled_joints_in_chain_i);
			for (unsigned int j = 0; j < enabled_joints_in_chain_i.size(); ++j)
				RTT::log(RTT::Info) << "  " << enabled_joints_in_chain_i[j]
						<< RTT::endlog();
		}
		model = _xbotcore_model.get_urdf_model();
	} else
		RTT::log(RTT::Info) << "URDF and SRDF have been already loaded!"
				<< RTT::endlog();

	return _models_loaded;
}

ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(cogimon::lwr_robot)

