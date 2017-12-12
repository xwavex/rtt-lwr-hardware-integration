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

	this->addOperation("getModel", &lwr_robot::getModel, this, ClientThread);

	this->addOperation("setControlMode", &lwr_robot::setControlMode, this,
			RTT::ClientThread);

	this->addOperation("setGravity", &lwr_robot::setGravity, this,
			RTT::ClientThread);

	this->addOperation("setDebug", &lwr_robot::setDebug, this,
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
	this->addOperation("addChain", &lwr_robot::addChain, this,
			RTT::ClientThread);

	this->addOperation("setTrqFakeImpedance", &lwr_robot::setTrqFakeImpedance, this,
			RTT::ClientThread);
			

	// Default IP of this computer
	this->ip_addr = "192.168.0.1";
	this->addProperty("ip_addr", ip_addr).doc("IP address of the computer");

	_models_loaded = false;
	remote = NULL;
}

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
		chains.push_back(it->first);
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

void lwr_robot::setGravity(const std::string& kinematic_chain, const bool g) {
	std::vector<std::string> chain_names = getKinematicChains();
	if (!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain)
			!= chain_names.end())) {
		log(Warning) << "Kinematic Chain " << kinematic_chain
				<< " is not available!" << endlog();
	}
	kinematic_chains[kinematic_chain]->setGravity(g);
}

void lwr_robot::setDebug(const std::string& kinematic_chain, const bool debug) {
	std::vector<std::string> chain_names = getKinematicChains();
	if (!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain)
			!= chain_names.end())) {
		log(Warning) << "Kinematic Chain " << kinematic_chain
				<< " is not available!" << endlog();
	}
	kinematic_chains[kinematic_chain]->setDebug(debug);
}

void lwr_robot::setTrqFakeImpedance(const std::string& kinematic_chain, const rstrt::dynamics::JointImpedance imp, bool fakeImpedance) {
	std::vector<std::string> chain_names = getKinematicChains();
	if (!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain)
			!= chain_names.end())) {
		log(Warning) << "Kinematic Chain " << kinematic_chain
				<< " is not available!" << endlog();
	}
	kinematic_chains[kinematic_chain]->setTrqFakeImpedance(imp, fakeImpedance);
}

bool lwr_robot::getModel(const std::string& model_name) {
	if (model) {
		log(Warning) << "Model [" << model_name << "] already loaded !"
				<< endlog();
		return true;
	}
	//parse URDF
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
	//###################################################### TODO CURRENTLY NOT USED!!!!
	urdf_joints_ = model->joints_;
	urdf_links_ = model->links_;
	RTT::log(RTT::Info) << "Model name " << model->getName() << RTT::endlog();
	RTT::log(RTT::Info) << "Model has " << urdf_joints_.size() << " joints"
			<< RTT::endlog();
	RTT::log(RTT::Info) << "Model has " << urdf_links_.size() << " links"
			<< RTT::endlog();
    //###################################################### TODO CURRENTLY NOT USED!!!!


	/*for (unsigned int i = 0; i < _xbotcore_model.get_chain_names().size();
	 ++i) {
	 std::string chain_name = _xbotcore_model.get_chain_names()[i];
	 RTT::log(RTT::Info) << chain_name << " :chain" << RTT::endlog();
	 std::vector<std::string> enabled_joints_in_chain;
	 _xbotcore_model.get_enabled_joints_in_chain(chain_name,
	 enabled_joints_in_chain);
	 srdf_advr::Model::RTTGazebo temp = _xbotcore_model.getRTTGazebo(chain_name);
	 kinematic_chains.insert(
	 std::pair<std::string, boost::shared_ptr<KinematicChain>>(
	 chain_name,
	 boost::shared_ptr<KinematicChain>(
	 new KinematicChain(chain_name,
	 enabled_joints_in_chain,
	 *(this->ports()),
	 new friRemote(temp.hardware_info_.portNo_,
	 temp.hardware_info_.address_.c_str(),
	 ip_addr.c_str(),
	 this->getActivity()->thread()->getTask())))));
	 RTT::log(RTT::Info) << "PORTNO: " << temp.hardware_info_.portNo_<< RTT::endlog();
	 RTT::log(RTT::Info) << "IP: " << ip_addr<< RTT::endlog();
	 RTT::log(RTT::Info) << "IP2: " << temp.hardware_info_.address_<< RTT::endlog();
	 }*/

	// RTT::log(RTT::Info) << "Kinematic Chains map created!"
	// 		<< kinematic_chains.size() << RTT::endlog();

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
		// get the URDF and SRDF paths.
		// TODO check if the files exist!
		std::string _urdf_path = URDF_path;
		std::string _srdf_path = SRDF_path;

		RTT::log(RTT::Info) << "URDF path: " << _urdf_path << RTT::endlog();
		RTT::log(RTT::Info) << "SRDF path: " << _srdf_path << RTT::endlog();

		_models_loaded = _xbotcore_model.init(_urdf_path, _srdf_path);

		for (unsigned int i = 0; i < _xbotcore_model.get_chain_names().size(); ++i) {
			RTT::log(RTT::Info) << "chain #" << i << " " << _xbotcore_model.get_chain_names()[i] << RTT::endlog();

			// get the enabled joints from the parsed URDF and SRDF from xbotcoremodel.
			std::vector<std::string> enabled_joints_in_chain_i;
			_xbotcore_model.get_enabled_joints_in_chain(_xbotcore_model.get_chain_names()[i], enabled_joints_in_chain_i);

			for (unsigned int j = 0; j < enabled_joints_in_chain_i.size(); ++j) {
				RTT::log(RTT::Info) << "  " << enabled_joints_in_chain_i[j] << RTT::endlog();
			}
		}
		// store the parsed URDF model from xbotcoremodel.
		model = _xbotcore_model.get_urdf_model();
	} else {
		RTT::log(RTT::Info) << "URDF and SRDF have been already loaded!" << RTT::endlog();
	}

	RTT::log(RTT::Info) << "MODEL LOADED" << RTT::endlog();
	return _models_loaded;
}

bool lwr_robot::addChain(std::string name, std::string robot_ip, int robot_port,
		const std::string& URDF_path, const std::string& SRDF_path) {
	// load model and joints from the URDF and SRDF.
	loadURDFAndSRDF(URDF_path, SRDF_path);
	// get the names of the parsed chains from SRDF.
	std::string chain_name = _xbotcore_model.get_chain_names()[0];
	RTT::log(RTT::Info) << name << " :chain" << RTT::endlog();
	// get the enabled joints. ################################################################## TODO DUPLICATE!!!
	std::vector<std::string> enabled_joints_in_chain;
	_xbotcore_model.get_enabled_joints_in_chain(chain_name,
			enabled_joints_in_chain);
	// get the enabled joints. ################################################################## TODO DUPLICATE!!!
	kinematic_chains.insert(
			std::pair<std::string, boost::shared_ptr<KinematicChain>>(name,
					boost::shared_ptr<KinematicChain>(
						    /** create A kinematic chain. */
							new KinematicChain(
								/** choose a name. */
								chain_name,
							    /** add the enabled joints to control. */
								enabled_joints_in_chain,
								/** pass the pointer to the ports to enable the kinematic chain object to add ports. */
								*(this->ports()),
								/** create a new fri remote interface object (NOT WORKING FOR MULTI CHAINS?!). */
								new friRemote(
									/** port for the remote robot. */
									robot_port,
									/** ip for the remote robot. */
									robot_ip.c_str(),
									/** ip of this host. */
									ip_addr.c_str(),
									/** pass the task of the activity of this component. */
									this->getActivity()->thread()->getTask()
								)
							)
					)
			)
	);
	RTT::log(RTT::Info) << "PORTNO: " << robot_port << RTT::endlog();
	RTT::log(RTT::Info) << "IP: " << ip_addr << RTT::endlog();
	RTT::log(RTT::Info) << "IP2_ROBOT: " << robot_ip << RTT::endlog();
	return true;
}

ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(cogimon::lwr_robot)
