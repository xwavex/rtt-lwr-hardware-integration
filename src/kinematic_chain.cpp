#include <kinematic_chain.h>
#include <pid_values_tmp.h>

using namespace cogimon;
KinematicChain::KinematicChain(const std::string& chain_name,
		const std::vector<std::string> &joint_names,
		RTT::DataFlowInterface& ports, friRemote* friInst) :
		_kinematic_chain_name(chain_name), _ports(ports), _current_control_mode(
				std::string(ControlModes::JointPositionCtrl)), _joint_names(
				joint_names) {
	//move pointer for _fri_inst
	this->_fri_inst = friInst;
	char str[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &(_fri_inst->remote.krcAddr.sin_addr), str,
	INET_ADDRSTRLEN);
	//store krc_ip address in case it's needed
	this->_krc_ip = std::string(str);
	RTT::log(RTT::Info) << "Creating Kinematic Chain " << chain_name << " WITH IP: "<<this->_krc_ip<< RTT::endlog();
	//changed from string string pair to string int as FRI doesn't use joint names but rather indexed positions (could change back to string and convert to int again?)
	for (unsigned int i = 0; i < _joint_names.size(); ++i)
		_map_joint_name_index.insert(
				std::pair<std::string, int>(_joint_names[i], i));

	RTT::log(RTT::Info) << "Joints: " << RTT::endlog();
	for (unsigned int i = 0; i < _joint_names.size(); ++i)
		RTT::log(RTT::Info) << "    " << _joint_names[i] << RTT::endlog();

	_initial_joints_configuration.reserve(joint_names.size());
	for (unsigned int i = 0; i < joint_names.size(); ++i)
		_initial_joints_configuration.push_back(0.0);

	estExtTorques = rstrt::dynamics::JointTorques(_joint_names.size());
	estExtTorques.torques.setZero();
	estExtTorques_port.setName("est_ext_torque");
	estExtTorques_port.setDataSample(estExtTorques);
	output_M_var.setZero();
	output_M_port.setName("output_M");
	output_M_port.setDataSample(output_M_var);
}

std::vector<RTT::base::PortInterface*> KinematicChain::getAssociatedPorts() {
	return _inner_ports;
}

bool KinematicChain::initKinematicChain() {
	RTT::log(RTT::Info) << "DEBUG1" << RTT::endlog();
	setJointNamesAndIndices();
	_number_of_dofs = _map_joint_name_index.size();
	RTT::log(RTT::Info) << "DEBUG2 :" << _number_of_dofs << RTT::endlog();
	//set last position memory for torque mode
	_last_pos = new float[_number_of_dofs];
	memset(_last_pos, 0, _number_of_dofs * sizeof(*_last_pos));
	RTT::log(RTT::Info) << "DEBUG3" << RTT::endlog();
	//set zero vector for torque mode 0 vectors
	zero_vector = new float[_number_of_dofs];
	RTT::log(RTT::Info) << "DEBUG4" << RTT::endlog();
	memset(zero_vector, 0, _number_of_dofs * sizeof(*zero_vector));
	_joint_pos.resize(_number_of_dofs);
	_joint_pos.setZero();
	_joint_trq.resize(_number_of_dofs);
	_joint_trq.setZero();
	_joint_stiff.resize(_number_of_dofs);
	_joint_stiff.setZero();
	_joint_damp.resize(_number_of_dofs);
	_joint_damp.setZero();
	RTT::log(RTT::Info) << "DEBUG5" << RTT::endlog();
	if (!setController(std::string(ControlModes::JointPositionCtrl)))
		return false;
	if (!setController(std::string(ControlModes::JointImpedanceCtrl)))
		return false;
	if (!setController(std::string(ControlModes::JointTorqueCtrl)))
		return false;
	RTT::log(RTT::Info) << "DEBUG6" << RTT::endlog();
	setFeedBack();
	RTT::log(RTT::Info) << "DEBUG7" << RTT::endlog();
	//TODO change to init krc conrtoller (Possibly may not need anymore but needs adapting to new KRC.src program)
	if (!initKRC()) {
		return false;
	}
	setInitialPosition(false);
	RTT::log(RTT::Info) << "DEBUG8" << RTT::endlog();
	setInitialImpedance();
	RTT::log(RTT::Info) << "DEBUG9" << RTT::endlog();
	return true;
}

bool KinematicChain::resetKinematicChain() {
	setControlMode(ControlModes::JointPositionCtrl);
	setInitialPosition(false);
	setInitialImpedance();
	return true;
}

//scoped names actually become ints for fri
std::vector<int> KinematicChain::getJointScopedNames() {
	std::vector<int> joint_names;
	for (unsigned int i = 0; i < _joint_names.size(); ++i)
		joint_names.push_back(_map_joint_name_index.at(_joint_names[i]));
	return joint_names;
}

std::vector<std::string> KinematicChain::getJointNames() {
	std::vector<std::string> joint_names;
	for (unsigned int i = 0; i < _joint_names.size(); ++i)
		joint_names.push_back(_joint_names[i]);
	return joint_names;
}

std::vector<std::string> KinematicChain::getControllersAvailable() {
	return _controllers_name;
}

std::string KinematicChain::getKinematicChainName() {
	return _kinematic_chain_name;
}

unsigned int KinematicChain::getNumberOfDOFs() {
	return _number_of_dofs;
}

std::string KinematicChain::getCurrentControlMode() {
	return _current_control_mode;
}

void KinematicChain::setFeedBack() {
	full_feedback.reset(new full_fbk);
	full_feedback->orocos_port.setName(
			_kinematic_chain_name + "_JointFeedback");
	full_feedback->orocos_port.doc(
			"Output for Joint-fb from FRI to Orocos world. Contains joint-position, and -torque.");
	_ports.addPort(full_feedback->orocos_port);
	// TODO needs to be solved better
	_inner_ports.push_back(
			_ports.getPort(full_feedback->orocos_port.getName()));

	full_feedback->joint_feedback = JointState(_number_of_dofs);
	full_feedback->orocos_port.setDataSample(full_feedback->joint_feedback);
}

bool KinematicChain::setController(const std::string& controller_type) {
	if (controller_type == ControlModes::JointPositionCtrl) {
		position_controller.reset(new position_ctrl);
		position_controller->orocos_port.setName(
				_kinematic_chain_name + "_" + ControlModes::JointPositionCtrl);
		position_controller->orocos_port.doc(
				"Input for JointPosition-cmds from Orocos to FRI world.");
		_ports.addPort(position_controller->orocos_port);
		// TODO needs to be solved better
		_inner_ports.push_back(
				_ports.getPort(position_controller->orocos_port.getName()));

		position_controller->joint_cmd = JointAngles(_number_of_dofs);
		position_controller->joint_cmd.angles.setZero();
	} else if (controller_type == ControlModes::JointImpedanceCtrl) {
		impedance_controller.reset(new impedance_ctrl);
		impedance_controller->orocos_port.setName(
				_kinematic_chain_name + "_" + ControlModes::JointImpedanceCtrl);
		impedance_controller->orocos_port.doc(
				"Input for JointImpedance-cmds from Orocos to FRI world.");
		_ports.addPort(impedance_controller->orocos_port);
		// TODO needs to be solved better
		_inner_ports.push_back(
				_ports.getPort(impedance_controller->orocos_port.getName()));

		impedance_controller->joint_cmd = JointImpedance(_number_of_dofs);
		impedance_controller->joint_cmd.stiffness.setZero();
		impedance_controller->joint_cmd.damping.setZero();
	} else if (controller_type == ControlModes::JointTorqueCtrl) {
		torque_controller.reset(new torque_ctrl);
		torque_controller->orocos_port.setName(
				_kinematic_chain_name + "_" + ControlModes::JointTorqueCtrl);
		torque_controller->orocos_port.doc(
				"Input for JointTorque-cmds from Orocos to FRI world.");
		_ports.addPort(torque_controller->orocos_port);
		// TODO needs to be solved better
		_inner_ports.push_back(
				_ports.getPort(torque_controller->orocos_port.getName()));

		torque_controller->joint_cmd = JointTorques(_number_of_dofs);
		torque_controller->joint_cmd.torques.setZero();
	} else {
		RTT::log(RTT::Error) << "Control Mode: " << controller_type
				<< " is not available!" << RTT::endlog();
		return false;
	}
	_controllers_name.push_back(controller_type);
	return true;
}

bool KinematicChain::setJointNamesAndIndices() {
	std::map<std::string, int>::iterator it1;
	for (it1 = _map_joint_name_index.begin();
			it1 != _map_joint_name_index.end(); it1++) {
		_map_joint_name_index[it1->first] = it1->second;

		RTT::log(RTT::Info) << "Joint " << it1->first << " in chain "
				<< _kinematic_chain_name << " has scoped name "
				<< _map_joint_name_index[it1->first] << RTT::endlog();
	}
	return true;
}

void KinematicChain::setInitialPosition(const bool use_actual_model_pose) {
	position_controller->orocos_port.clear();
	RTT::log(RTT::Info) << _joint_names.size()<<" joint size" << RTT::endlog();
	//get initial positoin from fri, currently using loop possibly convert to Eigen::map or something?
	if (use_actual_model_pose) {
		for (unsigned int i = 0; i < _joint_names.size(); ++i)
			position_controller->joint_cmd.angles[i] =
					_fri_inst->getMsrMsrJntPosition()[i];
	} else {
		for (unsigned int i = 0; i < _joint_names.size(); ++i){
			RTT::log(RTT::Info) << position_controller->joint_cmd.angles.size()<<" loop, "<<
					_initial_joints_configuration.size()<< RTT::endlog();
			position_controller->joint_cmd.angles[i] =
					_initial_joints_configuration.at(i);}
	}
	position_controller->joint_cmd_fs = RTT::FlowStatus::NewData;
}

void KinematicChain::setInitialImpedance() {
	hardcoded_impedance impedance_init;
	for (unsigned int i = 0; i < _joint_names.size(); ++i) {
		impedance_controller->joint_cmd.stiffness[i] =
				impedance_init.impedance[_joint_names[i]].first;
		impedance_controller->joint_cmd.damping[i] =
				impedance_init.impedance[_joint_names[i]].second;
	}
	impedance_controller->joint_cmd_fs = RTT::FlowStatus::NewData;
}

bool KinematicChain::setControlMode(const std::string &controlMode) {
	if (controlMode != ControlModes::JointPositionCtrl
			&& controlMode != ControlModes::JointTorqueCtrl
			&& controlMode != ControlModes::JointImpedanceCtrl) {
		RTT::log(RTT::Warning) << "Control Mode " << controlMode
				<< " does not exist!" << RTT::endlog();
		return false;
	}

	if (!(std::find(_controllers_name.begin(), _controllers_name.end(),
			controlMode) != _controllers_name.end())) {
		RTT::log(RTT::Warning) << "Control Mode " << controlMode
				<< " is not available!" << RTT::endlog();
		return false;
	}
	//TODO add all control modes in properly with different options between them
	//set control modes and the correct cmdFlags as well (must be done in monitor mode not command mode! and can't be changed in command mode)	
	if (controlMode == ControlModes::JointPositionCtrl) {
		_fri_inst->getCmdBuf().cmd.cmdFlags = FRI_CMD_JNTPOS;
		_fri_inst->setToKRLInt(14, 10);
	} else if (controlMode == ControlModes::JointTorqueCtrl) {
		_fri_inst->getCmdBuf().cmd.cmdFlags = 0;
		_fri_inst->getCmdBuf().cmd.cmdFlags |= FRI_CMD_JNTPOS;
		_fri_inst->getCmdBuf().cmd.cmdFlags |= FRI_CMD_JNTSTIFF;
		_fri_inst->getCmdBuf().cmd.cmdFlags |= FRI_CMD_JNTDAMP;
		_fri_inst->getCmdBuf().cmd.cmdFlags |= FRI_CMD_JNTTRQ;
		_fri_inst->setToKRLInt(14, 30);
	}
//	else if (controlMode == ControlModes::JointTorqueCtrl
//			|| controlMode == ControlModes::JointImpedanceCtrl)
//		_gazebo_position_joint_controller->Reset();

	_current_control_mode = controlMode;
	return true;
}

void KinematicChain::sense() {
	//recieve data from fri
	_fri_inst->doReceiveData();
	//set mode command to zero so fri does not continue if it is being executed again (possibly move to component stop method)
	_fri_inst->setToKRLInt(15, 0);
	//if not in monitor mode straight return as nothing can be sensed
	if (_fri_inst->getFrmKRLInt(15) < 10) {
		RTT::log(RTT::Info) << _fri_inst->getFrmKRLInt(15)<<", "<<_fri_inst->getQuality()<<" :NOTHING BEING SENSED"<< RTT::endlog();
		return;
	}
	//if in monitor mode command fri to switch to command mode with the correct control mode
	if (_fri_inst->getFrmKRLInt(15) == 10) {
		RTT::log(RTT::Info) << "SWITCHING TO COMMAND MODE"<< RTT::endlog();
		_fri_inst->setToKRLInt(15, 10);
		//return;
	}
	if (full_feedback) {
		RTT::log(RTT::Info) << "FEEDBACK RECEIVING"<< RTT::endlog();
		time_now = RTT::os::TimeService::Instance()->getNSecs();
		//get the current pos
		for (unsigned int i = 0; i < _number_of_dofs; ++i)
			full_feedback->joint_feedback.angles(i) =
					_fri_inst->getMsrMsrJntPosition()[i];

		//TODO calculate velocity feedback! Easy way first, maybe Kalman afterwards?

		for (unsigned int i = 0; i < _number_of_dofs; ++i) {
			full_feedback->joint_feedback.velocities(i) =
					(_fri_inst->getMsrMsrJntPosition()[i] - _last_pos[i])
							/ ((time_now - last_time)*1E-9);
			_last_pos[i] = _fri_inst->getMsrMsrJntPosition()[i];
		}
		//get the current torques
		for (unsigned int i = 0; i < _number_of_dofs; ++i) {
			full_feedback->joint_feedback.torques(i) =
					_fri_inst->getMsrJntTrq()[i];
		}

		output_M_port.write(Eigen::Map<Eigen::Matrix<float,7,7>>(_fri_inst->getMsrBuf().data.massMatrix));
		estExtTorques.torques = Eigen::Map<Eigen::VectorXf>(_fri_inst->getMsrBuf().data.estExtJntTrq,7,1);
		estExtTorques_port.write(estExtTorques);
		if (full_feedback->orocos_port.connected())
			full_feedback->orocos_port.write(full_feedback->joint_feedback);
		last_time = time_now;
	}
}

void KinematicChain::getCommand() {
	if (_current_control_mode == ControlModes::JointTorqueCtrl)
		torque_controller->joint_cmd_fs =
				torque_controller->orocos_port.readNewest(
						torque_controller->joint_cmd);
	else if (_current_control_mode == ControlModes::JointPositionCtrl)
		position_controller->joint_cmd_fs =
				position_controller->orocos_port.readNewest(
						position_controller->joint_cmd);
	else if (_current_control_mode == ControlModes::JointImpedanceCtrl) {
		position_controller->joint_cmd_fs =
				position_controller->orocos_port.readNewest(
						position_controller->joint_cmd);
		impedance_controller->joint_cmd_fs =
				impedance_controller->orocos_port.readNewest(
						impedance_controller->joint_cmd);
		torque_controller->joint_cmd_fs =
				torque_controller->orocos_port.readNewest(
						torque_controller->joint_cmd);
	}
}

void KinematicChain::move() {
	/*if(_fri_inst->getQuality()!= FRI_QUALITY::FRI_QUALITY_PERFECT){

	 return;
	 }*/
//only run when KRC is in command mode (don't need to check for perfect communication as the program will only go into command mode when communication is perfect)
	RTT::log(RTT::Info) << "MOVE"<< RTT::endlog();
	if (_fri_inst->getFrmKRLInt(15) == 20) {
		RTT::log(RTT::Info) << "CMD MODE"<< RTT::endlog();
		if (_current_control_mode == ControlModes::JointPositionCtrl) {
			RTT::log(RTT::Info) << "JNTPOS MODE"<< RTT::endlog();
			//if(_fri_inst->getCurrentControlScheme()
			//		!= FRI_CTRL::FRI_CTRL_POSITION){
			//	_fri_inst->setToKRLInt(1, 10);
			//}else{
			//Currently done on index only, maybe convert to names as well?
//		_fri_inst->doPositionControl(
//				position_controller->joint_cmd.angles.data(), false);
			std::vector<int> joint_scoped_names = getJointScopedNames();
			for (unsigned int i = 0; i < joint_scoped_names.size(); ++i) {
				_joint_pos(joint_scoped_names[i]) =
						position_controller->joint_cmd.angles(i);
			}
			RTT::log(RTT::Info) << "JNTPOS: "<<_joint_pos<< RTT::endlog();
			if (position_controller->joint_cmd_fs == RTT::NewData) {
				_fri_inst->doPositionControl(_joint_pos.data(), false);
			}
		} else if (_current_control_mode == ControlModes::JointTorqueCtrl) {
			//if(_fri_inst->getCurrentControlScheme()
			//		!= FRI_CTRL::FRI_CTRL_JNT_IMP){
			//	_fri_inst->setToKRLInt(1, 30);
			//}else{
			std::vector<int> joint_scoped_names = getJointScopedNames();
			for (unsigned int i = 0; i < joint_scoped_names.size(); ++i) {
				_joint_trq(joint_scoped_names[i]) =
						torque_controller->joint_cmd.torques(i);
			}
			_fri_inst->doJntImpedanceControl(_fri_inst->getMsrMsrJntPosition(),
					zero_vector, zero_vector, _joint_trq.data(), false);
//}
		} else if (_current_control_mode == ControlModes::JointImpedanceCtrl) {
//if(_fri_inst->getCurrentControlScheme()
			//			!= FRI_CTRL::FRI_CTRL_JNT_IMP){
			//	_fri_inst->setToKRLInt(1, 30);
			//	}else{
			std::vector<int> joint_scoped_names = getJointScopedNames();
			for (unsigned int i = 0; i < joint_scoped_names.size(); ++i) {
				_joint_pos(joint_scoped_names[i]) =
						position_controller->joint_cmd.angles(i);
				_joint_stiff(joint_scoped_names[i]) =
						impedance_controller->joint_cmd.stiffness(i);
				_joint_damp(joint_scoped_names[i]) =
						impedance_controller->joint_cmd.damping(i);
				_joint_trq(joint_scoped_names[i]) =
						torque_controller->joint_cmd.torques(i);
			}
			_fri_inst->doJntImpedanceControl(_joint_pos.data(),
					_joint_stiff.data(), _joint_damp.data(), _joint_trq.data(),
					false);
//}
		}
	} else {
		//if not in command mode run keep the jntPosition updated, required by FRI
		for (int i = 0; i < LBR_MNJ; i++) {
			_fri_inst->getCmdBuf().cmd.jntPos[i] =
					_fri_inst->getMsrBuf().data.cmdJntPos[i]
							+ _fri_inst->getMsrBuf().data.cmdJntPosFriOffset[i];
		}
	}

	//Send data
	_fri_inst->doSendData();
}

std::string KinematicChain::printKinematicChainInformation() {
	std::stringstream joint_names_stream;
	for (unsigned int i = 0; i < _joint_names.size(); ++i)
		joint_names_stream << _joint_names[i] << " ";

	std::vector<std::string> controller_names = getControllersAvailable();
	std::stringstream controller_names_stream;
	for (unsigned int i = 0; i < controller_names.size(); ++i)
		controller_names_stream << controller_names[i] << " ";

	std::stringstream info;
	info << "Kinematic Chain: " << _kinematic_chain_name << std::endl;
	info << "    Number of DOFs: " << _number_of_dofs << std::endl;
	info << "    Joints:  [" << joint_names_stream.str() << "]" << std::endl;
	info << "    Control Modes:  [ " << controller_names_stream.str() << "]"
			<< std::endl;
	info << "    Current Control Mode: " << _current_control_mode << std::endl;

	return info.str();
}

bool KinematicChain::initKRC() {
	last_quality = FRI_QUALITY_BAD;
	/*int i = 0;
	 RTT::log(RTT::Info) << "Trying to initialize"
	 << RTT::endlog();
	 last_quality = FRI_QUALITY_BAD;
	 //TODO do handshake as separate procedure?
	 while (_fri_inst->getQuality() < FRI_QUALITY::FRI_QUALITY_PERFECT || i<200) {
	 RTT::log(RTT::Info) << "Trying to initialize :"<<_fri_inst->getQuality()
	 << RTT::endlog();
	 _fri_inst->doReceiveData();
	 _fri_inst->setToKRLInt(0, 1);

	 last_quality = _fri_inst->getQuality();
	 if (last_quality >= FRI_QUALITY_OK ) {
	 i++;
	 _fri_inst->setToKRLInt(1, 10);
	 _fri_inst->doTest();
	 //_fri_inst->doDataExchange();
	 }
	 _fri_inst->setToKRLReal(1, _fri_inst->getFrmKRLReal(1));

	 _fri_inst->doSendData();
	 }*/
	return true;
}
