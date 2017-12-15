#include <kinematic_chain.h>
#include <pid_values_tmp.h>

using namespace cogimon;
KinematicChain::KinematicChain(const std::string& chain_name,
                               const std::vector<std::string> &joint_names,
							   RTT::DataFlowInterface& ports,
							   friRemote* friInst) :
		                                             _kinematic_chain_name(chain_name),
													 _ports(ports),
													 _current_control_mode(std::string(ControlModes::JointPositionCtrl)),
													 _joint_names(joint_names),
													 _friInternalCommandMode(-1),
													 _overrideFakeImpedance(false) {

	// store pointer for _fri_inst.
	this->_fri_inst = friInst;
	
	// convert an IP address to human-readable form and back.
	char inet_addr_str_chars[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &(_fri_inst->remote.krcAddr.sin_addr), inet_addr_str_chars, INET_ADDRSTRLEN);

	//store krc_ip address in case it's needed
	this->_krc_ip = std::string(inet_addr_str_chars);
	RTT::log(RTT::Info) << "Creating Kinematic Chain " << chain_name
			<< " WITH IP: " << this->_krc_ip << RTT::endlog();

	// TODO? changed from string string pair to string int as FRI doesn't use joint names,
	// but rather indexed positions. (could change back to string and convert to int again?).
	for (unsigned int i = 0; i < _joint_names.size(); ++i) {
		// map the joints.
		_map_joint_name_index.insert(std::pair<std::string, int>(_joint_names[i], i));
	}

	RTT::log(RTT::Info) << "Joints: " << RTT::endlog();
	for (unsigned int i = 0; i < _joint_names.size(); ++i)
		RTT::log(RTT::Info) << "    " << _joint_names[i] << RTT::endlog();

	// set the initial joint configuration to zero.
	_initial_joints_configuration.reserve(joint_names.size());
	for (unsigned int i = 0; i < joint_names.size(); ++i) {
		_initial_joints_configuration.push_back(0.0);
	}

	// initialize the variable for the estimated external torques.
	estExtTorques = rstrt::dynamics::JointTorques(_joint_names.size());
	estExtTorques.torques.setZero();
	estExtTorques_port.setName("est_ext_torque");
	estExtTorques_port.setDataSample(estExtTorques);

	// initialize the output of the inertial matrix.
	output_M_var.setZero();
	std::string temp_str = chain_name;
	temp_str.append("_output_M");
	output_M_port.setName(temp_str);
	output_M_port.setDataSample(output_M_var);
	_ports.addPort(output_M_port);

	// initialize trqModeJntImpedance port.
	trqModeJntImpedance = rstrt::dynamics::JointImpedance(_joint_names.size());
	temp_str = chain_name;
	temp_str.append("_trqModeJntImpedance");
	in_trqModeJntImpedance_port.setName(temp_str);
	in_trqModeJntImpedance_flow = RTT::NoData;
	_ports.addPort(in_trqModeJntImpedance_port);

	// initialize variable to include the gravity.
	include_gravity = true;

	// initialize the gravity compensation term coming from the robot. 
	gravity_torques = rstrt::dynamics::JointTorques(_joint_names.size());
	gravity_torques.torques.setZero();
	gravity_port.setName("out_gravity_port");
	gravity_port.setDataSample(gravity_torques);
	_ports.addPort(gravity_port);

	// initialize the variable for debugging.
	debug = false;
}

std::vector<RTT::base::PortInterface*> KinematicChain::getAssociatedPorts() {
	return _inner_ports;
}

bool KinematicChain::initKinematicChain() {
	// WHAT IS THIS ACTUALLY DOING?
	setJointNamesAndIndices();

	// get the number of degrees of freedom from the mapped joints.
	_number_of_dofs = _map_joint_name_index.size();

	// set the last position for torque mode to zero.
	_last_pos = new float[_number_of_dofs];
	memset(_last_pos, 0, _number_of_dofs * sizeof(*_last_pos));

	// set the zero vector for torque mode to zero. It is used for stiffness and damping.
	zero_vector = new float[_number_of_dofs];
	memset(zero_vector, 0, _number_of_dofs * sizeof(*zero_vector));

	// initialize internal variables.
	_joint_pos.resize(_number_of_dofs);
	_joint_pos.setZero();
	_joint_trq.resize(_number_of_dofs);
	_joint_trq.setZero();
	_joint_stiff.resize(_number_of_dofs);
	_joint_stiff.setZero();
	_joint_damp.resize(_number_of_dofs);
	_joint_damp.setZero();

	// check that all controller are correctly initialized for this chain.
	if (!setController(std::string(ControlModes::JointPositionCtrl)))
		return false;
	if (!setController(std::string(ControlModes::JointImpedanceCtrl)))
		return false;
	if (!setController(std::string(ControlModes::JointTorqueCtrl)))
		return false;

	// initialize the feedback.
	setFeedBack();

	//TODO change to init krc conrtoller (Possibly may not need anymore but needs adapting to new KRC.src program)
	// currently not doing anything except for setting last_quality = FRI_QUALITY_BAD.
	if (!initKRC()) {
		return false;
	}

	setInitialPosition(false);

	setInitialImpedance();

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
	// TODO WHAT IS THIS ACTUALLY DOING?
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
	// clear the port so that nothing is send.
	position_controller->orocos_port.clear();
	RTT::log(RTT::Info) << _joint_names.size() << " joint size" << RTT::endlog();

	// get initial position from fri, currently using loop possibly convert to Eigen::map or something?
	if (use_actual_model_pose) {
		for (unsigned int i = 0; i < _joint_names.size(); ++i) {
			// set the current position of the robot into the controller.
			position_controller->joint_cmd.angles[i] = _fri_inst->getMsrMsrJntPosition()[i];
		}
	} else {
		for (unsigned int i = 0; i < _joint_names.size(); ++i) {
			RTT::log(RTT::Info) << position_controller->joint_cmd.angles.rows()
					<< " loop, " << _initial_joints_configuration.size()
					<< RTT::endlog();
			// set the initial configuration from a propery or port.
			position_controller->joint_cmd.angles[i] = _initial_joints_configuration.at(i);
		}
	}
	// bump the position controller for new data.
	position_controller->joint_cmd_fs = RTT::FlowStatus::NewData;
}

void KinematicChain::setInitialImpedance() {
	// set the hardcoded impedance. TODO replace by information from SRDF.
	hardcoded_impedance impedance_init;
	for (unsigned int i = 0; i < _joint_names.size(); ++i) {
		impedance_controller->joint_cmd.stiffness[i] =
				impedance_init.impedance[_joint_names[i]].first;
		impedance_controller->joint_cmd.damping[i] =
				impedance_init.impedance[_joint_names[i]].second;
	}
	// bump the impedance controller for new data.
	impedance_controller->joint_cmd_fs = RTT::FlowStatus::NewData;
}

bool KinematicChain::setControlMode(const std::string &controlMode) {
	if (controlMode != ControlModes::JointPositionCtrl
	 && controlMode != ControlModes::JointTorqueCtrl
	 && controlMode != ControlModes::JointImpedanceCtrl) {
		RTT::log(RTT::Warning) << "Control Mode " << controlMode << " does not exist!" << RTT::endlog();
		return false;
	}

	if (!(std::find(_controllers_name.begin(),
	                _controllers_name.end(),
		            controlMode) != _controllers_name.end())) {
		RTT::log(RTT::Warning) << "Control Mode " << controlMode << " is not available!" << RTT::endlog();
		return false;
	}
	// TODO add all control modes in properly with different options between them.
	// set the control modes and the correct cmdFlags as well (must be done in monitor mode not command mode! It can't be changed in command mode).
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
	_fri_inst->doDataExchange();
	_current_control_mode = controlMode;
	return true;
}

bool KinematicChain::sense() {
	// get the current time.
	time_now = RTT::os::TimeService::Instance()->getNSecs();

	// recieve data from fri.
	int r = _fri_inst->doReceiveData();
	// just output for information.
	if (recieved && (r < 0)) {
		RTT::log(RTT::Warning) << this->getKinematicChainName() << "::sense() didn't receive any data! Skipping!" << RTT::endlog();
	} else if ((!recieved) && (r >= 0)) {
		RTT::log(RTT::Info) << this->getKinematicChainName() << "::sense() received data!" << RTT::endlog();
	}

	recieved = (r >= 0);
	if (!recieved) {
		// TODO do we need to handle this case?
		return false;
	}

    //RTT::log(RTT::Info) << _fri_inst->doReceiveData()<< ":RECIEVING"<<RTT::endlog();
	//RTT::log(RTT::Info) <<_fri_inst->getQuality() <<" QUALITY"<<RTT::endlog();

	//set mode command to zero so fri does not continue if it is being executed again (possibly move to component stop method)
	_fri_inst->setToKRLInt(15, 0); // ?
	if (_fri_inst->getQuality() < 2) {
		RTT::log(RTT::Critical) << this->getKinematicChainName() << "::sense() bad quality! " << _fri_inst->getQuality() << " < 2 Skipping!" << RTT::endlog();
		// TODO ALWAYS HANDLE QUALITY AND ACT ACCORDINGLY!
		return false;
	}
	//if not in monitor mode straight return as nothing can be sensed.
	// TODO DLW do this differently! Only trigger the update function when we are in command mode, or at least send commands only in command mode???
	int tmpCtrlModeFri = _fri_inst->getFrmKRLInt(15);
	if (_friInternalCommandMode != tmpCtrlModeFri) {
		RTT::log(RTT::Info) << this->getKinematicChainName() << "::sense() noticed a control mode update from " << _friInternalCommandMode << " to " << tmpCtrlModeFri << RTT::endlog();
		_friInternalCommandMode = tmpCtrlModeFri;
	}

	if (_friInternalCommandMode < 10) {
		// RTT::log(RTT::Warning) << this->getKinematicChainName() << "::sense() not in monitor mode! _fri_inst->getFrmKRLInt(15) = " << _fri_inst->getFrmKRLInt(15) << " Skipping!" << RTT::endlog();
		return false;
	}
	/*if (_fri_inst->getCurrentControlScheme()!=FRI_CTRL_JNT_IMP){
	 RTT::log(RTT::Info) << _fri_inst->getFrmKRLInt(15)<<", "<<_fri_inst->getQuality()<<" :ControlMode"<<_kinematic_chain_name<< RTT::endlog();
	 //setCommand
	 return;
	 }*/

	// TODO DLW auslagern???? DO THIS DIFFERENTLY AND NOT TRY TO DO THIS EVERY TIME!!!
	//if in monitor mode command fri to switch to command mode with the correct control mode
	tmpCtrlModeFri = _fri_inst->getFrmKRLInt(15);
	if (_friInternalCommandMode != tmpCtrlModeFri) {
		RTT::log(RTT::Info) << this->getKinematicChainName() << "::sense() noticed a control mode update (on second check) from " << _friInternalCommandMode << " to " << tmpCtrlModeFri << RTT::endlog();
		_friInternalCommandMode = tmpCtrlModeFri;
	}

	if (_friInternalCommandMode == 10) {
		RTT::log(RTT::Warning) << this->getKinematicChainName() << "::sense() switching to COMMAND MODE! Quality: " << _fri_inst->getQuality() << RTT::endlog();
		if (_fri_inst->getQuality() >= 2) {
			RTT::log(RTT::Info) << this->getKinematicChainName() << "::sense() Quality is good -> Proceed with switch to COMMAND MODE! " << _fri_inst->getQuality() << RTT::endlog();
			// request to call friStart() to go into command mode.
			_fri_inst->setToKRLInt(15, 10);
			//_fri_inst->doDataExchange();
		} else {
			RTT::log(RTT::Critical) << this->getKinematicChainName() << "::sense() bad quality! " << _fri_inst->getQuality() << " < 2 Skipping!" << RTT::endlog();
		}
	}

	// should always be true
	if (full_feedback) {
		// // get the current time.
		// time_now = RTT::os::TimeService::Instance()->getNSecs();

		//get the current joint positions from fri.
		for (unsigned int i = 0; i < _number_of_dofs; i++) {
			full_feedback->joint_feedback.angles(i) = _fri_inst->getMsrMsrJntPosition()[i];
		}

		//TODO calculate velocity feedback! Easy way first, maybe Kalman afterwards?
		for (unsigned int i = 0; i < _number_of_dofs; ++i) {
			// TODO DLW needs to be initialized, otherwise we will have a peak at the very beginning: full_feedback->joint_feedback.angles(i).
			full_feedback->joint_feedback.velocities(i) = (full_feedback->joint_feedback.angles(i) - _last_pos[i]) / _fri_inst->getMsrBuf().intf.desiredMsrSampleTime;	//((time_now - last_time)*1E-9);
			_last_pos[i] = full_feedback->joint_feedback.angles(i);
		}

		// get the current torques from fri.
		for (unsigned int i = 0; i < _number_of_dofs; ++i) {
			full_feedback->joint_feedback.torques(i) = _fri_inst->getMsrJntTrq()[i];
			//full_feedback->joint_feedback.torques(i) =_fri_inst->getMsrBuf().data.gravity[i];
		}
		
		// get inertial matrix from fri.
		output_M_port.write(Eigen::Map<Eigen::Matrix<float, 7, 7>>(_fri_inst->getMsrBuf().data.massMatrix));

		// get estimated external torques from fri.
		estExtTorques.torques = Eigen::Map<Eigen::VectorXf>(_fri_inst->getMsrBuf().data.estExtJntTrq, 7, 1);
		// write the est.ext torques to the output port.
		estExtTorques_port.write(estExtTorques);

		// get the gravity compoensation torques from fri.
		gravity_torques.torques = Eigen::Map<Eigen::VectorXf>(_fri_inst->getMsrBuf().data.gravity, 7, 1);
		// write the grav comp torques to the output port.
		gravity_port.write(gravity_torques);

		// if the output port is connected send the data.
		//if (full_feedback->orocos_port.connected())
		full_feedback->orocos_port.write(full_feedback->joint_feedback);

		// set store the last time.
		last_time = time_now;
		RTT::log(RTT::Debug) << this->getKinematicChainName() << "::sense() Took " << RTT::os::TimeService::Instance()->secondsSince(time_now) << " seconds." << RTT::endlog();
	}
	return true;
}

void KinematicChain::getCommand() {
	RTT::nsecs time_start = RTT::os::TimeService::Instance()->getNSecs();

	// TODO if we couldn't receive any feedback, do not read any command!?
	if (!recieved) {
		return;
	}

	// if the quality is bad, do not read a command!?
	FRI_QUALITY q = _fri_inst->getQuality();
	if (q < 2) {
		RTT::log(RTT::Critical) << this->getKinematicChainName() << "::getCommand() bad quality! " << q << " < 2 Skipping!" << RTT::endlog();
		return;
	}

	int tmpCtrlModeFri = _fri_inst->getFrmKRLInt(15);
	if (_friInternalCommandMode != tmpCtrlModeFri) {
		RTT::log(RTT::Info) << this->getKinematicChainName() << "::getCommand() noticed a control mode update from " << _friInternalCommandMode << " to " << tmpCtrlModeFri << RTT::endlog();
		_friInternalCommandMode = tmpCtrlModeFri;
	}

	// read newest command from the ports?!
	if (_current_control_mode == ControlModes::JointTorqueCtrl) {
		torque_controller->joint_cmd_fs =
				torque_controller->orocos_port.readNewest(
						torque_controller->joint_cmd);
		// read impedance for changing the HACK externally!
		in_trqModeJntImpedance_flow = in_trqModeJntImpedance_port.readNewest(trqModeJntImpedance);
	} else if (_current_control_mode == ControlModes::JointPositionCtrl) {
		position_controller->joint_cmd_fs =
				position_controller->orocos_port.readNewest(
						position_controller->joint_cmd);
	} else if (_current_control_mode == ControlModes::JointImpedanceCtrl) {
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
	RTT::log(RTT::Debug) << this->getKinematicChainName() << "::getCommand() Took " << RTT::os::TimeService::Instance()->secondsSince(time_start) << " seconds." << RTT::endlog();
}

void KinematicChain::stop() {
	if (_current_control_mode == ControlModes::JointPositionCtrl) {
		// we do nothing here.
	} else if (_current_control_mode == ControlModes::JointTorqueCtrl) {
		// TODO DLW check if we are in control mode first!
		std::vector<int> joint_scoped_names = getJointScopedNames();
		for (unsigned int i = 0; i < joint_scoped_names.size(); ++i) {
			_joint_trq(joint_scoped_names[i]) = 0;
			zero_vector[joint_scoped_names[i]] = 0.4;
		}
		// we try to brake a litte bit by increasing the impedance and setting zero ref torque.
		_fri_inst->doJntImpedanceControl(_fri_inst->getMsrMsrJntPosition(), zero_vector, zero_vector, _joint_trq.data(), true);
	}

	int r = 0;
	//while(_fri_inst->getFrmKRLInt(15) != 10 &&  (r>=0)){
	while (r < 20) {
		// TODO DLW check what this exactly does!
		_fri_inst->setToKRLInt(15, 20);
		_fri_inst->doDataExchange();
		r++;
		RTT::log(RTT::Critical) << r << ":stopping" << RTT::endlog();
	}
	_fri_inst->setToKRLInt(15, 0);
	_fri_inst->doDataExchange();
	_fri_inst->doDataExchange();
	_fri_inst->doDataExchange();
}

void KinematicChain::move() {
	RTT::nsecs time_start = RTT::os::TimeService::Instance()->getNSecs();

	// TODO only send a command if we also received something?!
	if (!recieved) {
		RTT::log(RTT::Warning)
			<< this->getKinematicChainName()
			<< "::move() didn't receive any data! doSendData() and Skipping!" << RTT::endlog();
		_fri_inst->doSendData();
		return; // Strange????
	}

	// TODO only send a command if the quality is still good?!
	FRI_QUALITY q = _fri_inst->getQuality();
	if (q < 2) {
		RTT::log(RTT::Critical)
			<< this->getKinematicChainName() << "::move() bad quality! " << q
			<< " < 2 doSendData() and Skipping!" << RTT::endlog();
		_fri_inst->doSendData();
		return;
	}
	//RTT::log(RTT::Info) << "DEBUG1: "<<_fri_inst->getFrmKRLInt(15) <<", "<< (_current_control_mode == ControlModes::JointTorqueCtrl) <<", "<< (_fri_inst->getCurrentControlScheme()==FRI_CTRL_JNT_IMP)<<RTT::endlog();
	/*if(_fri_inst->getQuality()!= FRI_QUALITY::FRI_QUALITY_PERFECT){

	return;
	}*/
	int tmpCtrlModeFri = _fri_inst->getFrmKRLInt(15);
	if (_friInternalCommandMode != tmpCtrlModeFri) {
		RTT::log(RTT::Info) << this->getKinematicChainName() << "::move() noticed a control mode update from " << _friInternalCommandMode << " to " << tmpCtrlModeFri << RTT::endlog();
		_friInternalCommandMode = tmpCtrlModeFri;
	}
	// only run when KRC is in command mode.
	if (_friInternalCommandMode == 20) {
		// check the currently active control mode.
		// TODO we could also verify it from fri again.
		if (_current_control_mode == ControlModes::JointPositionCtrl) {
			//if(_fri_inst->getCurrentControlScheme() != FRI_CTRL::FRI_CTRL_POSITION){
			//	_fri_inst->setToKRLInt(1, 10);
			//}
			std::vector<int> joint_scoped_names = getJointScopedNames();
			for (unsigned int i = 0; i < joint_scoped_names.size(); ++i) {
				// read the command positions stored in the positioncontroller.
				_joint_pos(joint_scoped_names[i]) = position_controller->joint_cmd.angles(i);
			}
			// TODO perhaps the loop should also got in the IF below.
			// basically we are not sending old data; which makes sense in position mode.
			if (position_controller->joint_cmd_fs == RTT::NewData) {
				// only send data if it was new.
				_fri_inst->doPositionControl(_joint_pos.data(), false);
			}
		} else if (_current_control_mode == ControlModes::JointTorqueCtrl && _fri_inst->getCurrentControlScheme() == FRI_CTRL_JNT_IMP) {

			std::vector<int> joint_scoped_names = getJointScopedNames();
			for (unsigned int i = 0; i < joint_scoped_names.size(); i++) {
				// read the command torques stored in the torquecontroller.
				_joint_trq(joint_scoped_names[i]) = torque_controller->joint_cmd.torques(i);

				// if we do not want to include the gravity, we substract it to simulate the kuka not compensating for it.
				if (!include_gravity) {
					// read current gravity vector from fri and substract it from the current torque command.
					_joint_trq(joint_scoped_names[i]) -= _fri_inst->getMsrBuf().data.gravity[joint_scoped_names[i]];
				}
			}
			//RTT::log(RTT::Critical)<<"Joint Torques: "<<_joint_trq.transpose()<<RTT::endlog();

			// TODO why?? if debug is active set the torque command to zero, and let kuka compensate for gravity.
			if (debug) {
				_joint_trq.setZero();
				if (_overrideFakeImpedance) {
					_fri_inst->doJntImpedanceControl(_fri_inst->getMsrMsrJntPosition(), trqModeJntImpedance.stiffness.data(), trqModeJntImpedance.damping.data(), _joint_trq.data(), false);
				} else {
					if (in_trqModeJntImpedance_flow != RTT::NoData) {
						_fri_inst->doJntImpedanceControl(_fri_inst->getMsrMsrJntPosition(), trqModeJntImpedance.stiffness.data(), trqModeJntImpedance.damping.data(), _joint_trq.data(), false);
					} else {
						_fri_inst->doJntImpedanceControl(_fri_inst->getMsrMsrJntPosition(), zero_vector, zero_vector, _joint_trq.data(), false);
					}
				}
			} else {
				if (torque_controller->joint_cmd_fs != RTT::NoData) {
					// TODO try to implement a smooth transition!!!! DLW
					//_fri_inst->doJntImpedanceControl(_fri_inst->getMsrMsrJntPosition(), zero_vector, zero_vector, _joint_trq.data(), false);
					if (_overrideFakeImpedance) {
						_fri_inst->doJntImpedanceControl(_fri_inst->getMsrMsrJntPosition(), trqModeJntImpedance.stiffness.data(), trqModeJntImpedance.damping.data(), _joint_trq.data(), false);
					} else {
						if (in_trqModeJntImpedance_flow != RTT::NoData) {
							_fri_inst->doJntImpedanceControl(_fri_inst->getMsrMsrJntPosition(), trqModeJntImpedance.stiffness.data(), trqModeJntImpedance.damping.data(), _joint_trq.data(), false);
						} else {
							_fri_inst->doJntImpedanceControl(_fri_inst->getMsrMsrJntPosition(), zero_vector, zero_vector, _joint_trq.data(), false);
						}
					}
					// Call to data exchange - and the like 
					// friInst.doJntImpedanceControl(NULL,//newJntVals, 
					// 			  newJntStiff, 
					// 			  newJntDamp, 
					// 			  newJntAddTorque, 
					// 			  false);
				}
			}
		} else if (_current_control_mode == ControlModes::JointImpedanceCtrl) {
			// TODO CURRENTLY NOT TESTED!!!
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

				if (!include_gravity) {
					_joint_trq(joint_scoped_names[i]) -= _fri_inst->getMsrBuf().data.gravity[joint_scoped_names[i]];
				}
			}

			if (torque_controller->joint_cmd_fs != RTT::NoData) {
				_fri_inst->doJntImpedanceControl(_joint_pos.data(), _joint_stiff.data(), _joint_damp.data(), _joint_trq.data(), false);
			}
		}	
	} else {
		// RTT::log(RTT::Warning)
		// 	<< this->getKinematicChainName()
		// 	<< "::move() whyyyyy not in command mode?! And updating strange stuff?" << RTT::endlog();
		// if not in command mode run keep the jntPosition updated, required
		// by FRI
		// TODO DLW dunno, could be a cause of error. Needs investigation.
		for (int i = 0; i < LBR_MNJ; i++) {
			_fri_inst->getCmdBuf().cmd.jntPos[i] =
				_fri_inst->getMsrBuf().data.cmdJntPos[i] +
				_fri_inst->getMsrBuf().data.cmdJntPosFriOffset[i];

			// RTT::log(RTT::Warning)
			// 	<< this->getKinematicChainName()
			// 	<< "::move() _fri_inst->getCmdBuf().cmd.jntPos[" << i << "] = " << _fri_inst->getCmdBuf().cmd.jntPos[i] << RTT::endlog();
        }
		// not sure if I need to do this?!
		//_fri_inst->doReceiveData();
	}

	// we need to do this manually, because we use e.g., doJntImpedanceControl with flagDataExchange = false.
	_fri_inst->doSendData();

	RTT::log(RTT::Debug) << this->getKinematicChainName() << "::move() Took " << RTT::os::TimeService::Instance()->secondsSince(time_start) << " seconds." << RTT::endlog();
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
	info << "    Impedance: Stiffness = " << trqModeJntImpedance.stiffness << std::endl;
	info << "    Impedance: Damping = " << trqModeJntImpedance.damping << std::endl;
	info << "    overrideFakeImpedance = " << _overrideFakeImpedance << std::endl;

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
void KinematicChain::setGravity(bool g) {
	include_gravity = g;
}

void KinematicChain::setDebug(bool g) {
	debug = g;
}

void KinematicChain::setTrqFakeImpedance(rstrt::dynamics::JointImpedance imp, bool fakeImpedance) {
	trqModeJntImpedance = imp;
	_overrideFakeImpedance = fakeImpedance;
}