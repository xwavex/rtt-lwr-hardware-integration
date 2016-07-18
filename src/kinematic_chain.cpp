#include <kinematic_chain.h>
#include <pid_values_tmp.h>

using namespace cogimon;
KinematicChain::KinematicChain(const std::string& chain_name,
		const std::vector<std::pair<std::string, int>> &joint_names,
		RTT::DataFlowInterface& ports,
		friRemote* friInst) :
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
	RTT::log(RTT::Info) << "Creating Kinematic Chain " << chain_name
			<< RTT::endlog();
	//changed from string string pair to string int as FRI doesn't use joint names but rather indexed positions (could change back to string and convert to int again?)
	for (unsigned int i = 0; i < _joint_names.size(); ++i)
		_map_joint_name_index.insert(
				std::pair<std::string, int>(_joint_names[i]));

	RTT::log(RTT::Info) << "Joints: " << RTT::endlog();
	for (unsigned int i = 0; i < _joint_names.size(); ++i)
		RTT::log(RTT::Info) << "    " << _joint_names[i].first << RTT::endlog();
}

std::vector<RTT::base::PortInterface*> KinematicChain::getAssociatedPorts() {
	return _inner_ports;
}

bool KinematicChain::initKinematicChain() {
	setJointNamesAndIndices();
	_number_of_dofs = _map_joint_name_index.size();
	_last_pos = new float[_number_of_dofs];
	memset(_last_pos, 0, _number_of_dofs * sizeof(*_last_pos));
	zero_vector = new float[_number_of_dofs];
	memset(zero_vector, 0, _number_of_dofs * sizeof(*zero_vector));
	_joint_pos.resize(_number_of_dofs);
	_joint_pos.setZero();
	_joint_trq.resize(_number_of_dofs);
	_joint_trq.setZero();
	_joint_stiff.resize(_number_of_dofs);
	_joint_stiff.setZero();
	_joint_damp.resize(_number_of_dofs);
	_joint_damp.setZero();

	if (!setController(std::string(ControlModes::JointPositionCtrl)))
		return false;
	if (!setController(std::string(ControlModes::JointImpedanceCtrl)))
		return false;
	if (!setController(std::string(ControlModes::JointTorqueCtrl)))
		return false;

	setFeedBack();
	//TODO change to init krc conrtoller (Possibly may not need anymore but needs adapting to new KRC.src program)
	if (!initKRC()) {
		return false;
	}
	setInitialPosition();
	setInitialImpedance();
	return true;
}

std::vector<int> KinematicChain::getJointScopedNames() {
	std::vector<int> joint_names;
	for (unsigned int i = 0; i < _joint_names.size(); ++i)
		joint_names.push_back(_joint_names[i].second);
	return joint_names;
}

std::vector<std::string> KinematicChain::getJointNames() {
	std::vector<std::string> joint_names;
	for (unsigned int i = 0; i < _joint_names.size(); ++i)
		joint_names.push_back(_joint_names[i].first);
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
			"Output for Joint-fb from FRI to Orocos world. Contains joint-position, -velocity and -torque.");
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

void KinematicChain::setInitialPosition() {
	position_controller->orocos_port.clear();
	for (unsigned int i = 0; i < _joint_names.size(); ++i)
		position_controller->joint_cmd.angles[i] =
				_fri_inst->getMsrMsrJntPosition()[i];
	position_controller->joint_cmd_fs = RTT::FlowStatus::NewData;
}

void KinematicChain::setInitialImpedance() {
	hardcoded_impedance impedance_init;
	for (unsigned int i = 0; i < _joint_names.size(); ++i) {
		impedance_controller->joint_cmd.stiffness[i] =
				impedance_init.impedance[_joint_names[i].first].first;
		impedance_controller->joint_cmd.damping[i] =
				impedance_init.impedance[_joint_names[i].first].second;
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
	RTT::log(RTT::Warning) << "CHAGING TO CONTROL" << RTT::endlog();
//TODO add all control modes in properly with different options between them	
	if(controlMode == ControlModes::JointPositionCtrl){
		_fri_inst->getCmdBuf().cmd.cmdFlags=FRI_CMD_JNTPOS;
		_fri_inst->setToKRLInt(14,10);
	} else if (controlMode == ControlModes::JointTorqueCtrl){
		_fri_inst->getCmdBuf().cmd.cmdFlags=0;
		_fri_inst->getCmdBuf().cmd.cmdFlags|=FRI_CMD_JNTPOS;
		_fri_inst->getCmdBuf().cmd.cmdFlags|=FRI_CMD_JNTSTIFF;
		_fri_inst->getCmdBuf().cmd.cmdFlags|=FRI_CMD_JNTDAMP;
		_fri_inst->getCmdBuf().cmd.cmdFlags|=FRI_CMD_JNTTRQ;
		_fri_inst->setToKRLInt(14,30);
	}	
//	else if (controlMode == ControlModes::JointTorqueCtrl
//			|| controlMode == ControlModes::JointImpedanceCtrl)
//		_gazebo_position_joint_controller->Reset();

	_current_control_mode = controlMode;
	return true;
}

void KinematicChain::sense() {
	_fri_inst->doReceiveData();
	_fri_inst->setToKRLInt(15,0);
	if(_fri_inst->getFrmKRLInt(15)<10){
		return;
	}
	if(_fri_inst->getFrmKRLInt(15) == 10){
		_fri_inst->setToKRLInt(15,10);
		return;
	}
	if (full_feedback) {
		time_now = RTT::os::TimeService::Instance()->getNSecs();
		for (unsigned int i = 0; i < _number_of_dofs; ++i)
			full_feedback->joint_feedback.angles(i) =
					_fri_inst->getMsrMsrJntPosition()[i];

		//TODO calculate velocity feedback! Easy way first, maybe Kalman afterwards?

		for (unsigned int i = 0; i < _number_of_dofs; ++i) {
			full_feedback->joint_feedback.velocities(i) =
					(_fri_inst->getMsrMsrJntPosition()[i] - _last_pos[i])
							/ (time_now - last_time);
			_last_pos[i] = _fri_inst->getMsrMsrJntPosition()[i];
		}

		for (unsigned int i = 0; i < _number_of_dofs; ++i) {
			full_feedback->joint_feedback.torques(i) =
					_fri_inst->getMsrJntTrq()[i];
		}

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
if(_fri_inst->getFrmKRLInt(15)==20){
RTT::log(RTT::Info) << "movce! "<<_current_control_mode << RTT::endlog();
	if (_current_control_mode == ControlModes::JointPositionCtrl) {
		//if(_fri_inst->getCurrentControlScheme()
		//		!= FRI_CTRL::FRI_CTRL_POSITION){
		//	_fri_inst->setToKRLInt(1, 10);
		//}else{
	RTT::log(RTT::Info) << "jointPos!" << RTT::endlog();
		//Currently done on index only, maybe convert to names as well?
//		_fri_inst->doPositionControl(
//				position_controller->joint_cmd.angles.data(), false);
		std::vector<int> joint_scoped_names = getJointScopedNames();
		for (unsigned int i = 0; i < joint_scoped_names.size(); ++i) {
			_joint_pos(joint_scoped_names[i]) =position_controller->joint_cmd.angles(i);
		}
		if(position_controller->joint_cmd_fs==RTT::NewData){
		_fri_inst->doPositionControl(_joint_pos.data(), false);
		}
	} else if (_current_control_mode == ControlModes::JointTorqueCtrl) {
RTT::log(RTT::Info) << "jointtrq!" << RTT::endlog();
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
if(_fri_inst->getCurrentControlScheme()
				!= FRI_CTRL::FRI_CTRL_JNT_IMP){
			_fri_inst->setToKRLInt(1, 30);
		}else{
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
		_fri_inst->doJntImpedanceControl(_joint_pos.data(), _joint_stiff.data(),
				_joint_damp.data(), _joint_trq.data(), false);
}
	}
}else{
	for(int i = 0; i < LBR_MNJ;i++){
	_fri_inst->getCmdBuf().cmd.jntPos[i] = _fri_inst->getMsrBuf().data.cmdJntPos[i] + _fri_inst->getMsrBuf().data.cmdJntPosFriOffset[i];
	}
}
	_fri_inst->doSendData();
}

std::string KinematicChain::printKinematicChainInformation() {
	std::stringstream joint_names_stream;
	for (unsigned int i = 0; i < _joint_names.size(); ++i)
		joint_names_stream << _joint_names[i].first << " ";

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
