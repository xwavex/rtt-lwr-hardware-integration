#include <kinematic_chain.h>
#include <pid_values_tmp.h>

using namespace cogimon;
//todo add fri commuincation
KinematicChain::KinematicChain(const std::string& chain_name,
		const std::vector<std::pair<std::string, int>> &joint_names,
		RTT::DataFlowInterface& ports, //TODO FRI INST
		friRemote* friInst) :
		_kinematic_chain_name(chain_name), _ports(ports), _current_control_mode(
				std::string(ControlModes::JointPositionCtrl)), _joint_names(
				joint_names) {
	this->_fri_inst = friInst;
	char str[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &(_fri_inst->remote.krcAddr.sin_addr), str,
	INET_ADDRSTRLEN);
	this->_krc_ip = std::string(str);
	RTT::log(RTT::Info) << "Creating Kinematic Chain " << chain_name
			<< RTT::endlog();

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
	//TODO change to init krc conrtoller
	if (!initKRC()) {
		return false;
	}

//	if (!initGazeboJointController()) {
//		RTT::log(RTT::Error)
//				<< "Joint Controller can NOT be initialized, exiting"
//				<< RTT::endlog();
//		return false;
//	}
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
		//TODO change to krc
		//_gazebo_position_joint_controller.reset(new gazebo::physics::JointController(_model));
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
		//gazebo::physics::JointPtr joint = _model->GetJoint(it1->first);
//		if (!joint) {
//			RTT::log(RTT::Error) << "No Joint" << it1->first
//					<< " could be added, exiting" << RTT::endlog();
//			return false;
//		}
		//_map_joint_name_scoped_name[it1->first] =
		//	_model->GetJoint(it1->first)->GetScopedName();
		_map_joint_name_index[it1->first] = it1->second;

		RTT::log(RTT::Info) << "Joint " << it1->first << " in chain "
				<< _kinematic_chain_name << " has scoped name "
				<< _map_joint_name_index[it1->first] << RTT::endlog();
	}
	return true;
}

//bool KinematicChain::initGazeboJointController() {
//	for (unsigned int i = 0; i < _joint_names.size(); ++i)
//		_gazebo_position_joint_controller->AddJoint(
//				_model->GetJoint(_joint_names[i]));
//
//	hardcoded_pids PID;
//	std::vector<std::string> joint_scoped_names = getJointScopedNames();
//	for (unsigned int i = 0; i < joint_scoped_names.size(); ++i)
//		_gazebo_position_joint_controller->SetPositionPID(joint_scoped_names[i],
//				PID.pids[_joint_names[i]]);
//
//	return true;
//}

void KinematicChain::setInitialPosition() {
	///TODO: check if user initial config is set when it is used in the gazebo configure hook

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

	if (controlMode == ControlModes::JointPositionCtrl) {
		setInitialPosition();
		while (_fri_inst->getCurrentControlScheme()
				!= FRI_CTRL::FRI_CTRL_POSITION) {
			_fri_inst->setToKRLInt(1, 10);
			_fri_inst->doDataExchange();
		}
	} else if (controlMode == ControlModes::JointImpedanceCtrl) {
		while (_fri_inst->getCurrentControlScheme()
				!= FRI_CTRL::FRI_CTRL_JNT_IMP) {
			_fri_inst->setToKRLInt(1, 30);
			_fri_inst->doDataExchange();
		}
	} else {
		while (_fri_inst->getCurrentControlScheme()
				!= FRI_CTRL::FRI_CTRL_JNT_IMP) {
			_fri_inst->setToKRLInt(1, 30);
			_fri_inst->doDataExchange();
		}
	}
//	else if (controlMode == ControlModes::JointTorqueCtrl
//			|| controlMode == ControlModes::JointImpedanceCtrl)
//		_gazebo_position_joint_controller->Reset();

	_current_control_mode = controlMode;
	return true;
}

void KinematicChain::sense() {
	_fri_inst->doReceiveData();
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
	if (_current_control_mode == ControlModes::JointPositionCtrl) {
		//Currently done on index only, maybe convert to names as well?
//		_fri_inst->doPositionControl(
//				position_controller->joint_cmd.angles.data(), false);
		std::vector<int> joint_scoped_names = getJointScopedNames();
		for (unsigned int i = 0; i < joint_scoped_names.size(); ++i) {
			_joint_pos(joint_scoped_names[i]) =
					position_controller->joint_cmd.angles(i);
		}
		_fri_inst->doPositionControl(_joint_pos.data(), false);

//		_gazebo_position_joint_controller->Update();
	} else if (_current_control_mode == ControlModes::JointTorqueCtrl) {
//		for (unsigned int i = 0; i < _joint_names.size(); ++i)
//			_model->GetJoint(_joint_names[i])->SetForce(0,
//					torque_controller->joint_cmd.torques(i));

		std::vector<int> joint_scoped_names = getJointScopedNames();
		for (unsigned int i = 0; i < joint_scoped_names.size(); ++i) {
			_joint_trq(joint_scoped_names[i]) =
					torque_controller->joint_cmd.torques(i);
		}
		_fri_inst->doJntImpedanceControl(_fri_inst->getMsrMsrJntPosition(),
				zero_vector, zero_vector, _joint_trq.data(), false);

	} else if (_current_control_mode == ControlModes::JointImpedanceCtrl) {
//	for (unsigned int i = 0; i < _joint_names.size(); ++i) {
//		double q = full_feedback->joint_feedback.angles[i];
//		double qd = position_controller->joint_cmd.angles[i];
//		double Kd = impedance_controller->joint_cmd.stiffness[i];
//		double qdot = full_feedback->joint_feedback.velocities[i];
//		double Dd = impedance_controller->joint_cmd.damping[i];
//		double tauoff = torque_controller->joint_cmd.torques[i];
//		double tau = -Kd * (q - qd) - Dd * qdot + tauoff;
//		_model->GetJoint(_joint_names[i])->SetForce(0, tau);
//	}
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
//TODO do handshake as separate procedure?
	while (_fri_inst->getQuality() < FRI_QUALITY::FRI_QUALITY_PERFECT) {
		_fri_inst->setToKRLInt(0, 1);
		_fri_inst->doDataExchange();
		last_quality = _fri_inst->getQuality();
		if (last_quality >= FRI_QUALITY_OK) {
			_fri_inst->setToKRLInt(1, 30);
			_fri_inst->doDataExchange();
		}
		_fri_inst->setToKRLReal(1, _fri_inst->getFrmKRLReal(1));
		_fri_inst->doDataExchange();
	}
	return true;
}
