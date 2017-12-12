#ifndef _KINEMATIC_CHAIN_H_
#define _KINEMATIC_CHAIN_H_

// RST-RT includes
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/dynamics/JointImpedance.hpp>
#include <rst-rt/robot/JointState.hpp>

#include <friremote_rt.h>

#include <control_modes.h>
#include <fricomm_rt.h>

using namespace rstrt::kinematics;
using namespace rstrt::dynamics;
using namespace rstrt::robot;

/**
 * Hardcoded Ports.
 * TODO generate based on output of xbotcoremodel.
 */
typedef cogimon::jointCtrl<JointAngles> position_ctrl;
typedef cogimon::jointCtrl<JointImpedance> impedance_ctrl;
typedef cogimon::jointCtrl<JointTorques> torque_ctrl;

typedef cogimon::jointFeedback<JointState> full_fbk;


class KinematicChain {
public:
    /**
     * Contructor to set up a kinematic chain.
     * 
     * @param chain_name name of the chain.
     * @param joint_names list o joint names.
     * @param ports pointer to the ports of the enclosing component.
     * @param friInst remote interface of FRI.
     */
    KinematicChain(const std::string& chain_name,
                   const std::vector<std::string> &joint_names,
                   RTT::DataFlowInterface& ports,
                   friRemote* friInst);

    ~KinematicChain(){}

    std::string getKinematicChainName();
    unsigned int getNumberOfDOFs();
    std::string getCurrentControlMode();
    std::vector<std::string> getJointNames();
    std::vector<std::string> getControllersAvailable();
    bool initKinematicChain();
    bool resetKinematicChain();
    bool setControlMode(const std::string& controlMode);
    bool sense();
    void getCommand();
    void move();
    void stop();

    std::string printKinematicChainInformation();
    std::vector<RTT::base::PortInterface*> getAssociatedPorts();

    bool recieved, debug;

    boost::shared_ptr<position_ctrl> position_controller;
    boost::shared_ptr<impedance_ctrl> impedance_controller;
    boost::shared_ptr<torque_ctrl> torque_controller;

    boost::shared_ptr<full_fbk> full_feedback;
    
    // When false Gravity is removed from kuka
    // when true kuka compensates itself
    void setGravity(bool g);

    // if true zeros all output torques (grav comp mode)
    void setDebug(bool g);
    
    // set the fake impedance for debugging.
    void setTrqFakeImpedance(rstrt::dynamics::JointImpedance imp, bool fakeImpedance);
    
private:

    RTT::nsecs time_now, last_time;
    std::string _kinematic_chain_name;
    std::vector<std::string> _controllers_name;
    unsigned int _number_of_dofs;
    RTT::DataFlowInterface& _ports;

    std::vector<RTT::base::PortInterface*> _inner_ports;

    //gazebo::physics::ModelPtr _model;
    friRemote* _fri_inst;
    float* _last_pos;
    float* zero_vector;
    FRI_QUALITY last_quality;
    std::string _krc_ip;
    std::string _current_control_mode;
    //gazebo::physics::JointControllerPtr _gazebo_position_joint_controller;
    std::vector<std::string> _joint_names;
    std::map<std::string, int> _map_joint_name_index;
    Eigen::VectorXf _joint_pos, _joint_trq, _joint_stiff,_joint_damp;
    RTT::OutputPort<Eigen::Matrix<float,7,7>> output_M_port;
    RTT::OutputPort<rstrt::dynamics::JointTorques> estExtTorques_port;
    rstrt::dynamics::JointTorques estExtTorques;
    Eigen::Matrix<float,7,7> output_M_var;
    bool include_gravity;

    // torque mode joint impedance port
    RTT::InputPort<rstrt::dynamics::JointImpedance> in_trqModeJntImpedance_port;
    RTT::FlowStatus in_trqModeJntImpedance_flow;
    rstrt::dynamics::JointImpedance trqModeJntImpedance;

	std::vector<double> _initial_joints_configuration; /**< Vector that hold the initial joint angles. */

    bool setController(const std::string& controller_type);
    void setFeedBack();
    bool setJointNamesAndIndices();
    //bool initGazeboJointController();
    bool initKRC();
    std::vector<int> getJointScopedNames();
    void setInitialPosition(const bool use_model);
    void setInitialImpedance();

    int _friInternalCommandMode;

    bool _overrideFakeImpedance;

    RTT::OutputPort<rstrt::dynamics::JointTorques> gravity_port;
    rstrt::dynamics::JointTorques gravity_torques;
};

#endif
