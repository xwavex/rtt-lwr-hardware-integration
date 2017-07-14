#ifndef RTT_LWR_ROBOT_HPP
#define RTT_LWR_ROBOT_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/Semaphore.hpp>

#include <Eigen/Dense>

#include <vector>

#include <Eigen/Core>
#include <time.h>
#include <rtt/os/TimeService.hpp>
#include <sstream>
#include <rtt/Logger.hpp>

#include <thread>
#include <memory>


#include <control_modes.h>
#include <kinematic_chain.h>
#include <boost/shared_ptr.hpp>
#include <urdf_parser/urdf_parser.h>

#include <srdfdom_advr/model.h>
#include <urdf/model.h>
#include <XBotCoreModel.h>

namespace cogimon {

class lwr_robot: public RTT::TaskContext {
public:
    lwr_robot(std::string const& name);
    bool configureHook();
    void updateHook();
    void WorldUpdateBegin();
    void WorldUpdateEnd();
    virtual ~lwr_robot() {}

protected:
    bool getModel(const std::string& model_name);
    bool setControlMode(const std::string& kinematic_chain, const std::string& controlMode);
    std::vector<std::string> getKinematicChains();
    std::string getControlMode(const std::string& kinematic_chain);
    std::vector<std::string> getControlAvailableMode(const std::string& kinematic_chain);
    std::string printKinematicChainInformation(const std::string& kinematic_chain);
    boost::shared_ptr<urdf::ModelInterface const> model;

    bool loadURDFAndSRDF(const std::string& URDF_path, const std::string& SRDF_path);
    std::map<std::string, std::vector<std::string> > getKinematiChainsAndJoints();
    bool resetModelConfiguration();
    void setGravity(const std::string& kinematic_chain,const bool g);


    /**
     * Provides the joint name to index mapping for other components to retrieve.
     * If there isn't such an port (portName) existing, or used in an kinematic chain,
     * the call will return an empty map. Otherwise it will contain the mapping.
     */
    //std::map<std::string, int> getJointMappingForPort(std::string portName);


//    gazebo::physics::Joint_V gazebo_joints_;
    std::map <std::string, urdf::JointSharedPtr> urdf_joints_;
//    gazebo::physics::Link_V model_links_;
    std::map <std::string, urdf::LinkSharedPtr> urdf_links_;

    std::map<std::string, boost::shared_ptr<KinematicChain>> kinematic_chains;
    friRemote* remote;
	std::string ip_addr;

    bool _models_loaded;
    XBot::XBotCoreModel _xbotcore_model;
std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it;

private:
    bool is_configured;
};

}
#endif
