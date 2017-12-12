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
    void stopHook();
    virtual ~lwr_robot() {}

protected:
    /**
     * TODO: IS THIS FUNCTION STILL USED?
     */
    bool getModel(const std::string& model_name);

    /**
     * Set the control mode for a specific kinematic chain.
     * 
     * @param kinematic_chain which will be switched to a control mode.
     * @param controlMode supported control mode for the specific chain.
     * 
     * @return successful.
     */
    bool setControlMode(const std::string& kinematic_chain, const std::string& controlMode);

    /**
     * Get the available kinematic chains names.
     * 
     * @return list of chain names.
     */
    std::vector<std::string> getKinematicChains();

    /**
     * Get the currently active control mode name for a particular kinematic chain.
     * 
     * @param kinematic_chain selected kinematic chain.
     * 
     * @return currently active control mode name.
     */
    std::string getControlMode(const std::string& kinematic_chain);

    /**
     * Get the available control mode names for a particular kinematic chain.
     * 
     * @param kinematic_chain selected kinematic chain.
     * 
     * @return list of available control mode names.
     */
    std::vector<std::string> getControlAvailableMode(const std::string& kinematic_chain);

    /**
     * Print information regarding a particular kinematic chain.
     * 
     * @param kinematic_chain selected kinematic chain.
     * 
     * @return detailed information
     */
    std::string printKinematicChainInformation(const std::string& kinematic_chain);

    /**
     * Parses the URDF and SRDF from file.
     * 
     * ONLY indirectly called in @see addChain().
     * 
     * @param URDF_path file path for URDF.
     * @param SRDF_path file path for SRDF.
     * 
     * @return successful.
     */
    bool loadURDFAndSRDF(const std::string& URDF_path, const std::string& SRDF_path);
    std::map<std::string, std::vector<std::string> > getKinematiChainsAndJoints();

    /**
     * Reset the kinematic chains.
     * 
     * @return successful.
     */
    bool resetModelConfiguration();

    /**
     * Enable to include the gravity for a specific kinematic chain.
     * 
     * @param kinematic_chain which should or should not consider gravity.
     * @param g use gravity or not.
     */
    void setGravity(const std::string& kinematic_chain, const bool g);

    /**
     * Enable debug mode for a specific kinematic chain.
     * 
     * @param kinematic_chain which should or should not use the debug mode.
     * @param debug use debug mode or not.
     */
    void setDebug(const std::string& kinematic_chain, const bool debug);

    /**
     * Initialize the specific chains by loading from URDF and SRDF.
     * 
     * @param name of the kinematic chain.
     * @param robot_ip IP of the remote robot.
     * @param robot_port Port of the remote robot.
     * @param URDF_path path to the URDF.
     * @param SRDF_path path to the SRDF.
     * 
     * @return successful.
     */
    bool addChain(std::string name, std::string robot_ip, int robot_port, const std::string& URDF_path, const std::string& SRDF_path); 
    
    void setTrqFakeImpedance(const std::string& kinematic_chain, const rstrt::dynamics::JointImpedance imp, bool fakeImpedance);

    /**
     * Provides the joint name to index mapping for other components to retrieve.
     * If there isn't such an port (portName) existing, or used in an kinematic chain,
     * the call will return an empty map. Otherwise it will contain the mapping.
     */
    //std::map<std::string, int> getJointMappingForPort(std::string portName);


    std::map <std::string, urdf::JointSharedPtr> urdf_joints_;                            /**< Model joints loaded from URDF. */
    std::map <std::string, urdf::LinkSharedPtr> urdf_links_;                              /**< Model links loaded from URDF. */

    std::map<std::string, boost::shared_ptr<KinematicChain>> kinematic_chains;            /**< Map that hold the kinematic chain objects. */
    friRemote* remote;                                                                    /**< Pointer to the FRI interface. */
	std::string ip_addr;                                                                  /**< IP address of the host PC. */

    bool _models_loaded;                                                                  /**< Variable to check if the model is loaded. */
    boost::shared_ptr<urdf::ModelInterface const> model;                                  /**< URDF model pointer. */
    XBot::XBotCoreModel _xbotcore_model;                                                  /**< Library object for loading the kinematic chains,
                                                                                               and parameters from URDF and SRDF. */
    std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it;                /**< Iterator for the loaded kinematic chains. */

private:
    bool is_configured;                                                                   /**< Variable to check if the component is configured. */
};

}
#endif
