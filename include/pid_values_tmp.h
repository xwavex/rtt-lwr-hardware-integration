#ifndef _PID_VALUES_TMP_
#define _PID_VALUES_TMP_
#include <string>

/**
 * This class is used for hard-coding the kinematic-chain and impedance parameters.
 * TODO replace by xbotcoremodel, to make it conform to the simulation case.
 */
class hardcoded_chains {
public:
	hardcoded_chains() {
		std::vector<std::pair<std::string,int>> joints;

		joints.push_back(std::pair<std::string,int>("lwr_arm_0_joint",0));
		joints.push_back(std::pair<std::string,int>("lwr_arm_1_joint",1));
		joints.push_back(std::pair<std::string,int>("lwr_arm_2_joint",2));
		joints.push_back(std::pair<std::string,int>("lwr_arm_3_joint",3));
		joints.push_back(std::pair<std::string,int>("lwr_arm_4_joint",4));
		joints.push_back(std::pair<std::string,int>("lwr_arm_5_joint",5));
		joints.push_back(std::pair<std::string,int>("lwr_arm_6_joint",6));
		// TODO read with xbotcoremodel! Don't call it "upper"!!!
		map_chains_joints.insert(
				std::pair<std::string, std::pair<std::string,std::vector<std::pair<std::string,int>>>>("upper",
						std::pair<std::string,std::vector<std::pair<std::string,int>>>("192.168.0.21",joints)));
		joints.clear();

	}
	std::map<std::string, std::pair<std::string, std::vector<std::pair<std::string,int>>>> map_chains_joints;

};


class hardcoded_impedance {
public:
	hardcoded_impedance() {
		impedance["lwr_arm_0_joint"] = std::pair<double, double>(800., 30.);
		impedance["lwr_arm_1_joint"] = std::pair<double, double>(800., 30.);
		impedance["lwr_arm_2_joint"] = std::pair<double, double>(800., 30.);
		impedance["lwr_arm_3_joint"] = std::pair<double, double>(800., 30.);
		impedance["lwr_arm_4_joint"] = std::pair<double, double>(800., 30.);
		impedance["lwr_arm_5_joint"] = std::pair<double, double>(800., 30.);
		impedance["lwr_arm_6_joint"] = std::pair<double, double>(800., 30.);
	}

	/**
	 * @brief impedance, std::pair<stiffness, damping>
	 */
	std::map<std::string, std::pair<double, double>> impedance;

};

#endif
