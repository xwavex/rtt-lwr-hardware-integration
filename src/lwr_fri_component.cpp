
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>

//#include <Eigen/Dense>
#include <boost/graph/graph_concepts.hpp>
#include <rtt/os/Timer.hpp>
#include <rtt/os/TimeService.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/atomic.hpp>
#include <friremote_rt.h>
//#include <friudp_rt.h>

#include <rci/dto/JointAngles.h>
#include <rci/dto/JointTorques.h>
#include <rci/dto/JointVelocities.h>
#define l(lvl) RTT::log(lvl) << "[" << this->getName() << "] "

class LWRFriComponent: public RTT::TaskContext {
public:

	LWRFriComponent(std::string const& name) :
			RTT::TaskContext(name), steps_rtt_(0), steps_gz_(0), rtt_done(
			true), gazebo_done(false), new_data(true), cnt_lock_(100), last_steps_rtt_(
					0), set_new_pos(false), nb_no_data_(0), set_brakes(false), nb_static_joints(
					0), // HACK: The urdf has static tag for base_link, which makes it appear in gazebo as a joint
			last_gz_update_time_(0), nb_cmd_received_(0), sync_with_cmds_(true) {
		
		this->ports()->addPort("JointPositionCommand",
				port_JointPositionCommand).doc(
				"Input for JointPosition-cmds from Orocos to Gazebo world.");
		this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc(
				"Input for JointTorque-cmds from Orocos to Gazebo world.");
		this->ports()->addPort("JointVelocityCommand",
				port_JointVelocityCommand).doc(
				"Input for JointVelocity-cmds from Orocos to Gazebo world.");

		this->ports()->addPort("JointVelocity", port_JointVelocity).doc(
				"Output for JointVelocity-fbs from Gazebo to Orocos world.");
		this->ports()->addPort("JointTorque", port_JointTorque).doc(
				"Output for JointTorques-fbs from Gazebo to Orocos world.");
		this->ports()->addPort("JointPosition", port_JointPosition).doc(
				"Output for JointPosition-fbs from Gazebo to Orocos world.");

		this->provides("debug")->addAttribute("jnt_pos", jnt_pos_);
		this->provides("debug")->addAttribute("jnt_vel", jnt_vel_);
		this->provides("debug")->addAttribute("jnt_trq", jnt_trq_);
		this->provides("debug")->addAttribute("gz_time", gz_time_);
		this->provides("debug")->addAttribute("write_duration",
				write_duration_);
		this->provides("debug")->addAttribute("read_duration", read_duration_);
		this->provides("debug")->addAttribute("rtt_time", rtt_time_);
		this->provides("debug")->addAttribute("steps_rtt", steps_rtt_);
		this->provides("debug")->addAttribute("steps_gz", steps_gz_);
		this->provides("debug")->addAttribute("period_sim", period_sim_);
		this->provides("debug")->addAttribute("period_wall", period_wall_);

		this->provides("misc")->addAttribute("urdf_string", urdf_string);
		tempCount =100;
	}

	virtual void updateHook() {
		do {
		rtt_time_now_ = RTT::os::TimeService::ticks2nsecs(
							RTT::os::TimeService::Instance()->getTicks());
		//Standard FRI!
		res = -1;
		res = friInst->doReceiveData();
		friInst->setToKRLInt(0,1);
		lastQuality = friInst->getQuality();
		if(lastQuality >= FRI_QUALITY_OK){
			friInst->setToKRLInt(1,30);
		}
		friInst->setToKRLReal(1,friInst->getFrmKRLReal(1));
		if(lastQuality >= FRI_QUALITY_PERFECT){
			#ifdef DEBUG
			RTT::log(RTT::Error)<<"CONNECTED TO ROB!\n"<<RTT::endlog();
			#endif
		//END STANDARD
		//TODO check for fri quality!!
		//float pos[7] = ;
		
			for (unsigned j = 0; j < 7; j++) {
				#ifdef DEBUG
				RTT::log(RTT::Error)<<"UPDATE JNT_DATA!\n"<<RTT::endlog();
				#endif
				jnt_pos_->setFromRad(j,
					friInst->getMsrMsrJntPosition()[j]);
				#ifdef DEBUG
				RTT::log(RTT::Error)<<"UPDATE JNT_DATA! TIME NOW: \n"<<(rtt_time_now_)<< " TIME PAST: "<<rtt_last_clock<< " TIMEDIFF: "<<(rtt_time_now_-rtt_last_clock)<<RTT::endlog();
				#endif
				jnt_vel_->setFromRad_s(j, (friInst->getMsrMsrJntPosition()[j] - jnt_pos_last->rad(j))/((rtt_time_now_-rtt_last_clock)*1E-9));
				jnt_pos_last->setFromRad(j,jnt_pos_->rad(j));

				jnt_trq_->setFromNm(j, friInst->getMsrJntTrq()[j]);
			}
		
			updateData();
			
		}
		if ((nb_cmd_received_ == 0 && data_fs == RTT::NoData)
					|| jnt_pos_fs == RTT::NewData)
			break;

			

		if (rtt_time_now_ != rtt_last_clock && data_fs != RTT::NewData)
			#ifdef DEBUG
			RTT::log(RTT::Debug) << getName() << " "
					<< "Waiting for UpdateHook at " << rtt_time_now_
					<< " v:" << nb_cmd_received_ << data_fs
					<< RTT::endlog();
			#endif
			rtt_last_clock = rtt_time_now_;
			//while no new data for jnt_trqs, and no other commands recieved
			//maybe actually turn off for asynchronise control?
		} while (!(RTT::NewData == data_fs && nb_cmd_received_)
				&& sync_with_cmds_ && lastQuality < FRI_QUALITY_OK);
		// Increment simulation step counter (debugging)
		//steps_gz_++;

		// Get the RTT and gazebo time for debugging purposes
		rtt_time_ = RTT::os::TimeService::ticks2nsecs(
						RTT::os::TimeService::Instance()->getTicks());
		//TODO get total time maybe?

		if(lastQuality == FRI_QUALITY_PERFECT){
			switch (data_fs) {
			// Not Connected
			case RTT::NoData:
				set_brakes = true;

				break;

				// Connection lost
			case RTT::OldData:
				if (data_timestamp == last_data_timestamp && nb_no_data_++ >= 2){
					set_brakes = true;
				}
				break;

			// OK
			case RTT::NewData:
				set_brakes = false;
				if (nb_no_data_-- <= 0)
					nb_no_data_ = 0;
				break;
			}
		#ifdef DEBUG
		RTT::log(RTT::Error) << "Brakes?: " << set_brakes << RTT::endlog();
		#endif
		// Copy Current joint pos in case of brakes
//		if (!set_brakes)
//			for (unsigned j = 0; j < 7; j++)
//				jnt_pos_brakes_->setFromRad(j, jnt_pos_->rad(j));

		// Force Joint Positions in case of a cmd
		if (set_new_pos) {//TODO check control schemes!
			#ifdef DEBUG
			RTT::log(RTT::Error) << "set_new_pos = true" << RTT::endlog();
			#endif
			// Update specific joints regarding cmd
			for (unsigned j = 0; j < 7; j++) {
				jnt_pos_robot[j] = jnt_pos_cmd_->rad(j);
			}
			friInst->doPositionControl(jnt_pos_robot,false);
			// Aknowledge the settings
			set_new_pos = false;

		} else if (set_brakes) {
			//TODO implement brakes by going to position ctrl mode and giving same configuration

		} else if(friInst->getCurrentControlScheme()==3 && data_fs==RTT::NewData){//TODO define control schemes!
			#ifdef DEBUG
			RTT::log(RTT::Error)<<friInst->getCurrentControlScheme()<<" :TORQUE COMMAND!\n"<<RTT::endlog();
			#endif			
			// Write command
			// Update specific joints regarding cmd
			for (unsigned j = 0; j < 7; j++) {
				//tempCount to attempt to stop interpolation error, need a better solution
				if(tempCount==0){
				thau[j]=jnt_trq_cmd_->Nm(j); //-friInst->getGrav()[j]; //attempted to remove gravity from controller, highly dependant on the quality of the model though so might be better to avoid.
				}
				else{
				   tempCount--;
				}
				q[j]=jnt_pos_->rad(j);
				
			}
			friInst->doJntImpedanceControl(q, NULL, NULL, thau);
		}else if(friInst->getCurrentControlScheme()!=3){//Changes control scheme to jntImpedance need to have position and other schemes too!
friInst->setToKRLInt(1,30);
		}	}
		friInst->doSendData();
		last_data_timestamp = data_timestamp;

	}

	virtual bool configureHook() {		
		if (!(port_JointPositionCommand.connected()
				&& port_JointTorqueCommand.connected()
				&& port_JointVelocityCommand.connected() &&port_JointPosition.connected()&&port_JointVelocity.connected() && port_JointTorque.connected())) {
			return false;
		}
		friInst = new friRemote(49939, "192.168.0.21",getActivity()->thread()->getTask());
		lastQuality = FRI_QUALITY_BAD;
		RTT::log(RTT::Info)<<"CONFIGURE HOOK!\n"<<RTT::endlog();
		//possibly read and handshake with KRC here until perfect quality?

		jnt_pos_cmd_ = rci::JointAngles::create(7, 0.0);
		jnt_pos_ = rci::JointAngles::create(7, 0.0);
		jnt_pos_last = rci::JointAngles::create(7, 0.0);

		jnt_trq_cmd_ = rci::JointTorques::create(7, 0.0);
		jnt_trq_ = rci::JointTorques::create(7, 0.0);

		jnt_vel_cmd_ = rci::JointVelocities::create(7, 0.0);
		jnt_vel_ = rci::JointVelocities::create(7, 0.0);

		jnt_pos_brakes_ = rci::JointAngles::create(7, 0.0);

		port_JointPosition.setDataSample(jnt_pos_);
		port_JointVelocity.setDataSample(jnt_vel_);
		port_JointTorque.setDataSample(jnt_trq_);

		last_update_time_ = RTT::os::TimeService::Instance()->getNSecs(); //rtt_rosclock::rtt_now(); // still needed??
		#ifdef DEBUG
		RTT::log(RTT::Warning) << "Done configuring gazebo" << RTT::endlog();
		#endif		
		return true;
	}

	void updateData() {		
		// Increment simulation step counter (debugging)
		steps_rtt_++;

		// Get command from ports
		data_fs = port_JointTorqueCommand.read(jnt_trq_cmd_);

		jnt_pos_fs = port_JointPositionCommand.read(jnt_pos_cmd_);


		if (jnt_pos_fs == RTT::NewData) {
			//set_new_pos = true; // TODO remove!
		}

		data_timestamp = new_pos_timestamp =
				RTT::os::TimeService::Instance()->getNSecs();

		//read_duration_ = RTT::os::TimeService::Instance()->secondsSince(read_start);

		// Write state to ports
		RTT::os::TimeService::ticks write_start =
				RTT::os::TimeService::Instance()->getTicks();

		port_JointVelocity.write(jnt_vel_);
		port_JointPosition.write(jnt_pos_);
		port_JointTorque.write(jnt_trq_);

		write_duration_ = RTT::os::TimeService::Instance()->secondsSince(
				write_start);

		switch (data_fs) {
		case RTT::OldData:
			break;
		case RTT::NewData:
			#ifdef DEBUG
			RTT::log(RTT::Error) << getName() << " " << data_fs << " at "
					<< data_timestamp << RTT::endlog();
			#endif
			nb_cmd_received_++;
			last_timestamp = data_timestamp;
			break;
		case RTT::NoData:
			nb_cmd_received_ = 0;
			break;
		}
		#ifdef DEBUG
		RTT::log(RTT::Error)<<"END UPDATE!\n"<<RTT::endlog();
		#endif
	}

protected:

	//! Synchronization ??

	std::vector<int> joints_idx;

	std::vector<std::string> joint_names_;

	RTT::InputPort<rci::JointAnglesPtr> port_JointPositionCommand;
	RTT::InputPort<rci::JointTorquesPtr> port_JointTorqueCommand;
	RTT::InputPort<rci::JointVelocitiesPtr> port_JointVelocityCommand;

	RTT::OutputPort<rci::JointAnglesPtr> port_JointPosition;
	RTT::OutputPort<rci::JointTorquesPtr> port_JointTorque;
	RTT::OutputPort<rci::JointVelocitiesPtr> port_JointVelocity;

	RTT::FlowStatus jnt_pos_fs, data_fs;
	rci::JointAnglesPtr jnt_pos_cmd_, jnt_pos_, jnt_pos_last;
	rci::JointTorquesPtr jnt_trq_, jnt_trq_cmd_;
	rci::JointVelocitiesPtr jnt_vel_, jnt_vel_cmd_;
	rci::JointAnglesPtr jnt_pos_brakes_;

	//! RTT time for debugging
	double rtt_time_;
	//! Gazebo time for debugging
	double gz_time_;
	double wall_time_;
double rtt_time_now_ = 0;
		double rtt_last_clock = 0;

	RTT::nsecs last_gz_update_time_, new_pos_timestamp;
	RTT::Seconds gz_period_;
	RTT::Seconds gz_duration_;

	RTT::nsecs last_update_time_;
	RTT::Seconds rtt_period_;
	RTT::Seconds read_duration_;
	RTT::Seconds write_duration_;

	int steps_gz_;
	int steps_rtt_, last_steps_rtt_, nb_no_data_;
//	unsigned int n_joints_;
	int cnt_lock_;
	double period_sim_;
	double period_wall_;
	boost::atomic<bool> new_data, set_new_pos;
	boost::atomic<bool> rtt_done, gazebo_done;

	RTT::nsecs last_data_timestamp, data_timestamp, last_timestamp;bool set_brakes;
	int nb_static_joints;

	int nb_cmd_received_;bool sync_with_cmds_;

	// contains the urdf string for the associated model.
	std::string urdf_string;


	//JOSH STUFF!!!
	friRemote* friInst;
	std::string ip_left;
	FRI_QUALITY lastQuality;
	int tempCount;
	int res;
	float thau[7];
	float q[7];
	float jnt_pos_robot[7];
};

ORO_LIST_COMPONENT_TYPE(LWRFriComponent)
ORO_CREATE_COMPONENT_LIBRARY();

