#include <iostream>
#include <fstream>
#include <mutex>

#include "gestion_parabole/gestion_parabole.h"
#include "cmg_msgs/State.h"
#include "cmg_msgs/SpeedList.h"
#include "cmg_msgs/Speed.h"
#include "cmg_msgs/GimbalTarget.h"
#include "fyt_mae/fyt_commons.h"

using namespace fyt_par; 

class Controller {
	private:
		ros::NodeHandle n;
		ros::Publisher pub_guidage, pub_fw, pub_gi;
		ros::Subscriber sub_qm, sub_state;
		ManoeuvreFactory<const Controller&> mf;
		quaternion_t qm;
		mutable std::mutex qm_mut;
		params_t agcs;
		params_t paraboles;
		bool alert, running;
		unsigned int current_parabole;
	public:
		Controller(params_t conf) : 
			n("~"), 
			mf(*this), 
			agcs(conf["ag_configs"]), 
			paraboles(conf["paraboles"])
		{
			pub_guidage = n.advertise<cmg_msgs::Guidage>("guidage", 1);
			pub_fw = n.advertise<cmg_msgs::SpeedList>("fw_cmd", 5);
			pub_gi = n.advertise<cmg_msgs::GimbalTarget>("gi_cmd", 5);
			sub_qm = n.subscribe("qm", 5, &Controller::set_qm, this);
			sub_state = n.subscribe("states", 1, &Controller::state_cb, this);
			current_parabole = 0;
		}
		quaternion_t get_qm() const {
			std::unique_lock<std::mutex> lock(qm_mut);
			return qm;
		}
		void set_qm(const quaternion_t::ConstPtr& msg) {
			std::unique_lock<std::mutex> lock(qm_mut);
			qm = *msg;
		}
		void get_ready(params_t agc) {
			cmg_msgs::SpeedList msg_fw;
			double speed = agc["vitesse_toupie"];
			for (int id : agc["ags"]) {
				cmg_msgs::Speed spd;
				spd.id = id;
				spd.speed = speed;
				msg_fw.speeds.push_back(spd);
			}
			pub_fw.publish(msg_fw);
			cmg_msgs::GimbalTarget msg_gi;
			msg_gi.mode = 0;
			for (int i = 0; i < agc["init_pos"].size(); i++) {
				msg_gi.positions.push_back(agc["init_pos"][i]);
			}
			pub_gi.publish(msg_gi);
		}
		void state_cb(const cmg_msgs::State::ConstPtr& msg) {
			params_t pbc = paraboles[current_parabole];
			int curr_agc = pbc["ag_config"];
			params_t agc = agcs[curr_agc];
			if (msg->state == STATE_READY) {
				ROS_INFO("Loading ag config number %d",curr_agc);
				get_ready(agc);	
			}
			if (msg->state == STATE_MISS) {
				if (!running) {
					alert = false;
					running = true;
					ROS_INFO("Starting maneuver");
					mf.run(agc, pbc);
					if (alert) {
						ROS_WARN("Mission was interupted (went to safe state)");
					}
					running = false;
				} else {
					ROS_ERROR("New mission started but previous one was not done.");
				}
			}
			if (msg->state == STATE_POST) {
				current_parabole++;
			}
			if (msg->state == STATE_SAFE) {
				ROS_INFO("Going to safe state, alerting current maneuver");
				alert = true;
			}
		}
		void publish(const cmg_msgs::Guidage& guidage) const {
			pub_guidage.publish(guidage);
		}
		bool sleep(timing_t t) const {
			int poll_freq = 3;
			int max = t * poll_freq + 1;
			ros::Rate r(poll_freq);
			for (int i = 1; (i < max) && (!alert); i++) {
				ros::spinOnce();
				r.sleep();
				ROS_INFO("tic tac %d/%d",i,max);
			}
			return !alert;
		}
};

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "gestion_parabole");
	json j;
	std::ifstream in(argv[1]);
	in >> j;

	Controller c(j);
	ros::spin();
}
