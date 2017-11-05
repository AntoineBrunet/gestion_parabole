#include <iostream>
#include <fstream>

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
		params_t agcs;
		params_t paraboles;
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
			return qm;
		}
		void set_qm(const quaternion_t::ConstPtr& msg) {
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
				get_ready(agc);	
			}
			if (msg->state == STATE_MISS) {
				mf.run(agc, pbc);
			}
			if (msg->state == STATE_POST) {
				current_parabole++;
			}
		}
		void publish(const cmg_msgs::Guidage& guidage) const {
			pub_guidage.publish(guidage);
		}
};

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "gestion_parabole");
	json j;
	std::ifstream in(argv[1]);
	in >> j;

	Controller c(j);
/*	ManoeuvreFactory<const Controller&> mf(j["globals"], c);
	
	json paraboles = j["paraboles"];
	for (int i = 0; i < paraboles.size(); i++) {
		try {
			mf.run(paraboles[i]);
		} catch (std::exception & e) {
			std::cout << e.what() << std::endl;
		}
	}*/
}
