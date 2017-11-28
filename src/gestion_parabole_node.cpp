#include <iostream>
#include <fstream>

#include "gestion_parabole/gestion_parabole.h"
#include "gestion_parabole/SetId.h"
#include "cmg_msgs/State.h"
#include "cmg_msgs/SpeedList.h"
#include "cmg_msgs/Speed.h"
#include "cmg_msgs/AGConfig.h"
#include "cmg_msgs/Signal.h"
#include "fyt_mae/fyt_commons.h"
#include "sensor_msgs/Imu.h"

using namespace fyt_par; 

class Controller {
	private:
		ros::NodeHandle n;
		ros::Publisher pub_guidage, pub_agc, pub_fw, pub_sig;
		ros::Subscriber sub_qm, sub_state, sub_hg;
		ros::ServiceServer pid_serv;
		ManoeuvreFactory<const Controller&> mf;
		quaternion_t qm;
		moment_t hg;
		params_t agcs;
		params_t paraboles;
		bool alert, ended, running;
		unsigned int current_parabole;
		std::vector<cmg_msgs::Speed> prev_speeds;
	public:
		Controller(params_t conf) : 
			n("~"), 
			mf(*this), 
			prev_speeds(6),
			agcs(conf["ag_configs"]), 
			paraboles(conf["paraboles"])
		{
			pub_guidage = n.advertise<cmg_msgs::Guidage>("/parabola/guidage", 1);
			pub_agc = n.advertise<cmg_msgs::AGConfig>("/parabola/agconfig", 1);
			pub_sig = n.advertise<cmg_msgs::Signal>("/mae/signal", 1);
			pub_fw = n.advertise<cmg_msgs::SpeedList>("/fw/cmd", 5);
			sub_qm = n.subscribe("/imu/filtre", 5, &Controller::set_qm, this);
			sub_hg = n.subscribe("/boucle/hg", 1, &Controller::set_hg, this);
			sub_state = n.subscribe("/mae/state", 1, &Controller::state_cb, this);
			pid_serv = n.advertiseService("/parabola/set_id", &Controller::set_id, this);
			for (int i = 0; i < 6; i++) {
				prev_speeds[i].id = i;
				prev_speeds[i].speed = 0;
			}
			current_parabole = 0;
			running = false;
			alert = false;
			ended = false;
		}
		bool set_id(gestion_parabole::SetId::Request &req, gestion_parabole::SetId::Response &res) {
			ROS_INFO("Asked parabola %d/%d", req.id, paraboles.size());
			if (current_parabole != req.id && req.id < paraboles.size()) {
				ROS_INFO("Changing to parabola %d", req.id);
				current_parabole = req.id;
				params_t pbc = paraboles[req.id];
				int curr_agc = pbc["ag_config"];
				params_t agc = agcs[curr_agc];
				alert = false;
				ended = false;
				ROS_INFO("Loading ag config number %d",curr_agc);
				get_ready(agc);	
				return true;
			}
			return false;
		}
		quaternion_t get_qm() const {
			return qm;
		}
		void set_qm(const sensor_msgs::Imu::ConstPtr& msg) {
			qm = msg->orientation;
		}
		moment_t get_hg() const {
			return hg;
		}
		void set_hg(const moment_t & msg) {
			hg = msg;
		}
		void get_ready(params_t agc) {
			cmg_msgs::SpeedList msg_fw;
			msg_fw.speeds = prev_speeds;
			double speed = agc["vitesse_toupie"];
			cmg_msgs::AGConfig msg_agc;
			msg_agc.header.stamp = ros::Time::now();
			msg_agc.htoupie = agc["h_toupie"];
			msg_agc.tpara = agc["tfin"];
			msg_agc.id_para = current_parabole;
			for (int i = 0; i < msg_agc.running.size(); i++) {
				msg_agc.running[i] = false;
				msg_agc.init_pos[i] = agc["init_pos"][i];
			}
			for (int id : agc["ags"]) {
				msg_agc.running[id-1] = true;
				if (msg_fw.speeds[id-1].speed != speed) {
					msg_fw.speeds[id-1].speed = speed;
					msg_fw.header.stamp = ros::Time::now();
					pub_fw.publish(msg_fw);
					if (!sleep(2)) { return; }
				}
			}
			for (int i = 0; i < msg_agc.running.size(); i++) {
				if (!msg_agc.running[i] && (msg_fw.speeds[i].speed != 0.)) {
					msg_fw.speeds[i].speed = 0.;
				}
			}
			msg_fw.header.stamp = ros::Time::now();
			pub_fw.publish(msg_fw);
			prev_speeds = msg_fw.speeds;

			pub_agc.publish(msg_agc);

		}
		void state_cb(const cmg_msgs::State::ConstPtr& msg) {
			params_t pbc = paraboles[current_parabole];
			int curr_agc = pbc["ag_config"];
			params_t agc = agcs[curr_agc];
			if (msg->state == STATE_READY) {
				alert = false;
				ended = false;
				ROS_INFO("Loading ag config number %d",curr_agc);
				get_ready(agc);	
			}
			if (msg->state == STATE_MISS) {
				if (!running) {
					running = true;
					ROS_INFO("Starting maneuver");
					mf.run(agc, pbc);
					if (alert) {
						ROS_WARN("Mission was interupted (went to safe state)");
					}
					cmg_msgs::Guidage gui;
					gui.type = 2;
					gui.header.stamp = ros::Time::now();
					publish(gui);
					if (!ended) {
						cmg_msgs::Signal sig;
						sig.signal = SIG_END;
						pub_sig.publish(sig);
					}
					running = false;
				} else {
					ROS_ERROR("New mission started but previous one was not done.");
				}
			}
			if (msg->state == STATE_POST) {
				ended = true;
				current_parabole++;
				if (current_parabole >= paraboles.size()) {
					current_parabole = 0;
				}
			}
			if (msg->state == STATE_SAFE) {
				ROS_INFO("Going to safe state, alerting current maneuver");
				alert = true;
				ROS_INFO("Shutting down flywheels");
				cmg_msgs::SpeedList msg_fw;
				msg_fw.speeds = prev_speeds;
				for (int id = 0; id < 6; id++) {
					msg_fw.speeds[id].speed = 0;
				}
				msg_fw.header.stamp = ros::Time::now();
				pub_fw.publish(msg_fw);
				prev_speeds = msg_fw.speeds;
			}
		}
		void publish(const cmg_msgs::Guidage& guidage) const {
			pub_guidage.publish(guidage);
		}
		bool sleep(timing_t t) const {
			int poll_freq = 3;
			int max = t * poll_freq + 1;
			ros::Rate r(poll_freq);
			for (int i = 1; (i < max) && !(alert || ended); i++) {
				ros::spinOnce();
				r.sleep();
			//	ROS_INFO("tic tac %d/%d",i,max);
			}
			return !(alert || ended);
		}
};

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "gestion_parabole");
	std::string input_file;
	ros::param::get("/gestion_parabole/parabola_list", input_file);
	std::cout << "Parabola list is in: " << input_file << std::endl;
	json j;
	std::ifstream in(input_file);
	in >> j;

	Controller c(j);
	ros::spin();
}
