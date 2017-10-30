#include <iostream>
#include <fstream>

#include "gestion_parabole/gestion_parabole.h"

using namespace fyt_par; 

class Controller {
	private:
		ros::NodeHandle n;
		ros::Publisher pub;
		ros::Subscriber sub;
		quaternion_t qm;
	public:
		Controller() : n("~") {
			pub = n.advertise<cmg_msgs::Guidage>("guidage", 1);
			sub = n.subscribe("qm", 5, &Controller::set_qm, this);
		}
		quaternion_t get_qm() const {
			return qm;
		}
		void set_qm(const quaternion_t::ConstPtr& msg) {
			qm = *msg;
		}
		void publish(const cmg_msgs::Guidage& guidage) const {
			pub.publish(guidage);
		}
};

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "gestion_parabole");
	json j;
	std::ifstream in(argv[1]);
	in >> j;

	Controller c;
	ManoeuvreFactory<const Controller&> mf(j["globals"], c);
	
	json paraboles = j["paraboles"];
	for (int i = 0; i < paraboles.size(); i++) {
		try {
			mf.run(paraboles[i]);
		} catch (std::exception & e) {
			std::cout << e.what() << std::endl;
		}
	}
}
