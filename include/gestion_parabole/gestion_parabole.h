#ifndef GESTION_PARABOLE_H
#define GESTION_PARABOLE_H

#include <stdexcept>
#include <algorithm>
#include "ros/ros.h"
#include "cmg_msgs/Guidage.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "json.hpp"

using json=nlohmann::json;

namespace fyt_par {
	typedef geometry_msgs::Quaternion quaternion_t;
	typedef geometry_msgs::Vector3::ConstPtr moment_t;
	typedef int timing_t;
	typedef const json & params_t;

	class Manoeuvre {
		public:
			virtual void start() = 0;
	};

	quaternion_t qmult(quaternion_t a, quaternion_t b) {
		quaternion_t res;
		// TODO : implement
		res.x = 0;
		res.y = 0;
		res.z = 0;
		res.w = 0;
		return res;
	}
	constexpr unsigned int str2int(const char* str, int h = 0)
	{
		return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
	}
	
#include "gestion_parabole/fixed_pointing.h"
#include "gestion_parabole/object_tracking.h"
#include "gestion_parabole/large_maneuver.h"

	template <class Context> class ManoeuvreFactory {
		private:
			Context ctx;
			template <class T> quaternion_t make_quaternion(T tbl) {
				quaternion_t res;
				res.x = tbl[0];
				res.y = tbl[1];
				res.z = tbl[2];
				res.w = tbl[3];
				return res;
			}
		public:
			ManoeuvreFactory(Context ctx_) :
				ctx(ctx_) {}

			void run(params_t agc, params_t desc) {
				std::string type_name = desc["type"];
				std::cout << "Parabol type is " << type_name << std::endl;
				int type = str2int(type_name.c_str());
				if (type == str2int("FP")) {
					timing_t ts((float)(desc["t1"]));
					quaternion_t qd = make_quaternion<json>(desc["qc"]);
					FixedPointing<Context> fp(ctx, agc, ts, qd);
					fp.start();
				} else if (type == str2int("OT")) {
					ObjectTracking<Context> ot(ctx, agc);
					ot.start();
				} else if (type == str2int("MA")) {
					LargeManeuver<Context> ma(ctx, agc, desc);
					ma.start();
				} else {
					throw std::invalid_argument(type_name);
				}
			}
	};


}

#endif
