#ifndef GESTION_PARABOLE_H
#define GESTION_PARABOLE_H

#include <stdexcept>
#include <algorithm>
#include "ros/ros.h"
#include "cmg_msgs/Guidage.h"
#include "geometry_msgs/Quaternion.h"
#include "json.hpp"

using json=nlohmann::json;

namespace fyt_par {
	typedef geometry_msgs::Quaternion quaternion_t;
	typedef ros::Duration timing_t;
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

	template <class Context> class ObjectTracking : public Manoeuvre {
		private:
			timing_t t2;
			std::vector<double> kp,kv;
			Context ctx;
		public:
			ObjectTracking(Context ctx_, params_t p) : 
				ctx(ctx_),
				kp((const std::vector<double> &)(p["kp"])), 
				kv((const std::vector<double> &)(p["kv"])), 
				t2((float)(p["tfin"])) {}
			virtual void start() override {
				cmg_msgs::Guidage guidage;
				guidage.a = {0,0,0};
				guidage.b = {0,0,0};
				std::copy(kp.begin(), kp.end(), guidage.kp.begin());
				std::copy(kv.begin(), kv.end(), guidage.kv.begin());
				guidage.qd.x = 0;
				guidage.qd.y = 0;
				guidage.qd.z = 0;
				guidage.qd.w = 0;
				guidage.object_tracking = true;
				ctx.publish(guidage);

				t2.sleep();
			}
	};

	template <class Context> class FixedPointing : public Manoeuvre {
		private:
			timing_t t1,t2;
			quaternion_t qdec;
			std::vector<double> kp, kv;
			Context ctx;
		public:
			FixedPointing(Context ctx_, params_t p, timing_t ts, quaternion_t qd) : 
				ctx(ctx_),
				kp((const std::vector<double> &)(p["kp"])), 
				kv((const std::vector<double> &)(p["kv"])), 
				t1(ts), t2((float)(p["tfin"])),
				qdec(qd) {}

			virtual void start() override {
				cmg_msgs::Guidage guidage;
				guidage.a = {0,0,0};
				guidage.b = {0,0,0};
				guidage.kp = {0,0,0};
				std::copy(kv.begin(), kv.end(), guidage.kv.begin());
				guidage.qd.x = 0;
				guidage.qd.y = 0;
				guidage.qd.z = 0;
				guidage.qd.w = 0;
				guidage.object_tracking = false;
				ctx.publish(guidage);
				std::cout << "Publishing 1st guidance order" << std::endl;

				if (t1 < t2) {
					t1.sleep();

					quaternion_t qm = ctx.get_qm();
					guidage.qd = qmult(qm, qdec);
					std::copy(kp.begin(), kp.end(), guidage.kp.begin());
					guidage.kv = {0,0,0};
					ctx.publish(guidage);
					std::cout << "Publishing 2nd guidance order" << std::endl;

					timing_t dt = t2-t1;
					dt.sleep();
				} else {
					t2.sleep();
				}
			}
	};

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
				} else {
					throw std::invalid_argument(type_name);
				}
			}
	};


}

#endif
