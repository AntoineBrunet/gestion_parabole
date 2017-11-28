#ifndef LARGE_MANEUVER_H
#define LARGE_MANEUVER_H

template <class Context> class LargeManeuver : public Manoeuvre {
	private:
		timing_t t1,t2,t3,t4,tf;
		std::vector<double> hd;
		std::vector<double> kv;
		std::vector<double> isat_inv;
		double ht;
		Context ctx;
	public:
		LargeManeuver(Context ctx_, params_t p, params_t m) : 
			ctx(ctx_),
			t1((timing_t)(m["t1"])),
			t2((timing_t)(m["t2"])),
			t3((timing_t)(m["t3"])),
			t4((timing_t)(m["t4"])),
			ht(p["h_toupie"]),
			isat_inv((const std::vector<double>&)(p["i_sat_inv"])),
			hd((const std::vector<double>&)(m["hd"])),
			kv((const std::vector<double> &)(p["kv1"])), 
			tf((timing_t)(p["tfin"])) {}
		virtual void start() override {
			cmg_msgs::Guidage guidage;
			guidage.a = {0,0,0};
			guidage.b = {0,0,0};
			guidage.kp = {0,0,0};
			std::copy(kv.begin(), kv.end(), guidage.kv.begin());
			guidage.qd.x = 0;
			guidage.qd.y = 0;
			guidage.qd.z = 0;
			guidage.qd.w = 1;
			guidage.type = 0;
			guidage.header.stamp = ros::Time::now();
			ctx.publish(guidage);
			ctx.sleep(t1);
			
			timing_t dt = t2-t1;
			moment_t hg = ctx.get_hg();
			guidage.a = {
				-isat_inv[0]*(ht*hd[0] - hg.vector.x)/dt,
				-isat_inv[1]*(ht*hd[1] - hg.vector.y)/dt,
				-isat_inv[2]*(ht*hd[2] - hg.vector.z)/dt
			};
			guidage.header.stamp = ros::Time::now();
			ctx.publish(guidage);
			ctx.sleep(dt);

			guidage.b = {
				guidage.a[0] * dt,
				guidage.a[1] * dt,
				guidage.a[2] * dt
			};
			guidage.a = {0,0,0};
			guidage.header.stamp = ros::Time::now();
			ctx.publish(guidage);
			dt = t3-t2;
			ctx.sleep(dt);
		
			dt = t4-t3;
			guidage.a = {
				-guidage.b[0] / dt,
				-guidage.b[1] / dt,
				-guidage.b[2] / dt
			};
			guidage.header.stamp = ros::Time::now();
			ctx.publish(guidage);
			ctx.sleep(dt);

			guidage.a = {0,0,0};
			guidage.b = {0,0,0};
			dt = tf - t4;
			guidage.header.stamp = ros::Time::now();
			ctx.publish(guidage);
			ctx.sleep(dt);
		}
};

#endif
