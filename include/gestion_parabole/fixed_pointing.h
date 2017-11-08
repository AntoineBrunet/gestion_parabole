#ifndef FIXED_POINTING_H
#define FIXED_POINTING_H

template <class Context> class FixedPointing : public Manoeuvre {
	private:
		timing_t t1,t2;
		quaternion_t qdec;
		std::vector<double> kp, kv1, kv2;
		Context ctx;
	public:
		FixedPointing(Context ctx_, params_t p, timing_t ts, quaternion_t qd) : 
			ctx(ctx_),
			kp((const std::vector<double> &)(p["kp"])), 
			kv1((const std::vector<double> &)(p["kv1"])), 
			kv2((const std::vector<double> &)(p["kv2"])), 
			t1(ts), t2((float)(p["tfin"])),
			qdec(qd) {}

		virtual void start() override {
			cmg_msgs::Guidage guidage;
			guidage.a = {0,0,0};
			guidage.b = {0,0,0};
			guidage.kp = {0,0,0};
			std::copy(kv1.begin(), kv1.end(), guidage.kv.begin());
			guidage.qd.x = 0;
			guidage.qd.y = 0;
			guidage.qd.z = 0;
			guidage.qd.w = 0;
			guidage.object_tracking = false;
			ctx.publish(guidage);
			std::cout << "Publishing 1st guidance order" << std::endl;

			if (t1 < t2) {
				if (!ctx.sleep(t1)) { return; }

				quaternion_t qm = ctx.get_qm();
				guidage.qd = qmult(qm, qdec);
				std::copy(kp.begin(), kp.end(), guidage.kp.begin());
				std::copy(kv2.begin(), kv2.end(), guidage.kv.begin());
				ctx.publish(guidage);
				std::cout << "Publishing 2nd guidance order" << std::endl;

				timing_t dt = t2-t1;
				ctx.sleep(dt);
			} else {
				ctx.sleep(t2);
			}
		}
};
#endif
