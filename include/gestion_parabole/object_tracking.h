#ifndef OBJECT_TRACKING_H
#define OBJECT_TRACKING_H

template <class Context> class ObjectTracking : public Manoeuvre {
	private:
		timing_t t2;
		std::vector<double> kp,kv;
		Context ctx;
	public:
		ObjectTracking(Context ctx_, params_t p) : 
			ctx(ctx_),
			kp((const std::vector<double> &)(p["kp"])), 
			kv((const std::vector<double> &)(p["kv2"])), 
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
			guidage.qd.w = 1;
			guidage.type = 1;
			guidage.header.stamp = ros::Time::now();
			ctx.publish(guidage);

			ctx.sleep(t2);
		}
};

#endif
