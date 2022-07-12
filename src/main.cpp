#include <iostream>
#include "rrt.hpp"
#include "optimizer.hpp"
#include "display.hpp"

int Node::MAXID = 0;
int Obs::MAXID = 0;

int main(int argc, char* argv[]) {
	int N = 10;
	int n_obs = 20;
	float scanner_range = 10.f;
	int RRT_ITER_MAX = 1000;
	float ts = 0.03333333f;
	//float ts = 0.01666666f;//0.05f;
	float lr = 1.738f;
	float v_i = 0.f;
	float head_i = deg_to_rad(45);	
	float obs_max_bounds[6] {-7.5f,27.5f,-7.5f,27.5f,0.2f,0.6f};

	Vec4f state_init {0.f, 0.f, v_i, head_i};
	RowVec2f v_init {0.f, 0.f};
	RowVec2f v_dest {15.f, 15.f};
	Display_Pangolin *dp = new Display_Pangolin(1920,1080,"s");
	
	World *world = new World();
	KF *kf = new KF("kalman_joseph");
	Li_Radar *scanner = new Li_Radar(scanner_range);
	ObjetFactory *factory = new ObjetFactory();
	factory->createNObstacles(n_obs, obs_max_bounds, world->obstacles());

	BRRTStar *rrt = new BRRTStar(v_init, v_dest, world->obstacles(), RRT_ITER_MAX);
	for (int i=0; i<10; i++) {
		if (rrt->build_status()==RRT::Status::REACHED)
			break;
		delete rrt;
		std::cout<<"reinstantiating rrtstar\n";
		rrt = new BRRTStar(v_init, v_dest, world->obstacles(), RRT_ITER_MAX);
	}
	std::vector<NodePtr> path_ = *rrt->path();
	Optimizer *opt = new Optimizer(N, ts, lr);
	std::vector<VehiclePtr> vehicles;
	factory->createVehicle(Objet::Shape::CIRCLE, ts, lr, state_init, vehicles);
	VehiclePtr vehicle = vehicles[0];
	std::vector<Vec4f> trajectory {state_init};
	
	Vec4f state = state_init;
	Vec2f pi = state.block(0,0,2,1);
	Vec2f pf {15.f, 15.f};
	Vec2f u_opt;
	int prev = 0;

	std::unordered_map<int, Vec4f> *xm;
	std::vector<ObsPtr> obstacles_in_range;
	while (!pangolin::ShouldQuit()) {
		obstacles_in_range.clear();
		if (!rrt->collision_free(pi))
			break;
		if (path_.size() < N+1) {
			delete opt;
			prev = path_.size()-1;
			opt = new Optimizer(path_.size()-1, ts, lr);
		} else if (prev != N && path_.size() >= N+1) {
			delete opt;
			prev = N;
			opt = new Optimizer(N, ts, lr);
		}
		scanner->scan(pi, world->obstacles(), obstacles_in_range);
		xm = kf->update_kalman_gain(obstacles_in_range, scanner, true);
		bool success = opt->optimize(state, &path_, obstacles_in_range, kf);
		u_opt = opt->input_opt();
		if (!success) {
			//std::cout<<"not success.. input opt is : "<<u_opt<<"\n";
			//u_opt = Vec2f(0.f,-1.5f);
		}
		world->update();
		state = vehicle->update(u_opt);	
		pi = state.block(0,0,2,1);
		trajectory.push_back(state);
		// BRRT Star Testing
		path_ = rrt->update(pi);
		// RRT Star Testing 
		//rrt->update(pi);
		//path_ = *rrt->path();
		dp->render(success, N, rrt, &path_, trajectory, state, opt->pred_states(), world->obstacles(), &obstacles_in_range, scanner_range, kf);
	}
	while (!pangolin::ShouldQuit()) {
		dp->render(false, N, rrt, &path_, trajectory, state, opt->pred_states(), world->obstacles(), &obstacles_in_range, scanner_range, kf);
	}
	return 0;
}



