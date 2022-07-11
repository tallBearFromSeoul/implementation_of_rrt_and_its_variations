#include <iostream>
#include "rrt.hpp"
#include "optimizer.hpp"
#include "display.hpp"

int Node::MAXID = 0;
int Obs::MAXID = 0;

int main(int argc, char* argv[]) {
	int N = 20;
	int n_obs = 20;
	float scanner_range = 10.f;
	int RRT_ITER_MAX = 1000;
	float ts = 0.03333333f;
	//float ts = 0.01666666f;//0.05f;
	float lr = 1.738f;
	float v_i = 0.f;
	float head_i = deg_to_rad(45);	
	float obs_max_bounds[6] {-7.5f,27.5f,-7.5f,27.5f,-0.4f,0.4f};

	Vec4f state_init {0.f, 0.f, v_i, head_i};
	RowVec2f v_init {0.f, 0.f};
	RowVec2f v_dest {15.f, 15.f};
	Display_Pangolin *dp = new Display_Pangolin(1920,1080,"s");
	
	World *world = new World();
	KF *kf = new KF();
	Li_Radar *scanner = new Li_Radar(scanner_range);
	ObjetFactory *factory = new ObjetFactory();
	factory->createNObstacles(n_obs, obs_max_bounds, world->obstacles());
	/*
	RowVec2f op0 = {5,-4};
	RowVec2f op1 = {10,3};
	RowVec2f op2 = {8,11};
	RowVec2f op3 = {3,10};
	RowVec2f op4 = {15,20};
	RowVec2f op5 = {5,5};
	//std::vector<ObsPtr> obstacles;
	factory->createObstacle(Objet::Shape::CIRCLE, Objet::Type::DYNAMIC, op0, 1.f, obstacles, Objet::Behaviour::VERT);
	factory->createObstacle(Objet::Shape::CIRCLE, Objet::Type::DYNAMIC, op1, 1.f, obstacles, Objet::Behaviour::VERT);
	factory->createObstacle(Objet::Shape::CIRCLE, Objet::Type::DYNAMIC, op2, 1.f, obstacles, Objet::Behaviour::HORZ);
	factory->createObstacle(Objet::Shape::CIRCLE, Objet::Type::DYNAMIC, op3, 1.f, obstacles, Objet::Behaviour::DIAG);
	factory->createObstacle(Objet::Shape::CIRCLE, Objet::Type::DYNAMIC, op4, 1.f, obstacles, Objet::Behaviour::NEGDIAG);
	factory->createObstacle(Objet::Shape::CIRCLE, Objet::Type::DYNAMIC, op5, 1.f, obstacles, Objet::Behaviour::DIAG);
	int n_obs = obstacles.size();
	*/

	/*
	RRTStar *rrt = new RRTStar(v_init, v_dest, &obstacles, RRT_ITER_MAX);
	while (rrt->build_status()!=RRT::Status::REACHED) {
		delete rrt;
		std::cout<<"reinstantiating rrtstar\n";
		rrt = new RRTStar(v_init, v_dest, &obstacles, RRT_ITER_MAX);
	}
	*/
	BRRTStar *rrt = new BRRTStar(v_init, v_dest, world->obstacles(), RRT_ITER_MAX);
	while (rrt->build_status()!=RRT::Status::REACHED) {
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
	int prev = 0;

	std::unordered_map<int, Vec4f> *xm;

	while (!pangolin::ShouldQuit()) {
		if (!rrt->collision_free(pi))
			break;
		std::vector<ObsPtr> obstacles_in_range;
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
		xm = kf->update_recursive(obstacles_in_range, scanner);
		opt->optimize(state, &path_, obstacles_in_range, xm);
		Vec2f u_opt = opt->input_opt();
		world->update();
		state = vehicle->update(u_opt);	
		pi = state.block(0,0,2,1);
		trajectory.push_back(state);
		// BRRT Star Testing
		path_ = rrt->update(pi);
		// RRT Star Testing 
		//rrt->update(pi);
		//path_ = *rrt->path();
		dp->render(rrt, &path_, trajectory, state, opt->pred_states(), world->obstacles(), scanner_range, kf);
	}
	while (!pangolin::ShouldQuit()) {
		dp->render(rrt, &path_, trajectory, state, opt->pred_states(), world->obstacles(), scanner_range, kf);
	}
	return 0;
}



/*
	RowVec2f n0{0,0};
	RowVec2f n1{5,5};
	RowVec2f n2{9,6};
	RowVec2f n3{4,7};
	RowVec2f n4{8,1};
	RowVec2f n5{2,3};
	RowVec2f n7{3,3};
	std::vector<RowVec2f> values{n0,n1,n2,n3,n4,n5};//,n6,n7};
	std::vector<NodePtr> nodes;
	for (int i=0; i<values.size(); i++) {
		nodes.push_back(std::make_shared<Node>(values[i]));
	}
	
	// Testing RRT to find path from n0 to n1 while obstacles present
	//Obs obs(Objet::Shape::CIRCLE, n4, 1.f);
	//std::vector<Obs> obstacles{obs};
	//RRT rrt(n0, n1, &obstacles);
	//RRT::Status s = rrt.build(100);
	//if (s == RRT::Status::REACHED) {
	//	std::cout<<"REACHED\n";
	//}

	// Testing kdTree nearest and neighborhood function
	kdTree kdtree(nodes);
	kdtree.report();
	RowVec2f n6{3,3};
	NodePtr res = kdtree.nearest(n6);
	std::cout<<"The nearest to : "<<n6<<" is : "<<res->val()<<"\n";;
	std::vector<NodePtr> res2 = kdtree.neighbors(n7, 5.f);
	std::cout<<"The neighbors of : "<<n7<<" is : \n";
	for (int i=0; i<res2.size(); i++) {
		std::cout<<res2[i]->val()<<"\n";
	}
	return 0;
}
*/

