
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

