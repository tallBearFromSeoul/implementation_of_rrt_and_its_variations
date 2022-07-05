#include <random>
#include "graph.hpp"
#include "kdtree.hpp"
#include "vehicle.hpp"

typedef Eigen::Matrix2f Mat2f;

class RRT : public kdTree, public Graph {
	public:
		enum Status{
			NONE, TRAPPED, ADVANCED, REACHED, NO_NEIGHBORS
		};
	private:
	protected:
		const float PI = 3.145927;
		const float DEST_THRESH = 0.5f;
		const float COL_THRESH = 0.1f; 
		float SPEED_LIMIT = 1.0f;
		float STEER_LIMIT = 35.f;//40.f

		std::random_device _rd;
		std::mt19937 _gen;
		std::uniform_real_distribution<float> _uni_dis, _uni_dis_x, _uni_dis_y, _uni_bias_dis;
		std::normal_distribution<float> _norm_dis;
		std::vector<ObsPtr> *_obstacles;
		RowVec2f _v_init, _v_dest, _v_diff, _v_mid;
		Status _build_status=Status::NONE;
	public:
		Status build_status() {return _build_status;};
		RRT(const RowVec2f &__v_init, const RowVec2f &__v_dest, std::vector<ObsPtr> *__obstacles) : _v_init(__v_init), _v_dest(__v_dest), kdTree(__v_init.cols()), _obstacles(__obstacles) {
			_kd_root = std::make_shared<TreeNode>(__v_init, 0);
			_v_diff = (_v_dest - _v_init) / 2.f;
			_v_mid = (_v_dest + _v_init) / 2.f;
			_gen = std::mt19937(_rd());
			_uni_dis = std::uniform_real_distribution<float>(-1.0,1.0);
			_uni_dis_x = std::uniform_real_distribution<float>(std::min(_v_init(0),_v_dest(0)), std::max(_v_init(0),_v_dest(0)));
			_uni_dis_y = std::uniform_real_distribution<float>(std::min(_v_init(1),_v_dest(1)), std::max(_v_init(1),_v_dest(1)));
			_uni_bias_dis = std::uniform_real_distribution<float>(0.0,1.0);

			_norm_dis = std::normal_distribution<float>(0.f, 0.25f);
		};
		
		RRT(const RowVec2f &__v_init, const RowVec2f &__v_dest, std::vector<ObsPtr> *__obstacles, int __k) : _v_init(__v_init), _v_dest(__v_dest), kdTree(__v_init.cols()), _obstacles(__obstacles) {
			_kd_root = std::make_shared<TreeNode>(__v_init, 0);
			_v_diff = (_v_dest - _v_init) / 2.f;
			_v_mid = (_v_dest + _v_init) / 2.f;
			_gen = std::mt19937(_rd());
			_uni_dis = std::uniform_real_distribution<float>(-1.0,1.0);
			_uni_dis_x = std::uniform_real_distribution<float>(std::min(_v_init(0),_v_dest(0)), std::max(_v_init(0),_v_dest(0)));
			_uni_dis_y = std::uniform_real_distribution<float>(std::min(_v_init(1),_v_dest(1)), std::max(_v_init(1),_v_dest(1)));
			_uni_bias_dis = std::uniform_real_distribution<float>(0.0,1.0);
			_norm_dis = std::normal_distribution<float>(0.f, 0.25f);
			_build_status = build(__k);
		};
		virtual Status build(int __k) {
			Status status, latest_status;
			NodePtr last_node = nullptr;
			for (int i=0; i<__k; i++) {
				RowVec2f v_rand = random_config(last_node);
				status = extend(v_rand, last_node, latest_status);
				latest_status = status;
				if (status==Status::REACHED) {
					return status;
				}
			}
			return status;
		}
		
		Status extend(const RowVec2f &__v_rand, NodePtr &__last_node, Status latest_status) {
			NodePtr n_near, n_new;
			n_near = nearest(__v_rand);
			if (latest_status == Status::TRAPPED) {
				n_new = new_biased_config(n_near->val());
			} else {
				n_new = new_config(n_near->val());
			}
			if (!collision_free(n_new)) {
				return Status::TRAPPED;
			}
			__last_node = n_new;
			insert(_kd_root, n_new);
			if (_n_nodes == 0)
				add_edge(_kd_root, n_new);
			else
				add_edge(n_near, n_new);
			if ((n_new->val()-_v_dest).norm()<DEST_THRESH) {
				NodePtr n_dest = std::make_shared<Node>(_v_dest);
				add_edge(n_new, n_dest);
				dfs(_kd_root, n_dest);
				return Status::REACHED;
			} else {
				return Status::ADVANCED;
			}
		}
		
		bool collision_free(const NodePtr &__n) {
			bool res = true;
			RowVec2f __v = __n->val();
			for (int i=0; i<_obstacles->size(); i++) {
				res &= _obstacles->at(i)->collision_free(__v, COL_THRESH);
			}
			return res;
		}
		
		RowVec2f random_config() {
			RowVec2f res = RowVec2f::Zero();
			res(0) = _uni_dis_x(_gen);
			res(1) = _uni_dis_y(_gen);
			return res;
		}
		RowVec2f random_config(const Vec2f &__cur_p) {
			RowVec2f res = RowVec2f::Zero();
			res(0) = std::uniform_real_distribution<float>{std::min(__cur_p(0),_v_dest(0))-1.f, std::max(__cur_p(0),_v_dest(0))+1.f}(_gen);
			res(1) = std::uniform_real_distribution<float>{std::min(__cur_p(1),_v_dest(1))-1.f, std::max(__cur_p(1),_v_dest(1))+1.f}(_gen);
			return res;
		}

		RowVec2f random_config(const NodePtr &__last_node) {
			RowVec2f res = RowVec2f::NullaryExpr(1,2,[&](){return _uni_dis(_gen);});
			//if (__last_node != nullptr)
			//	res += __last_node->val();
			return res;
		}

		RowVec2f random_biased_config(const NodePtr &__last_node) {
			RowVec2f res = RowVec2f::NullaryExpr(1,2,[&](){return _uni_bias_dis(_gen);});
			res += __last_node->val();
			return res;
		}

		NodePtr new_biased_config(const RowVec2f &__v_near) {
			RowVec2f diff_normal = _v_dest-__v_near;
			diff_normal.normalize();
			float theta = (1-_norm_dis(_gen)) * STEER_LIMIT*1.5;
			Vec2f rotated = rotate(deg_to_rad(theta))*diff_normal.transpose();
			RowVec2f new_v = __v_near + SPEED_LIMIT*rotated.transpose();
			return std::make_shared<Node>(new_v);
		}

		NodePtr new_config(const RowVec2f &__v_near) {	
			RowVec2f diff_normal = _v_dest-__v_near;
			diff_normal.normalize();
			float theta = _uni_dis(_gen) * STEER_LIMIT;
			Vec2f rotated = rotate(deg_to_rad(theta))*diff_normal.transpose();
			RowVec2f new_v = __v_near + SPEED_LIMIT*rotated.transpose();
			return std::make_shared<Node>(new_v);
		}

		NodePtr new_config(const RowVec2f &__src, const RowVec2f &__dst) {
			RowVec2f diff_normal = __dst-__src;
			diff_normal.normalize();
			float theta = _uni_dis(_gen) * STEER_LIMIT;
			Vec2f rotated = rotate(deg_to_rad(theta))*diff_normal.transpose();
			RowVec2f new_v = __src + SPEED_LIMIT*rotated.transpose();
			return std::make_shared<Node>(new_v);

		}
		
		Mat2f rotate(float __theta) {
			Mat2f rot_mat {{cos(__theta), -sin(__theta)}, {sin(__theta), cos(__theta)}};
			return rot_mat;
		}

		float deg_to_rad(float __deg) {
			return PI*__deg/180.f;
		}
};

class RRTStar : public RRT {
	private:
		bool _bfs_path_found = false;
	protected:
		const float NEIGH_THRESH = 1.f;
		const int REBUILD_ITER_MAX = 50;
		std::unordered_map<int, float> _nid2cost_map;
		NodePtr n_dest;
	public:
		RRTStar(const RowVec2f &__v_init, const RowVec2f &__v_dest, std::vector<ObsPtr> *__obstacles, int __k) : RRT(__v_init, __v_dest, __obstacles) {
			_build_status = build(__k);
		}

		Status build(int __k) {
			Status latest_status = Status::NONE;
			NodePtr n_init = _kd_root;
			_nid2cost_map[_kd_root->id()] = 0.f;
			for (int i=0; i<__k; i++) {
				// Uniform Random Sampling of a vertex in the range of _v_init, and _v_dest.
				RowVec2f v_rand = random_config();
				// Obtain nearest node from the vertex through KD Tree.
				NodePtr n_near, n_new;
				n_near = nearest(v_rand);
				// Generate a new node from the nearest node 
				// where the new node will be stochastically rotated
				// and placed towards _v_dest
				// while conforming to the steer limit.
				n_new = RRT::new_config(n_near->val());
				// Collision check of the new node
				// if the check fails then just skip the iteration;
				if (!collision_free(n_new)) {
					n_new.reset();
					latest_status = Status::TRAPPED;
					continue;
				}
				// Obtain neighboring nodes withing the Ball of radius NEIGH_THRESH=1.f, 
				// from the node n_new using KD Tree.
				std::vector<NodePtr> nbs = neighbors(n_new, NEIGH_THRESH);
				
				NodePtr n_min;
				// Compute the n_min node,
				// which is the node that will be rewired such that
				// it will become the parent of the n_new node.
				if (nbs.size() == 0) {
					// If there is no neighboring nodes within the Ball of radius NEIGH_THRESH=1.f found,
					// the nearest node from the n_new node will be used as n_min.
					n_min = nearest(n_new->val());
				} else {
					// Compute the n_min node,
					// which is the node with the minimum cost out of the neighbors.
					n_min = choose_parent(nbs, n_near, n_new);
				}
				// Insert the n_new node into the KD Tree and the graph,
				// such that we can include the node into our search next iterations.
				insert(_kd_root, n_new);
				add_edge(n_min, n_new);
				// Update the cost of the n_new node,
				// which will just be the Sum of costs of n_min node 
				// and the norm of the difference of n_min and n_new vertices.
				_nid2cost_map[n_new->id()] = _nid2cost_map[n_min->id()] + (n_new->val()-n_min->val()).norm();
				// Perform the rewiring 
				rewire(nbs, n_min, n_new);
				
				if ((n_min->val()-_v_dest).norm()<DEST_THRESH) {
					n_dest = std::make_shared<Node>(_v_dest);
					add_edge(n_min, n_dest);
					_nid2cost_map[n_dest->id()] = _nid2cost_map[n_min->id()] + (_v_dest-n_min->val()).norm();
					dfs(n_init, n_dest);
					if (_path_found)
						return Status::REACHED;
				} 
				latest_status = Status::ADVANCED;
				//status = extend(v_rand, last_node, latest_status, _kd_root, false);
			}
			return latest_status;
		}

		void rewire(const std::vector<NodePtr> &__nbs, const NodePtr &__n_min, const NodePtr &__n_new) {
			for (const NodePtr &__nb : __nbs) {
				NodePtr n_p = RRT::new_config(__n_new->val());
				if (!collision_free(n_p)) {
					continue;
				}
				if ((n_p->val()-__n_new->val()).norm() + _nid2cost_map[__n_new->id()] < _nid2cost_map[__nb->id()]) {
					_nid2cost_map[__nb->id()] = (n_p->val()-__n_new->val()).norm() + _nid2cost_map[__n_new->id()];
					reconnect(__n_new, __nb);
				}
			}
			
		}

		NodePtr choose_parent(const std::vector<NodePtr> &__nbs, const NodePtr &__n_min, const NodePtr &__n_new) {
			float c_min = (__n_new->val()-__n_min->val()).norm() + _nid2cost_map[__n_min->id()];
			NodePtr z_min = __n_min;
			for (const NodePtr &__nb : __nbs) {
				NodePtr n_p = RRT::new_config(__nb->val());
				if (!collision_free(n_p)) {
					continue;
				}
				float c_p = (n_p->val()-__nb->val()).norm() + _nid2cost_map[__nb->id()];
				if (c_p < _nid2cost_map[__n_new->id()] && c_p < c_min) {
					c_min = c_p;
					z_min = __nb;
				}
			}
			_nid2cost_map[z_min->id()] = c_min; 
			return z_min;
		}

		void bfs_update_cost(const NodePtr &__n_cur, std::vector<NodePtr> __nbs) {
			_bfs_path_found = false;
			for (const NodePtr &__nb : __nbs) {
				_nid2cost_map[__nb->id()] = _nid2cost_map[__n_cur->id()] + (__nb->val()-__n_cur->val()).norm();
				reconnect(__n_cur, __nb);
			}	
			while (__nbs.size() > 0) {
				NodePtr __nb = __nbs[0];
				int gid_nb = _nid2gid_map[__nb->id()];
				for (int gid_e : _adj_list[gid_nb]) {
					if (gid_e == _nid2gid_map[n_dest->id()])
						_bfs_path_found = true;
					int nid_e = _gid2nid_map[gid_e];
					_nid2cost_map[nid_e] = _nid2cost_map[__nb->id()] + (__nb->val()-(_nid2n_map[nid_e])->val()).norm();
					__nbs.push_back(_nid2n_map[nid_e]);
				}
				__nbs.erase(__nbs.begin());
			}
		}

		Status update(const Vec2f &__cur_p) {
			Status latest_status;
			NodePtr n_cur, n_near;
			n_cur = std::make_shared<Node>(__cur_p);
			_nid2cost_map[n_cur->id()] = 0.f;
			add_node(n_cur);
			std::vector<NodePtr> nbs = neighbors(n_cur, NEIGH_THRESH*2.f);
			if (nbs.size() == 0) {
				n_near = nearest(__cur_p);
				nbs = {n_near};
			}
			dfs(n_cur, nbs, n_dest);
			if (_path_found)
				return Status::REACHED;
			//bfs_update_cost(n_cur, nbs);

			for (int i=0; i<REBUILD_ITER_MAX; i++) {
					RowVec2f v_rand = random_config(__cur_p);
					NodePtr n_near, n_new;
					n_near = nearest(v_rand);
					n_new = new_config(n_near->val());
					if (!collision_free(n_new)) {
						n_new.reset();
						latest_status = Status::TRAPPED;
						continue;
					}
					std::vector<NodePtr> nbs = neighbors(n_new, NEIGH_THRESH);
					NodePtr n_min;
					if (nbs.size() == 0) {
						n_min = nearest(n_new->val());
					} else {
						n_min = choose_parent(nbs, n_near, n_new);
					}
					insert(_kd_root, n_new);
					add_edge(n_min, n_new);
					_nid2cost_map[n_new->id()] = _nid2cost_map[n_min->id()] + (n_new->val()-n_min->val()).norm();
					rewire(nbs, n_min, n_new);

					//status = extend(v_rand, last_node, latest_status, n_cur, true);
					if (latest_status == Status::REACHED) {
						return latest_status;
					}
					latest_status = Status::ADVANCED;
				}
			
			nbs = neighbors(n_cur, NEIGH_THRESH*2.f);
			if (nbs.size() == 0) {
				n_near = nearest(__cur_p);
				nbs = {n_near};
			}
			dfs(n_cur, nbs, n_dest);
			//bfs_update_cost(n_cur, nbs);

			insert(_kd_root, n_cur);
			if (_bfs_path_found) {
				std::cout<<"bfs path found \n";
				dfs(n_cur,nbs,n_dest);
			}
			if (_path_found)
				return Status::REACHED;
			/*
			if (!_path_found) {
				std::cerr<<"update() : -> NO_PATH_ERROR\n";
				NodePtr n_new;
				std::vector<NodePtr> nbs_new;
				while (nbs_new.size() == 0) {
					n_new = new_config(last_node->val());
					if (!collision_free(n_new))
						continue;
					add_edge(last_node, n_new);
					
					insert(_kd_root, n_new);
					_nid2cost_map[n_new->id()] = _nid2cost_map[last_node->id()] + (last_node->val()-n_new->val()).norm();
					nbs_new = neighbors(n_new, NEIGH_THRESH);
					if (nbs_new.size() == 0) {
						NodePtr n_new_near = nearest(n_new->val());
						nbs_new = {n_new_near};
					}
					rewire(nbs_new, last_node, n_new); 
					last_node = n_new;
				}
			}
			*/
			/*
			for (const NodePtr &__nb : nbs) {
				last_node = n_cur;	
				for (int i=0; i<REBUILD_ITER_MAX; i++) {
					RowVec2f v_rand = random_config(last_node);
					status = extend(v_rand, last_node, latest_status, __nb, true);
					latest_status = status;
					if (status == RRT::Status::REACHED) {
						return status;
					}
				}	
			}
			*/
			std::cout<<"path found  : "<<_path_found<<"\n";
			return latest_status;
		}

};
/*
class BiRRTStar : public RRT {
	private:
		bool _bfs_path_found = false;
	protected:
		const float NEIGH_THRESH = 1.f;
		const int REBUILD_ITER_MAX = 30;
		std::pair<std::vector<NodePtr>, float> _best_path;
		std::unordered_map<int, float> _nid2cost_map;
		std::unordered_map<int, float> _nid2cost_map_B;
		NodePtr n_dest;
		TreeNodePtr _kd_root_B;
		kdTree *_kdtree_B;
		Graph *_graph_B;
	public:
		BiRRTStar(const RowVec2f &__v_init, const RowVec2f &__v_dest, std::vector<ObsPtr> *__obstacles, int __k) : RRT(__v_init, __v_dest, __obstacles) {
			n_dest = std::make_shared<Node>(_v_dest);
			_kd_root_B = std::make_shared<TreeNode>(n_dest, 0);
			
			_kdtree_B = new kdTree(_v_init.cols());
			_kdtree_B->set_root(_kd_root_B);
			_graph_B = new Graph();
			_nid2cost_map_B[n_dest->id()] = 0.f;
			_build_status = build(__k);
		}
		Status build(int __k) {
			RRT::Status status, latest_status;
			NodePtr last_node_A = nullptr, last_node_B = n_dest;
			_nid2cost_map[_kd_root->id()] = 0.f;
			for (int i=0; i<__k; i++) {
				if (latest_status == RRT::Status::TRAPPED)
					last_node_A = nullptr;
				RowVec2f v_rand = random_config();
				status = extend(v_rand, last_node_A, latest_status, _kd_root, n_new_id);
				latest_status = status;
				std::pair<std::vector<NodePtr>, float> __path = connect();
				if (__path.second < _best_path.second)
					_best_path = __path;
			}
			return status;
		}

		std::pair<std::vector<NodePtr>, float> connect(const NodePtr &__n_new_id) {
			NodePtr n_near_B, n_new_B;
			n_near_B = kdtree_B->nearest(n_new->val());
			n_new_B = RRT::new_config(n_near_B->val(), __n_new->val());
			std::vector<NodePtr> nbs_B = _kdtree_B->neighbors(n_new_B, NEIGH_THRESH*2.f);
			if (nbs_B.size() == 0)
				return {};
			NodePtr n_min_B = choose_parent_B(nbs_B, n_near_B, n_new_B, __n_new);
			_kdtree_B->insert(_kd_root_B, __n_new);
			_graph_B->add_edge(__n_new, n_min_B);
			_nid2cost_map_B[n_new_B->id()] = _nid2cost_map_B[n_min_B->id()] + (n_new_B->val()-n_min->val()).norm();

			std::vector<NodePtr> __path = dfs(graph(),_graph_B->graph());
			std::pair<std::vector<NodePtr>, float> best_path = std::make_pair(__path, _nid2cost_map[__n_new->id()]+_nid2cost_map_B[n_new_B->id()]);
			return best_path;
		}

		void bfs_update_cost(const NodePtr &__n_cur, std::vector<NodePtr> __nbs) {
			_bfs_path_found = false;
			for (const NodePtr &__nb : __nbs) {
				_nid2cost_map[__nb->id()] = _nid2cost_map[__n_cur->id()] + (__nb->val()-__n_cur->val()).norm();
				reconnect(__n_cur, __nb);
			}	
			while (__nbs.size() > 0) {
				NodePtr __nb = __nbs[0];
				int gid_nb = _nid2gid_map[__nb->id()];
				for (int gid_e : _adj_list[gid_nb]) {
					if (gid_e == _nid2gid_map[n_dest->id()])
						_bfs_path_found = true;
					int nid_e = _gid2nid_map[gid_e];
					_nid2cost_map[nid_e] = _nid2cost_map[__nb->id()] + (__nb->val()-(_nid2n_map[nid_e])->val()).norm();
					__nbs.push_back(_nid2n_map[nid_e]);
				}
				__nbs.erase(__nbs.begin());
			}
		}

		Status update(const Vec2f &__cur_p) {
			Status status, latest_status;
			NodePtr n_cur, n_near, last_node;
			
			n_cur = std::make_shared<Node>(__cur_p);
			_nid2cost_map[n_cur->id()] = 0.f;
			add_node(n_cur);
			n_near = nearest(__cur_p);
			std::vector<NodePtr> nbs = neighbors(n_cur, NEIGH_THRESH*1.f);
			bfs_update_cost(n_cur, nbs);

			insert(_kd_root, n_cur);
			if (_bfs_path_found) {
				std::cout<<"bfs path found \n";
				dfs(n_cur,nbs,n_dest);
			}
			if (_path_found)
				return RRT::Status::REACHED;
			last_node = n_cur;
			if (!_path_found) {
				std::cerr<<"update() : -> NO_PATH_ERROR\n";
				NodePtr n_new;
				std::vector<NodePtr> nbs_new;
				while (nbs_new.size() == 0) {
					n_new = new_config(last_node->val());
					if (!collision_free(n_new))
						continue;
					add_edge(last_node, n_new);
					
					insert(_kd_root, n_new);
					_nid2cost_map[n_new->id()] = _nid2cost_map[last_node->id()] + (last_node->val()-n_new->val()).norm();
					nbs_new = neighbors(n_new, NEIGH_THRESH*4.0f);
					if (nbs_new.size() != 0) {
						rewire(nbs_new, last_node, n_new); 
						continue;
					}
					last_node = n_new;
				}
				for (int i=0; i<REBUILD_ITER_MAX; i++) {
					if (latest_status == RRT::Status::TRAPPED)
						last_node = n_cur;
					RowVec2f v_rand = random_biased_config(last_node);
					status = extend(v_rand, last_node, latest_status, n_cur);
					latest_status = status;
					if (status == RRT::Status::REACHED) {
						return status;
					}
				}
			}
			return status;
		}


		Status extend(const RowVec2f &__v_rand, NodePtr &__last_node, Status latest_status, const NodePtr &__n_init, NodePtr &n_new=nullptr) {
			NodePtr n_near;
			n_near = nearest(__v_rand);
			n_new = RRT::new_config(n_near->val());
			if (!collision_free(n_new)) {
				return Status::TRAPPED;
			}
			std::vector<NodePtr> nbs = neighbors(n_new, NEIGH_THRESH*2.f);
			if (nbs.size() == 0)
				return Status::NO_NEIGHBORS;
			NodePtr n_min = choose_parent(nbs, n_near, n_new);
			__last_node = n_min;
			
			insert(_kd_root, n_new);
			add_edge(n_min, n_new);
			_nid2cost_map[n_new->id()] = _nid2cost_map[n_min->id()] + (n_new->val()-n_min->val()).norm();
			rewire(nbs, n_min, n_new);
						if ((n_min->val()-_v_dest).norm()<DEST_THRESH) {
				//n_dest = std::make_shared<Node>(_v_dest);
				add_edge(n_min, n_dest);
				_nid2cost_map[n_dest->id()] = _nid2cost_map[n_min->id()] + (_v_dest-n_min->val()).norm();
				dfs(__n_init, n_dest);
				if (_path_found)
					return Status::REACHED;
			} 
			return Status::ADVANCED;
		}

		void rewire(const std::vector<NodePtr> &__nbs, const NodePtr &__n_min, const NodePtr &__n_new) {
			for (const NodePtr &__nb : __nbs) {
				NodePtr n_p = RRT::new_config(__n_new->val());
				if (!collision_free(n_p)) {
					continue;
				}
				if ((n_p->val()-__n_new->val()).norm() + _nid2cost_map[__n_new->id()] < _nid2cost_map[__nb->id()]) {
					_nid2cost_map[__nb->id()] = (n_p->val()-__n_new->val()).norm() + _nid2cost_map[__n_new->id()];
					reconnect(__n_new, __nb);
				}
			}
			
		}

		NodePtr choose_parent(const std::vector<NodePtr> &__nbs, const NodePtr &__n_near, const NodePtr &__n_new) {
			float c_min = (__n_new->val()-__n_near->val()).norm() + _nid2cost_map[__n_near->id()];
			_nid2cost_map[__n_new->id()] = c_min;
			NodePtr z_min = __n_near;
			for (const NodePtr &__nb : __nbs) {
				NodePtr n_p = RRT::new_config(__nb->val());
				if (!collision_free(n_p)) {
					continue;
				}
				float c_p = (n_p->val()-__nb->val()).norm() + _nid2cost_map[__nb->id()];
				if (c_p < _nid2cost_map[__n_new->id()] && c_p < c_min) {
					c_min = c_p;
					z_min = __nb;
				}
			}
			return z_min;
		}

		NodePtr choose_parent_B(const std::vector<NodePtr> &__nbs_B, const NodePtr &__n_near_B, const NodePtr &__n_new_B, const NodePtr &__n_new_A) {
			float c_min = (__n_new_B->val()-__n_near_B->val()).norm() + _nid2cost_map_B[__n_near_B->id()];
			_nid2cost_map_B[__n_new_B->id()] = c_min;
			NodePtr z_min = __n_near_B;
			for (const NodePtr &__nb : __nbs_B) {
				NodePtr n_p = RRT::new_config(__nb->val(), __n_new_A->val());
				if (!collision_free(n_p))
					continue;
				float c_p = (n_p->val()-__nb->val()).norm() + _nid2cost_map_B[__nb->id()];
				if (c_p < _nid2cost_map_B[__n_new_B->id()] && c_p < c_min) {
					c_min = c_p;
					z_min = __nb;
				}
			}
			return z_min;
		}

};
*/

