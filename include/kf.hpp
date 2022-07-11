#ifndef KF_HPP
#define KF_HPP
#include "grv.hpp"
#include <iostream>
#include <unordered_map>
#include <Eigen/Dense>
#include <fstream>

class KF {
	private:
		Vec4f _mu;
		Mat4f _cov;
		Mat4f _A;
		Mat4f _H;
		Vec4f _xm0;
		Mat4f _Pm0;
		std::unordered_map<int, Vec4f> _xm;
		std::unordered_map<int, Mat4f> _Pm;
		GRV *_process_noise;
	public:
		Vec4f xm(int __obs_id) {return _xm[__obs_id];}
		std::unordered_map<int, Vec4f> *xm() {return &_xm;}
		std::ofstream kf_output;

		KF() {
			_mu = Vec4f::Zero();
			_cov << 0.015f,0.f,0.f,0.f,
					 		0.f,0.015f,0.f,0.f,
							0.f,0.f,0.0075f,0.f,
							0.f,0.f,0.f,0.0075f;
			_A << 1.f,0.f,1.f,0.f,
						0.f,1.f,0.f,1.f,
						0.f,0.f,0.f,0.f,
						0.f,0.f,0.f,0.f;
			_H = Mat4f::Identity();
			//_Pm0 = Mat4f::Ones();//Zero();
			_Pm0 << 0.08f,0.f,0.f,0.f,
					 		0.f,0.08f,0.f,0.f,
							0.f,0.f,0.005f,0.f,
							0.f,0.f,0.f,0.005f;
			_process_noise = new GRV(_mu, _cov);
			kf_output = std::ofstream("../sample_output/kf_output.csv");
			kf_output << "obs_id,true_value,meas_value,corr_value,diff(%)\n";
		};

		//There are two types of algorithms :
		// - using recursion
		// - using Joseph's form of covariance matrix
		// implement both and compare.
		//
		
		// we will initially assume the obstacle to follow the linear system dynamics equations such that : 
		//  - obstacles do not accelerate and maintain constant velocity decided during instantiation.
		//  - they do not change direction of velocity as well.
		// Under these assumptions or settings, we can define the system dynamics of an obstacle as :
		//   dx/dt = v_x
		//   dy/dt = v_y
		// 	 X = {x,y,v_x,v_y}T
		// 	 dX/dt = {X(2), X(3), 0, 0}T
		// 	 or,
		// 	 A = dX/dt = {{1,0,1,0},
		// 	 					    {0,1,0,1},
		// 	 					    {0,0,0,0},
		// 	 					    {0,0,0,0}}
		//   measurement variable z without noise is : 
		//   z = {x,y,v_x,v_y}T
		//   z = H*X
		//   H = {{1,0,0,0},
		//        {0,1,0,0},
		//        {0,0,1,0},
		//        {0,0,0,1}}

		// With Gaussian noises v and w present, 
		// X(k) = A*x(k-1) + v(k-1)
		// z(k) = H*x(k) + w(k)

		// where,
		// v(k) -> sensor noise
		// w(k) -> process noise
		
		std::unordered_map<int, Vec4f> *update_recursive(const std::vector<ObsPtr> &__obstacles_in_range, Li_Radar *__lidar) {
			for (const ObsPtr &__obs : __obstacles_in_range) {
				update_recursive(__obs, __lidar);
			}
			return &_xm;
		}
		
		void update_recursive(const ObsPtr &__obs, Li_Radar *__lidar) {
			Vec4f v, xp, w, zbar;
			Mat4f Pp;
			v = __obs->sensor_noise();
			w = _process_noise->sample();
			zbar = _H*(_A*__obs->state() + v) + w;
			if (_xm.count(__obs->id())) {
				xp = _A*_xm[__obs->id()];
				Pp = _A*_Pm[__obs->id()]*_A.transpose() + __lidar->cov();
				_xm[__obs->id()] =  xp + Pp*_H.transpose() * (_H*Pp*_H.transpose() + _cov).inverse() * (zbar - _H*xp);
				_Pm[__obs->id()] = Pp - Pp*_H.transpose() * (_H*Pp*_H.transpose() + _cov).inverse() * _H*Pp;
			} else {
				_xm0 = Vec4f::Zero();
				xp = _A*_xm0;
				Pp = _A*_Pm0*_A.transpose() + __lidar->cov();
				_xm[__obs->id()] = xp + Pp*_H.transpose() * (_H*Pp*_H.transpose() + _cov).inverse() * (zbar - _H*xp);
				_Pm[__obs->id()] = Pp - Pp*_H.transpose() * (_H*Pp*_H.transpose() + _cov).inverse() * _H*Pp;
		}
		kf_output << __obs->id()<<","<<__obs->state().transpose()<<","<<zbar.transpose()<<","<<_xm[__obs->id()].transpose()<<","<<((_xm[__obs->id()] - __obs->state()).array()*100.f / __obs->state().array()).transpose()<<"\n";
		/*
		std::cout<<"obs id :"<<__obs->id()<<"\n";
		std::cout<<"true value : "<<__obs->state().transpose()<<"\n";
		std::cout<<"measured value : "<<zbar.transpose()<<"\n";
		std::cout<<"corrected value : "<<_xm[__obs->id()].transpose()<<"\n";
		std::cout<<"differece (%) : "<<((_xm[__obs->id()] - __obs->state()).array()*100.f / __obs->state().array()).transpose()<<"\n";
		*/
	}
};
#endif
