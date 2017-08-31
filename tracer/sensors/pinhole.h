#pragma once

#include "../ray.h"

namespace tracer {


class Pinhole {
	Eigen::Matrix4f transform_; //left = first column, up = second column, front = third column, origin = fourth column	

public:
	Pinhole(const Eigen::Vector3f& origin, const Eigen::Vector3f& front, const Eigen::Vector3f& up, const Eigen::Vector3f& left) :
		transform_(Eigen::Matrix4f::Identity()) {

		transform_.block<3,1>(0,0) = left;
		transform_.block<3,1>(0,1) = up;
		transform_.block<3,1>(0,2) = front;
		transform_.block<3,1>(0,3) = origin;
	}

	Pinhole(const Eigen::Vector3f& origin, const Eigen::Vector3f& front, const Eigen::Vector3f& up) :
		Pinhole(origin, front, up, up.cross(front).normalized()*up.norm()) { }


	const Eigen::Matrix4f& transform() const { return transform_; }

	Ray ray(float u, float v) const {
	       	//Between -1 and 1 each for the projection plane (-1, -1) top left corner
		return Ray(transform().block<3,1>(0,3),
			(-u)*transform().block<3,1>(0,0) +
			(-v)*transform().block<3,1>(0,1) +
			  transform().block<3,1>(0,2));
	}
};

}


