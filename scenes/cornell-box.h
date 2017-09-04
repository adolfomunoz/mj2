#pragma once

#include <tracer/tracer.h>
namespace scenes {

tracer::Scene cornell_box() {
	tracer::Scene sol;
	std::list<tracer::Plane> planes;
	planes.push_back(tracer::Plane(Eigen::Vector3f( 0, 0,-1), Eigen::Vector3f( 0, 0, 1)));
	planes.push_back(tracer::Plane(Eigen::Vector3f( 0,-1, 0), Eigen::Vector3f( 0, 1, 0)));
	planes.push_back(tracer::Plane(Eigen::Vector3f( 0, 1, 0), Eigen::Vector3f( 0,-1, 0)));
	planes.push_back(tracer::Plane(Eigen::Vector3f(-1, 0, 0), Eigen::Vector3f( 1, 0, 0)));
	planes.push_back(tracer::Plane(Eigen::Vector3f( 1, 0, 0), Eigen::Vector3f(-1, 0, 0)));

	sol.push_back(std::make_shared<tracer::Pack<tracer::Plane,5>>(planes));

	std::list<tracer::Sphere> spheres;
	spheres.push_back(tracer::Sphere(Eigen::Vector3f( 0.5, -0.65,-0.2), 0.35));
	spheres.push_back(tracer::Sphere(Eigen::Vector3f(-0.5, -0.65, 0.5), 0.35));
	sol.push_back(std::make_shared<tracer::Pack<tracer::Sphere,2>>(spheres));
	
	return sol;
}

}