#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include <catch.hpp>
#include <tracer/tracer.h>

TEST_CASE( "Intersection with plane", "[plane]" ) {
	tracer::Ray r(Eigen::Vector3f(0.0f,0.0f,2.0f), Eigen::Vector3f(0.0f,0.0f,-1.0f));
	tracer::Plane p(Eigen::Vector3f::Random(), Eigen::Vector3f(0.0f,0.0f,0.0f));
	std::optional<tracer::Hit> hit = p.trace(r);
	REQUIRE( hit );
	REQUIRE( hit->distance() == Approx(2.0f) );
}

TEST_CASE( "Intersection with sphere", "[sphere]" ) {
	tracer::Ray r(Eigen::Vector3f(0.0f,0.0f,2.0f), Eigen::Vector3f(0.0f,0.0f,-1.0f));
	tracer::Sphere p(Eigen::Vector3f(0.0f,0.0f,0.0f), 1.0f);
	std::optional<tracer::Hit> hit = p.trace(r);
	REQUIRE( hit );
	REQUIRE( hit->distance() == Approx(1.0f) );
}




