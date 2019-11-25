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

TEST_CASE( "Intersection with triangle", "[triangle]" ) {
	tracer::Ray r(Eigen::Vector3f(0.0f,0.0f,2.0f), Eigen::Vector3f(0.0f,0.0f,-1.0f));
	tracer::Triangle t(
		Eigen::Vector3f(-1.0f,-1.0f,0.0f),
		Eigen::Vector3f( 1.0f,-1.0f,0.0f),
		Eigen::Vector3f( 0.0f, 1.0f,0.0f));
	std::optional<tracer::Hit> hit = t.trace(r);
	REQUIRE( hit );
	REQUIRE( hit->distance() == Approx(2.0f) );	
}

TEST_CASE( "Intersection with axis aligned box", "[box]" ) {
	tracer::Ray r(Eigen::Vector3f(0.0f,0.0f,2.0f), Eigen::Vector3f(0.0f,0.0f,-1.0f));
	tracer::AxisAlignedBox b(
		Eigen::Vector3f(-1.0f,-1.0f,-1.0f),
		Eigen::Vector3f(1.0f,1.0f,1.0f));
	std::optional<tracer::Hit> hit = b.trace(r);
	REQUIRE( hit );
	REQUIRE( hit->distance() == Approx(1.0f) );
}

TEST_CASE( "Intersection with list of planes", "[list][plane]" ) {
	tracer::Ray r(Eigen::Vector3f(0.0f,0.0f,2.0f), Eigen::Vector3f(0.0f,0.0f,-1.0f));
	std::list<tracer::Plane> ps;
	ps.push_back(tracer::Plane(Eigen::Vector3f::Random(), Eigen::Vector3f(0.0f,0.0f,0.0f)));
	for (int i = 0; i<10; ++i) 
			ps.push_back(tracer::Plane(Eigen::Vector3f::Random(), Eigen::Vector3f(0.0f,0.0f,-10.0f)));
	tracer::List<tracer::Plane> planes(ps);
	std::optional<tracer::Hit> hit = planes.trace(r);
	REQUIRE( hit );
	REQUIRE( hit->distance() == Approx(2.0f) );
}

TEST_CASE( "Intersection with list of boxes", "[list][box]" ) {
	tracer::Ray r(Eigen::Vector3f(0.0f,0.0f,2.0f), Eigen::Vector3f(0.0f,0.0f,-1.0f));
	std::list<tracer::AxisAlignedBox> ps;
	ps.push_back(tracer::AxisAlignedBox(
		Eigen::Vector3f(-1,-1,-1), Eigen::Vector3f(1,1,1)));
	for (int i = 0; i<10; ++i) 
			ps.push_back(tracer::AxisAlignedBox(Eigen::Vector3f(1,1,1), Eigen::Vector3f(i+2,i+2,i+2)));
	tracer::List<tracer::AxisAlignedBox> boxes(ps);
	std::optional<tracer::Hit> hit = boxes.trace(r);
	REQUIRE( hit );
	REQUIRE( hit->distance() == Approx(1.0f) );
}



