#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include <catch.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
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
	auto planes = tracer::list(
		tracer::Plane(Eigen::Vector3f::Random(), Eigen::Vector3f(0,0,-10)),
		tracer::Plane(Eigen::Vector3f::Random(), Eigen::Vector3f(0,0,-20)),
		tracer::Plane(Eigen::Vector3f::Random(), Eigen::Vector3f(0,0,0)),
		tracer::Plane(Eigen::Vector3f::Random(), Eigen::Vector3f(0,0,-30))
	);		
	std::optional<tracer::Hit> hit = planes.trace(r);
	REQUIRE( hit );
	REQUIRE( hit->distance() == Approx(2.0f) );
}

TEST_CASE( "Intersection with list of spheres", "[list][sphere]" ) {
	tracer::Ray r(Eigen::Vector3f(0.0f,0.0f,2.0f), Eigen::Vector3f(0.0f,0.0f,-1.0f));
	auto spheres = tracer::list(
		tracer::Sphere(Eigen::Vector3f(0,0,-10), 2),
		tracer::Sphere(Eigen::Vector3f(0,0,0), 1),
		tracer::Sphere(Eigen::Vector3f(0,0,-20), 3),
		tracer::Sphere(Eigen::Vector3f(0,0,0), 0.5)
	);	
	std::optional<tracer::Hit> hit = spheres.trace(r);
	REQUIRE( hit );
	REQUIRE( hit->distance() == Approx(1.0f) );
}

TEST_CASE( "Intersection with list of triangles", "[list][triangle]" ) {
	tracer::Ray r(Eigen::Vector3f(0.0f,0.0f,2.0f), Eigen::Vector3f(0.0f,0.0f,-1.0f));
	auto triangles = tracer::list(
		tracer::Triangle(Eigen::Vector3f(1,1,1), Eigen::Vector3f(1,3,1), Eigen::Vector3f(0,3,1)),
		tracer::Triangle(Eigen::Vector3f(-1,-1,0), Eigen::Vector3f(1,-1,0), Eigen::Vector3f(0,1,0)),
		tracer::Triangle(Eigen::Vector3f(-1,-1,0), Eigen::Vector3f(1,-1,0), Eigen::Vector3f(0,1,-10)),
		tracer::Triangle(Eigen::Vector3f(-1,-1,-20), Eigen::Vector3f(1,-1,-20), Eigen::Vector3f(0,1,-20))
	);	
	std::optional<tracer::Hit> hit = triangles.trace(r);
	REQUIRE( hit );
	REQUIRE( hit->distance() == Approx(2.0f) );
}

TEST_CASE( "Intersection with list of boxes", "[list][box]" ) {
	tracer::Ray r(Eigen::Vector3f(0.0f,0.0f,2.0f), Eigen::Vector3f(0.0f,0.0f,-1.0f));
	auto boxes = tracer::list(
		tracer::AxisAlignedBox(Eigen::Vector3f(-1,-1,-1), Eigen::Vector3f(1,1,1)),
		tracer::AxisAlignedBox(Eigen::Vector3f(-1,-1, 0), Eigen::Vector3f(1,1,1)),
		tracer::AxisAlignedBox(Eigen::Vector3f( 1, 1,-3), Eigen::Vector3f(3,3,3)),
		tracer::AxisAlignedBox(Eigen::Vector3f( 1, 1, 1), Eigen::Vector3f(3,3,3))
	);	
	std::optional<tracer::Hit> hit = boxes.trace(r);
	REQUIRE( hit );
	REQUIRE( hit->distance() == Approx(1.0f) );
}

TEST_CASE( "Intersection with pack of planes", "[pack][plane]" ) {
	tracer::Ray r(Eigen::Vector3f(0.0f,0.0f,2.0f), Eigen::Vector3f(0.0f,0.0f,-1.0f));
	auto planes = tracer::pack(
		tracer::Plane(Eigen::Vector3f::Random(), Eigen::Vector3f(0,0,-10)),
		tracer::Plane(Eigen::Vector3f::Random(), Eigen::Vector3f(0,0,-20)),
		tracer::Plane(Eigen::Vector3f::Random(), Eigen::Vector3f(0,0,0)),
		tracer::Plane(Eigen::Vector3f::Random(), Eigen::Vector3f(0,0,-30))
	);		
	std::optional<tracer::Hit> hit = planes.trace(r);
	REQUIRE( hit );
	REQUIRE( hit->distance() == Approx(2.0f) );
}

TEST_CASE( "Intersection with pack of spheres", "[pack][sphere]" ) {
	tracer::Ray r(Eigen::Vector3f(0.0f,0.0f,2.0f), Eigen::Vector3f(0.0f,0.0f,-1.0f));
	auto spheres = tracer::pack(
		tracer::Sphere(Eigen::Vector3f(0,0,-10), 2),
		tracer::Sphere(Eigen::Vector3f(0,0,0), 1),
		tracer::Sphere(Eigen::Vector3f(0,0,-20), 3),
		tracer::Sphere(Eigen::Vector3f(0,0,0), 0.5)
	);	
	std::optional<tracer::Hit> hit = spheres.trace(r);
	REQUIRE( hit );
	REQUIRE( hit->distance() == Approx(1.0f) );
}

TEST_CASE( "Intersection with pack of triangles", "[pack][triangle]" ) {
	tracer::Ray r(Eigen::Vector3f(0.0f,0.0f,2.0f), Eigen::Vector3f(0.0f,0.0f,-1.0f));
	auto triangles = tracer::pack(
		tracer::Triangle(Eigen::Vector3f(1,1,1), Eigen::Vector3f(1,3,1), Eigen::Vector3f(0,3,1)),
		tracer::Triangle(Eigen::Vector3f(-1,-1,0), Eigen::Vector3f(1,-1,0), Eigen::Vector3f(0,1,0)),
		tracer::Triangle(Eigen::Vector3f(-1,-1,0), Eigen::Vector3f(1,-1,0), Eigen::Vector3f(0,1,-10)),
		tracer::Triangle(Eigen::Vector3f(-1,-1,-20), Eigen::Vector3f(1,-1,-20), Eigen::Vector3f(0,1,-20))
	);	
	std::optional<tracer::Hit> hit = triangles.trace(r);
	REQUIRE( hit );
	REQUIRE( hit->distance() == Approx(2.0f) );
}

TEST_CASE( "Intersection with pack of boxes", "[pack][box]" ) {
	tracer::Ray r(Eigen::Vector3f(0.0f,0.0f,2.0f), Eigen::Vector3f(0.0f,0.0f,-1.0f));
	auto boxes = tracer::pack(
		tracer::AxisAlignedBox(Eigen::Vector3f(-1,-1,-1), Eigen::Vector3f(1,1,1)),
		tracer::AxisAlignedBox(Eigen::Vector3f(-1,-1, 0), Eigen::Vector3f(1,1,1)),
		tracer::AxisAlignedBox(Eigen::Vector3f( 1, 1,-3), Eigen::Vector3f(3,3,3)),
		tracer::AxisAlignedBox(Eigen::Vector3f( 1, 1, 1), Eigen::Vector3f(3,3,3))
	);	
	std::optional<tracer::Hit> hit = boxes.trace(r);
	REQUIRE( hit );
	REQUIRE( hit->distance() == Approx(1.0f) );
}

TEST_CASE( "Intersection with instance of sphere (translation)", "[instance][sphere][translation]" ) {
	tracer::Ray r(Eigen::Vector3f(0.0f,0.0f,2.0f), Eigen::Vector3f(0.0f,0.0f,-1.0f));
	tracer::Instance i1(Eigen::Translation3f(0.0f,0.0f,1.0f),tracer::Sphere(Eigen::Vector3f(0.0f,0.0f,0.0f), 1.0f));
	std::optional<tracer::Hit> hit = i1.trace(r);
	REQUIRE( hit );
	REQUIRE( hit->distance() == Approx(0.0f) );
	
	tracer::Instance i2(Eigen::Translation3f(2.0f,0.0f,0.0f),tracer::Sphere(Eigen::Vector3f(0.0f,0.0f,0.0f), 1.0f));
	hit = i2.trace(r);
	REQUIRE( !hit );
}


TEST_CASE( "Intersection with instance of sphere (scaling)", "[instance][sphere][scaling]" ) {
	tracer::Ray r(Eigen::Vector3f(0.0f,0.0f,2.0f), Eigen::Vector3f(0.0f,0.0f,-1.0f));
	tracer::Instance i1(Eigen::Vector3f(2.0f,2.0f,2.0f).asDiagonal(),tracer::Sphere(Eigen::Vector3f(0.0f,0.0f,0.0f), 1.0f));
	std::optional<tracer::Hit> hit = i1.trace(r);
	REQUIRE( hit );
	REQUIRE( hit->distance() == Approx(0.0f) );
}


TEST_CASE( "Intersection with instance of sphere (composition)", "[instance][sphere][translation][scaling]" ) {
	tracer::Ray r(Eigen::Vector3f(0.0f,0.0f,2.0f), Eigen::Vector3f(0.0f,0.0f,-1.0f));
	tracer::Instance i1(Eigen::Translation3f(0.0f,0.0f,1.0f)*Eigen::Vector3f(2.0f,2.0f,2.0f).asDiagonal(),tracer::Sphere(Eigen::Vector3f(0.0f,0.0f,0.0f), 0.5f));
	std::optional<tracer::Hit> hit = i1.trace(r);
	REQUIRE( hit );
	REQUIRE( hit->distance() == Approx(0.0f) );
	
	tracer::Instance i2(Eigen::Translation3f(2.0f,0.0f,0.0f)*Eigen::Vector3f(2.0f,2.0f,2.0f).asDiagonal(),tracer::Sphere(Eigen::Vector3f(0.0f,0.0f,0.0f), 0.5f));
	hit = i2.trace(r);
	REQUIRE( !hit );
}



