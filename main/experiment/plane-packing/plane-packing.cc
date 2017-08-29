#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include <tuple>
#include <limits>
#include <list>
#include <array>
#include <iomanip>

#include <tracer/hit.h>
#include <tracer/ray.h>
#include <tracer/list.h>
#include <tracer/primitives/plane.h>

using namespace tracer; 

template<int N>
class PlanePack {
	Eigen::Matrix<float,N,3> normals_;
	Eigen::Matrix<float,N,1> distances_;
	std::array<Plane,N> planes_;
	unsigned int n_;
	
public:
	PlanePack() : n_(0) {}
	
	constexpr const Eigen::Matrix<float,N,3> normals() const noexcept { return normals_; }
	constexpr const Eigen::Matrix<float,N,1> distances() const noexcept { return distances_; }
	constexpr const std::array<Plane,N> planes() const noexcept { return planes_; }
	
	void push_back(const Plane& p) {
		assert(n_<N);
		normals_.row(n_) = p.normal();
		distances_[n_] = p.distance();
		planes_[n_]=p;
		++n_;
	}
};

std::optional<Hit> trace(const Ray& ray, const Object& object) noexcept {
	return object.trace(ray);
}

template<int N>
std::optional<Hit> trace(const Ray& ray, const PlanePack<N>& pack) noexcept {	
	Eigen::Matrix<float,N,1> distances = -(pack.normals() * ray.direction()).cwiseInverse().cwiseProduct(pack.normals() * ray.origin() + pack.distances());

	std::optional<Hit> hit; Ray r = ray;
//	std::cerr<<distances<<std::endl;
	for (unsigned int i = 0; i<N; ++i) {
		if (r.in_range(distances[i])) {
			hit = Hit(distances[i]);
			r.set_range_max(hit->distance());
		}
	}
	return hit;
};

template<typename Object>
std::tuple<std::optional<Hit>, float> time_per_ray(const Ray& ray, const Object& o, float time_count = 1.0)
{
    std::chrono::duration<float> duration(0);
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
	std::optional<Hit> h; unsigned long n=0;
    while (duration.count() < time_count)
    {
		h = trace(ray,o);
		duration = std::chrono::system_clock::now() - start;
		++n;
	}

    return std::make_tuple(h,duration.count()/float(n));
}

template<typename Aggregate>
void add_planes(Aggregate& a, unsigned int number) {
	float df = 8.0f/float(number-2);
	for (float f = 2.0; f <= 6.000001f; f+=df) {
		a.push_back(Plane(Eigen::Vector3f(0.0f,0.0f,-1.0f),Eigen::Vector3f(0.0f,0.0f,f)));
		a.push_back(Plane(Eigen::Vector3f(0.0f,0.0f,-1.0f),Eigen::Vector3f(0.0f,0.0f,1.0f-f)));
	}	
}

std::string to_string(const Plane& p) { return std::string("Single plane      "); }
std::string to_string(const std::list<Plane>& o) { return std::string("List of ")+std::to_string(o.size())+" planes"; }
template<int N>
std::string to_string(const PlanePack<N>& o) { return std::string("Pack of ")+std::to_string(N)+" planes"; }

template<typename Object>
void test(const Ray& r, const Object& o)
{
	std::optional<Hit> hit; float time;
	std::tie(hit,time) = time_per_ray(r,o);
	std::cout<<to_string(o)<<"\t\t"<<std::scientific<<std::setprecision(3)<<1.0f/time<<" rays per second\t";
	if (hit) std::cout<<"   Hit at "<<hit->distance()<<std::endl;
	else std::cout<<"   Didn't hit"<<std::endl;
}

int main(int argc, char** argv) {
	Ray r(Eigen::Vector3f(0.0f,0.0f,0.0f),Eigen::Vector3f(0.0f,0.0f,1.0f));	
	{
		Plane o(Eigen::Vector3f(0.0f,0.0f,-1.0f),Eigen::Vector3f(0.0f,0.0f,2.0f));
		test(r,o);
	}
	
	{
		List<Plane> planes;
		add_planes(planes,2);
		test(r,planes);
	}
	
	{
		PlanePack<2> planes;
		add_planes(planes,2);
		test(r,planes);
	}

	{
		List<Plane> planes;
		add_planes(planes,4);
		test(r,planes);
	}
	
	{
		PlanePack<4> planes;
		add_planes(planes,4);
		test(r,planes);
	}
	
	
	{
		List<Plane> planes;
		add_planes(planes,16);
		test(r,planes);
	}
		
	{
		PlanePack<16> planes;
		add_planes(planes,16);
		test(r,planes);
	}
	
	{
		List<Plane> planes;
		add_planes(planes,256);
		test(r,planes);
	}
	
	{
		PlanePack<256> planes;
		add_planes(planes,256);
		test(r,planes);
	}
}