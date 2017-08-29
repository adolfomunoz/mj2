#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include <tuple>
#include <limits>
#include <list>
#include <iomanip>

#include <tracer/tracer.h>

using namespace tracer; 

template<typename Object>
std::tuple<std::optional<Hit>, float> time_per_ray(const Ray& ray, const Object& o, float time_count = 1.0)
{
    std::chrono::duration<float> duration(0);
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
    std::optional<Hit> h; unsigned long n=0;
    while (duration.count() < time_count)
    {
	h = o.trace(ray);
	duration = std::chrono::system_clock::now() - start;
	++n;
    }

    return std::make_tuple(h,duration.count()/float(n));
}

template<typename Aggregate>
void add_planes(Aggregate& a, int number) {
	float df = 8.0f/float(std::max(number-2,1));
	for (float f = 2.0; f <= 6.000001f; f+=df) {
		a.push_back(Plane(Eigen::Vector3f(0.0f,0.0f,-1.0f),Eigen::Vector3f(0.0f,0.0f,f)));
		if ((number % 2) == 0)
			a.push_back(Plane(Eigen::Vector3f(0.0f,0.0f,-1.0f),Eigen::Vector3f(0.0f,0.0f,1.0f-f)));
	}	
}

template<int N>
void compare() {
	Ray r(Eigen::Vector3f(0.0f,0.0f,0.0f),Eigen::Vector3f(0.0f,0.0f,1.0f));
	
	std::list<tracer::Plane> planes;
	add_planes(planes, N);

	std::cout<<std::setw(6)<<N<<"\t"<<std::flush;
	
	std::optional<Hit> hit_list; float time_list;
	tracer::List<Plane> planes_list(planes);
	std::tie(hit_list, time_list) = time_per_ray(r, planes_list);
	std::cout<<std::scientific<<std::setprecision(3)<<1.0f/time_list<<" rps\t"<<std::flush;

	std::optional<Hit> hit_pack; float time_pack;
	tracer::Pack<Plane,N> planes_pack(planes);
	std::tie(hit_pack, time_pack) = time_per_ray(r, planes_pack);
	std::cout<<std::scientific<<std::setprecision(3)<<1.0f/time_pack<<" rps"<<std::endl;

	if ((!hit_list) || (hit_list->distance() != 2.0f)) std::cout<<"List error : "<<hit_list->distance()<<std::endl;
	if ((!hit_pack) || (hit_pack->distance() != 2.0f)) std::cout<<"Pack error : "<<hit_pack->distance()<<std::endl;
}

int main(int argc, char** argv) {
	std::cout<<"      \t         List\t         Pack"<<std::endl;
	compare<1>();
	compare<2>();
	compare<4>();
	compare<8>();
	compare<16>();
	compare<32>();
	compare<64>();
	compare<128>();
	compare<256>();
	compare<512>();
	compare<1024>();
}
