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
std::tuple<std::optional<Hit>, float> time_per_ray(const Ray& ray, const Object& o, float time_count = 2.0f)
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

void add_all(std::list<Plane>& a, int number) {
	float df = 8.0f/float(std::max(number-2,1));
	for (float f = 2.0; f <= 6.000001f; f+=df) {
		a.push_back(Plane(Eigen::Vector3f(0.0f,0.0f,-1.0f),Eigen::Vector3f(0.0f,0.0f,f)));
		if ((number % 2) == 0)
			a.push_back(Plane(Eigen::Vector3f(0.0f,0.0f,-1.0f),Eigen::Vector3f(0.0f,0.0f,1.0f-f)));
	}	
}

std::string to_string(const Plane& p)  { return std::string("planes "); }
std::string to_string(const Sphere& p) { return std::string("spheres"); }

void add_all(std::list<Sphere>& a, int number) {
	float df = 8.0f/float(std::max(number-2,1));
	for (float f = 2.0; f <= 6.000001f; f+=df) {
		a.push_back(Sphere(Eigen::Vector3f(0.0f,0.0f,2.0f*f),f));
		if ((number % 2) == 0)
			a.push_back(Sphere(Eigen::Vector3f(0.0f,0.0f,0.5f-2.0f*f),f));
	}	
}


template<typename O, int N>
void compare(float& ref_time_list, float& ref_time_pack) {
	Ray r(Eigen::Vector3f(0.0f,0.0f,0.0f),Eigen::Vector3f(0.0f,0.0f,1.0f));
	
	std::list<O> objects;
	add_all(objects, N);

	std::cout<<std::setw(6)<<N<<" "<<to_string(O())<<"\t"<<std::flush;
	
	std::optional<Hit> hit_list; float time_list;
	tracer::List<O>    list(objects);
	std::tie(hit_list, time_list) = time_per_ray(r, list);
	if (ref_time_list <= 0.0f) ref_time_list = time_list;
	std::cout<<std::scientific<<std::setprecision(3)<<1.0f/time_list<<" rps   ("<<std::fixed<<std::setw(6)<<std::setprecision(2)<<100.0f*(ref_time_list/time_list)<<"%)\t"<<std::flush;

	std::optional<Hit> hit_pack; float time_pack;
	tracer::Pack<O,N> pack(objects);
	std::tie(hit_pack, time_pack) = time_per_ray(r, pack);
	if (ref_time_pack <= 0.0f) ref_time_pack = time_pack;
	std::cout<<std::scientific<<std::setprecision(3)<<1.0f/time_pack<<" rps   ("<<std::fixed<<std::setw(6)<<std::setprecision(2)<<100.0f*(ref_time_pack/time_pack)<<"%)\t"<<std::flush;

	std::cout<<std::fixed<<std::setw(6)<<std::setprecision(2)<<100.0f*(time_list/time_pack)<<"%"<<std::endl;

	if ((!hit_list) || (hit_list->distance() != 2.0f)) std::cout<<"List error : "<<hit_list->distance()<<std::endl;
	if ((!hit_pack) || (hit_pack->distance() != 2.0f)) std::cout<<"Pack error : "<<hit_pack->distance()<<std::endl;
}

template<typename O>
void compare_sizes() {
	float ref_time_list = 0.0f, ref_time_pack = 0.0f;
	compare<O,1>(ref_time_list, ref_time_pack);
	compare<O,2>(ref_time_list, ref_time_pack);
	compare<O,4>(ref_time_list, ref_time_pack);
	compare<O,8>(ref_time_list, ref_time_pack);
	compare<O,16>(ref_time_list, ref_time_pack);
	compare<O,32>(ref_time_list, ref_time_pack);
	compare<O,64>(ref_time_list, ref_time_pack);
	compare<O,128>(ref_time_list, ref_time_pack);
	compare<O,256>(ref_time_list, ref_time_pack);
	compare<O,512>(ref_time_list, ref_time_pack);
	compare<O,1024>(ref_time_list, ref_time_pack);
}

int main(int argc, char** argv) {

	std::cout<<"       PLANES \t         List\t\t         Pack    \t\t\t Pack improvement"<<std::endl;
	compare_sizes<tracer::Plane>();
	std::cout<<"       SPHERES\t         List\t\t         Pack    \t\t\t Pack improvement"<<std::endl;
	compare_sizes<tracer::Sphere>();
}
