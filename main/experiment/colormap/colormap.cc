#include <array>
#define MATERIAL std::array<float,3>

#include <iostream>
#include <tracer/tracer.h>
#include <cimg-all.h>

int main(int argc, char** argv) {
	int w = 512;
	int h = 512;

	cimg_library::CImg<float> output(w,h,1,3);

	tracer::Pinhole camera(Eigen::Vector3f( 0, 0, -3), Eigen::Vector3f( 0, 0, 2), Eigen::Vector3f( 0, 1, 0));

	tracer::Scene scene;
    scene.add(tracer::Plane(Eigen::Vector3f(0,0,-10),Eigen::Vector3f(1,0,0)).set_material(std::array<float,3>{0.5f,0.5f,0.5f}));
    scene.add(tracer::Sphere(Eigen::Vector3f( 0.5, -0.65,-0.2), 0.35).set_material(std::array<float,3>{0.9f,0.1f,0.1f}));
    scene.add(tracer::Triangle(Eigen::Vector3f(0,-1,-0.8),Eigen::Vector3f(0,0,-0.5), Eigen::Vector3f(0.5,-1,-0.2)).set_material(std::array<float,3>{0.1f,0.9f,0.1f}));
    scene.add(tracer::AxisAlignedBox(Eigen::Vector3f(0.75,1, 0),Eigen::Vector3f(0.25,0.5,0.5)).set_material(std::array<float,3>{0.1f,0.1f,0.9f}));

	float du = 2.0/float(w);
	float dv = 2.0/float(h);
	float u, v; int i, j;
	for (j=0,v=0.5f*dv - 1.0f; j<h; ++j, v+=dv) for (i=0,u=0.5*du - 1.0f; i<w; ++i, u+=du) {
		std::optional<tracer::Hit> hit = scene.trace(camera.ray(u,v));
		if (hit) { 
            if (hit->material()) for (int c=0;c<3;++c) output(i,j,0,c) = hit->material()->at(c);
            else output(i,j,0,0) = output(i,j,0,1) =  output(i,j,0,2) = 0.2f;
        } 
        else     output(i,j,0,0) = output(i,j,0,1) =  output(i,j,0,2) = 0.001f;
	}

	output.save("output.hdr");
}
