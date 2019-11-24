#include <iostream>
#include <tracer/tracer.h>
#include <scenes/cornell-box.h>
#include <cimg-all.h>

int main(int argc, char** argv) {
	tracer::Scene scene = scenes::cornell_box();

	int w = 512;
	int h = 512;

	cimg_library::CImg<float> output(w,h,1,3);

	tracer::Pinhole camera(Eigen::Vector3f( 0, 0, -3), Eigen::Vector3f( 0, 0, 2), Eigen::Vector3f( 0, 1, 0));

	float du = 2.0/float(w);
	float dv = 2.0/float(h);
	float u, v; int i, j;
	for (j=0,v=0.5f*dv - 1.0f; j<h; ++j, v+=dv) for (i=0,u=0.5*du - 1.0f; i<w; ++i, u+=du) {
		std::optional<tracer::Hit> hit = scene.trace(camera.ray(u,v));
		if (hit) { for (int c=0;c<3;++c) output(i,j,0,c) = 0.5f*hit->normal()[c] + 0.5f; }
		else     output(i,j,0,0) = output(i,j,0,1) =  output(i,j,0,2) = 0.0f;
	}

	output.save("output.hdr");
}
