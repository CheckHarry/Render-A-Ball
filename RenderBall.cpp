#include <array>
#include "bitmap_image.hpp"
#include <math.h>

#define x_pixel_range 1500
#define y_pixel_range 1500
#define coor(x , y) y_pixel_range*x + y

using namespace std;

array<double, 3> viewer = { 4 , 0 , 0 };
array<double, 3> axis_x = { 1 , 0 , 0 };
array<double, 3> axis_y = { 0 , 1 , 0 };
array<double, 3> axis_z = { 0 , 0 , 1 };

array<double, 3> source = { 2 , 2 , 2 };
array<double, 3> lightcolor = { 245,200,240 };
array<double, 3> source2 = { 2 , 2 , 2 };
static  double pi = 3.14159;

array<double, 3> operator*(double scalar, array<double, 3> vec) {
	array<double, 3> result = { scalar * vec[0] , scalar * vec[1] , scalar * vec[2] };
	return result;
}

array<double, 3> operator+(array<double, 3> vec1, array<double, 3>  vec2) {
	array<double, 3> result = { vec1[0] + vec2[0],vec1[1] + vec2[1] ,vec1[2] + vec2[2] };
	return result;
}

array<double, 3> operator-(array<double, 3> vec1, array<double, 3>  vec2) {
	array<double, 3> result = { vec1[0] - vec2[0],vec1[1] - vec2[1] ,vec1[2] - vec2[2] };
	return result;
}

double dot_product(array<double, 3> x, array<double, 3> y) {
	double sum = 0;
	for (int i = 0; i < 3; i++) {
		sum += x[i] * y[i];
	}
	return sum;
}

bool is_intersect(array<double, 3> viewerpoint, array<double, 3> ray) {
	double a = dot_product(ray , ray);
	double b = 2 * (dot_product(viewerpoint , ray));
	double c = dot_product(viewerpoint , viewerpoint) - 1;
	return b * b - 4 * a * c >= 0 ;
}

array<double, 3> get_interectpoint(array<double, 3> viewerpoint, array<double, 3> ray) {
	double a = dot_product(ray, ray);
	double b = 2 * (dot_product(viewerpoint, ray));
	double c = dot_product(viewerpoint, viewerpoint) - 1;
	double lambda1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
	double lambda2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
	array<double, 3> v1 = lambda1 * ray;
	array<double, 3> v2 = lambda2 * ray;
	if (dot_product(v1 , v1) > dot_product(v2 , v2)) {
		return viewerpoint + v2;
	}
	else {
		return viewerpoint+ v1;
	}
}

array<double, 3> get_normal(array<double, 3> point) {
	double norm = sqrt(dot_product(point , point));
	array<double, 3> normalvector = { point[0] / norm , point[1] / norm ,point[2] / norm };
	return normalvector;
}

array<double, 3> get_reflect(array<double, 3> lightsource , array<double , 3> contact) {
	array<double, 3> ray = contact - lightsource;
	array<double, 3> normal = get_normal(contact);
	double lambda = -dot_product(ray, normal) / (dot_product(normal, normal));
	array<double, 3> reflectray = 2 * (ray + lambda * normal) - ray;
	return reflectray;
}

array<double, 3> multiply_rotationmatrix(array<double, 3> point , double theta) {
	double C = cos(theta);
	double S = sin(theta);
	
	array<double, 3> result_vec = { C * point[0] + S * point[2] , point[1] , -S * point[0] + C * point[2] };
	return result_vec;
}
// theta = pi * 1.5

void generateBitmapImage(unsigned char* image, int height, int width, char* imageFileName) {

}



int main() {
	array<double, 3> ray_center = multiply_rotationmatrix(axis_z, pi * 1.5);
	array<double, 3> ray_center_y = multiply_rotationmatrix(axis_x, pi * 1.5);
	array<double, 3> ray_center_x = multiply_rotationmatrix(axis_y, pi * 1.5);

	auto pixel = new array<double , x_pixel_range*y_pixel_range>;
	array<double, 3> intersectpoint;
	array<double, 3> viewer_ray;
	array<double, 3> reflect_ray;
	array<double, 3> reflect_ray2;
	array<double, 3> ray_view;
	for (int x = 0; x < x_pixel_range; x++) {
		for (int y = 0; y < y_pixel_range; y++) {
			double step_x = double(x - double(x_pixel_range / 2)) / double(x_pixel_range *0.85);
			double step_y = double(double(y_pixel_range / 2) - y)/ double(y_pixel_range *0.85);
			ray_view = ray_center + step_x*ray_center_x + step_y*ray_center_y ;
			bool k = is_intersect(viewer, ray_view);
			if (k) {
				intersectpoint = get_interectpoint(viewer, ray_view);
				viewer_ray = intersectpoint - viewer;
				reflect_ray = get_reflect(source, intersectpoint);
				reflect_ray2 = get_reflect(source2, intersectpoint);
				(*pixel)[coor(x,y)] = max((dot_product(viewer_ray, -1 * reflect_ray) / sqrt(dot_product(viewer_ray, viewer_ray) * dot_product(reflect_ray, reflect_ray))) + 1, double(0))/2;
											  
			}
			else {
				(*pixel)[coor(x, y)] = double(0);
			}
		}
	}
	
	bitmap_image image(x_pixel_range, y_pixel_range);
	image_drawer draw(image);
	for (int i = 0; i < x_pixel_range; i++) {
		for (int j = 0; j < x_pixel_range; j++) {
			image.set_pixel(		i	,		j		,	 int((*pixel)[coor(i,j)] * lightcolor[0]),int((*pixel)[coor(i,j)] * lightcolor[2]),int((*pixel)[coor(i,j)] * lightcolor[2])	);
			
		}
	}
	image.save_image("output.bmp");
	delete pixel;

}