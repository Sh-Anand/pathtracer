#include "ray.h"

HOST_DEVICE Ray::Ray(const Vector3D o, const Vector3D d, int depth) {
    // printf("Ray : %lf %lf %lf %lf %lf %lf \n", o.x, o.y, o.z, d.x, d.y, d.z);
    this->o = o;
    this->d = d;
    this->depth = depth;
    this->min_t = 0;
    this->max_t = INFINITY;
    this->inv_d = 1/d;
}
HOST_DEVICE Ray::Ray(const Vector3D o, const Vector3D d, float max_t, int depth) {
    this->inv_d = 1/d;
    this->max_t = max_t;
    Ray(o, d, depth);
}