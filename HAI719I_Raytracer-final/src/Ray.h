#ifndef RAY_H
#define RAY_H
#include "Line.h"
#include <stack>

class Ray : public Line {
public:
    Ray() : o(), d(), Line() {}
    // o : origin
    // d : direction

    Vec3 o;
    Vec3 d;

    Ray( Vec3 const & o , Vec3 const & d ) : o(o), d(d), Line(o,d) {}

    void toString(){
        std::cout << "RAY --> origin: (" << o[0] << "," << o[1] << "," << o[2] << "), direction: (" << d[0] << "," << d[1] << "," << d[2] << ")" << std::endl;
    }

    static Ray getReflectedRay(Vec3 ray_dir, Vec3 normal, Vec3 intersection, float dot){
        Vec3 ray_refl_d = ray_dir - (2 * normal * dot);
        Ray ray_refl = Ray(intersection + ray_refl_d * 0.01f, ray_refl_d);
        return ray_refl;
    }

    static Ray getInvisibletRay(Ray ray, Vec3 normal, Vec3 intersection, float dot, float im){

        float prod = (im * im) * (dot * dot);

        // create new ray
        Vec3 ray_inv_d;
        if(prod >= 0.0){
            ray_inv_d = (im * ray.direction()) + (im * (dot + sqrt(prod)) * normal);
            ray_inv_d.normalize();
        }else{
            ray_inv_d = ray.direction();
        }

        Ray ray_inv = Ray(intersection + ray_inv_d * 0.01f, ray_inv_d);
        return ray_inv;
    }

    static Ray getRefractedRay(Vec3 ray_dir, Vec3 normal, Vec3 intersection, float n1, float n2){

        // n1: index of refraction medium 1
        // n2 : index of refraction medium 2

        Vec3 incident_dir = ray_dir.negative(); // I
        incident_dir.normalize();

        float dot = Vec3::dot(incident_dir, normal); // (N . I)

        float nr = n1/n2; // refraction coefficient

        Vec3 ray_refr_dir;

        float dot2 = dot*dot;
        float nr2 = nr*nr;

        ray_refr_dir = (nr*dot - sqrt(1 - nr2*(1-dot2) )) * normal -  (nr * incident_dir);
        ray_refr_dir.normalize();

        //Ray ray_refr = Ray(intersection + ray_refr_dir * 0.01, intersection + ray_refr_dir); // to avoid issue of self intersection
        Ray ray_refr = Ray(intersection,  ray_refr_dir);

        return ray_refr;
    }

};
#endif
