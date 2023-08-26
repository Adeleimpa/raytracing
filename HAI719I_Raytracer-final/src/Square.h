#ifndef SQUARE_H
#define SQUARE_H
#include "Vec3.h"
#include <vector>
#include "Mesh.h"
#include <cmath>
#include "BoundingBox.h"


struct RayBoxIntersection {
    float first_intersection;
    float second_intersection;
    bool intersectionExists;
};

struct RaySquareIntersection{
    bool intersectionExists;
    float t;
    float u,v;
    Vec3 intersection;
    Vec3 normal;
};


class Square : public Mesh {
public:
    Vec3 m_normal;
    Vec3 m_bottom_left;
    Vec3 m_right_vector;
    Vec3 m_up_vector;

    Square() : Mesh() {}
    Square(Vec3 const & bottomLeft , Vec3 const & rightVector , Vec3 const & upVector , float width=1. , float height=1. ,
           float uMin = 0.f , float uMax = 1.f , float vMin = 0.f , float vMax = 1.f) : Mesh() {
        setQuad(bottomLeft, rightVector, upVector, width, height, uMin, uMax, vMin, vMax);
    }

    void setQuad( Vec3 const & bottomLeft , Vec3 const & rightVector , Vec3 const & upVector , float width=1. , float height=1. ,
                  float uMin = 0.f , float uMax = 1.f , float vMin = 0.f , float vMax = 1.f) {
        m_right_vector = rightVector;
        m_up_vector = upVector;
        m_normal = Vec3::cross(rightVector , upVector);
        m_bottom_left = bottomLeft;

        m_normal.normalize();
        m_right_vector.normalize();
        m_up_vector.normalize();

        m_right_vector = m_right_vector*width;
        m_up_vector = m_up_vector*height;

        vertices.clear();
        vertices.resize(4);
        vertices[0].position = bottomLeft;                                      vertices[0].u = uMin; vertices[0].v = vMin;
        vertices[1].position = bottomLeft + m_right_vector;                     vertices[1].u = uMax; vertices[1].v = vMin;
        vertices[2].position = bottomLeft + m_right_vector + m_up_vector;       vertices[2].u = uMax; vertices[2].v = vMax;
        vertices[3].position = bottomLeft + m_up_vector;                        vertices[3].u = uMin; vertices[3].v = vMax;
        vertices[0].normal = vertices[1].normal = vertices[2].normal = vertices[3].normal = m_normal;
        triangles.clear();
        triangles.resize(2);
        triangles[0][0] = 0;
        triangles[0][1] = 1;
        triangles[0][2] = 2;
        triangles[1][0] = 0;
        triangles[1][1] = 2;
        triangles[1][2] = 3;
    }

    Vec3 bottomLeft() const{
        return this->vertices[0].position;
    }

    Vec3 bottomRight() const{
        return this->vertices[1].position;
    }

    Vec3 upRight() const{
        return this->vertices[2].position;
    }

    Vec3 upLeft() const{
        return this->vertices[3].position;
    }

    Vec3 normal() const{
        return Vec3::cross((bottomRight()-bottomLeft()), (upLeft()-bottomLeft()));
    }

    // compute the intersection between a rayon and a quad
    RaySquareIntersection intersect(const Ray &ray) const {
        RaySquareIntersection intersection;

        Vec3 a = bottomLeft(); // any point in plan
        float D = Vec3::dot(a , normal());

        float t = (D - Vec3::dot(ray.o, normal() )) / Vec3::dot(ray.d, normal());

        if (t > 0.00001){
            Vec3 intersection_point = ray.o + t*ray.d;

            Vec3 new_x_axis = bottomRight() - bottomLeft();
            Vec3 new_y_axis = upLeft() - bottomLeft();

            float u = Vec3::dot(new_x_axis, (intersection_point-bottomLeft())) / new_x_axis.norm();
            float v = Vec3::dot(new_y_axis, (intersection_point-bottomLeft())) / new_y_axis.norm();

            // is in quad ?
            if(u >= 0 and u <= new_x_axis.norm() and v >= 0 and v <= new_y_axis.norm()){
                intersection.intersectionExists = true;
                intersection.t = t;
                intersection.intersection = intersection_point;
                intersection.u = u;
                intersection.v = v;
                intersection.normal = normal();
            }else{
                intersection.intersectionExists = false;
            }
        }else{
            intersection.intersectionExists = false;
        }

        return intersection;
    }

    // unused method since kd tree does not work
    // there is probably an error here
    /*static RayBoxIntersection intersectBox(const Ray &ray, BoundingBox bb) {
        //std::cout << "intersectBox" << std::endl;

        RayBoxIntersection rbi;

        //std::cout << "ray origin:" << ray.o[0] <<","<<ray.o[1]<<","<<ray.o[2] << ", dir:" << ray.d[0]<<","<<ray.d[1]<<","<<ray.d[2] << std::endl;

        std::vector< Square > square_list;
        square_list.resize( 6 );
        Square & s1 = square_list[0];
        Square & s2 = square_list[1];
        Square & s3 = square_list[2];
        Square & s4 = square_list[3];
        Square & s5 = square_list[4];
        Square & s6 = square_list[5];

        s1.setQuad(bb.min_corner, bb.DRF_corner-bb.min_corner, bb.TLF_corner-bb.min_corner, bb.dimensions[0], bb.dimensions[1]); // front quad
        s2.setQuad(bb.min_corner, bb.DLB_corner-bb.min_corner, bb.TLF_corner-bb.min_corner, bb.dimensions[2], bb.dimensions[1]); // left quad
        s3.setQuad(bb.min_corner, bb.DRF_corner-bb.min_corner, bb.DLB_corner-bb.min_corner, bb.dimensions[0], bb.dimensions[2]); // bottom quad
        s4.setQuad(bb.DRF_corner, bb.DRB_corner-bb.DRF_corner, bb.TRF_corner-bb.DRF_corner, bb.dimensions[2], bb.dimensions[1]); // right quad
        s5.setQuad(bb.DLB_corner, bb.DLB_corner-bb.DRB_corner, bb.DLB_corner-bb.TLF_corner, bb.dimensions[0], bb.dimensions[1]); // back quad
        s6.setQuad(bb.TLF_corner, bb.TLF_corner-bb.TRF_corner, bb.TLF_corner-bb.TLB_corner, bb.dimensions[0], bb.dimensions[2]); // top quad

        //bb.toString();

        float i1, i2;

        int counter = 0;
        for(unsigned int i = 0; i < square_list.size(); i++){
            RaySquareIntersection rsi = square_list[i].intersect(ray);

            if(rsi.intersectionExists){ // if intersection exists
                rbi.intersectionExists = true;
                counter++;
                // either first of second intersection
                if(counter == 2){
                    //std::cout << "two intersections ewer found" << std::endl;
                    i2 = rsi.t;
                    rbi.first_intersection = std::min(i1, i2);
                    rbi.second_intersection = std::max(i1, i2);
                }else if(counter == 1){
                    //std::cout << "one intersection was found" << std::endl;
                    i1 = rsi.t;
                }else{
                    //std::cout << "more than two intersections were found" << std::endl;
                }
                // what happens when 3 intersections (touch corner) ?
            }
        }
        return rbi;
    }*/
};
#endif // SQUARE_H
