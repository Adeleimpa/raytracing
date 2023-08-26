//
// Created by Adèle Imparato on 19/12/2022.
//

#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include "Vec3.h"

#include <GL/glut.h>
#include <cfloat>

// unused structure since visit kd tree does not work
/*struct RayBoxIntersection {
    float first_intersection;
    float second_intersection;
    bool intersectionExists;
};*/


struct BoundingBox {
    Vec3 min_corner;
    Vec3 max_corner;
    Vec3 dimensions;

    Vec3 TLF_corner; // Top Left Front
    Vec3 TLB_corner; // Top Left Back
    Vec3 TRF_corner; // Top Right Front
    Vec3 DLB_corner; // Down Left Front
    Vec3 DRF_corner; // Down Right Front
    Vec3 DRB_corner; // Down Right Back

    BoundingBox() : min_corner(Vec3(0.0,0.0,0.0)) , max_corner(Vec3(0.0,0.0,0.0)), dimensions(Vec3(0.0,0.0,0.0)) {}

    void setDimensions(){
        dimensions = Vec3(max_corner[0] - min_corner[0], max_corner[1] - min_corner[1], max_corner[2] - min_corner[2]);
    }

    void constructCorners(){
        TLF_corner = Vec3(min_corner[0], max_corner[1], min_corner[2]);
        TLB_corner = Vec3(min_corner[0], max_corner[1], max_corner[2]);
        TRF_corner = Vec3(max_corner[0], max_corner[1], min_corner[2]);
        DLB_corner = Vec3(min_corner[0], min_corner[1], max_corner[2]);
        DRF_corner = Vec3(max_corner[0], min_corner[1], min_corner[2]);
        DRB_corner = Vec3(max_corner[0], min_corner[1], max_corner[2]);
    }

    void drawBoundingBox(){
        std::vector<Vec3> corners{max_corner, TRF_corner, TLF_corner, TLB_corner, DLB_corner, min_corner, DRF_corner, DRB_corner};

        glColor3f(3.0F, 3.0F, 3.0F); // white
        glBegin(GL_LINE_STRIP);
        for(unsigned int i = 0 ; i < corners.size(); i++) {
            glVertex3f( corners[i][0] , corners[i][1] , corners[i][2] );
        }
        glVertex3f(max_corner[0] , max_corner[1] , max_corner[2]);
        glVertex3f(TLB_corner[0] , TLB_corner[1] , TLB_corner[2]);
        glVertex3f(DLB_corner[0] , DLB_corner[1] , DLB_corner[2]);
        glVertex3f(DRB_corner[0] , DRB_corner[1] , DRB_corner[2]);
        glVertex3f(DRF_corner[0] , DRF_corner[1] , DRF_corner[2]);
        glVertex3f(TRF_corner[0] , TRF_corner[1] , TRF_corner[2]);
        glVertex3f(TLF_corner[0] , TLF_corner[1] , TLF_corner[2]);
        glVertex3f(min_corner[0] , min_corner[1] , min_corner[2]);

        glEnd();
    }

    // returns true if the ray intersects with the bounding box
    bool intersects(const Ray &ray) const {
        bool res = true;

        float min = -FLT_MAX;
        float max = FLT_MAX;

        for (int i = 0; i < 3; i++) {
            float t0 = (min_corner[i] - ray.origin()[i]) / ray.direction()[i];
            float t1 = (max_corner[i] - ray.origin()[i]) / ray.direction()[i];

            if (t0 > t1){
                // swap values
                float temp = t0;
                t0 = t1;
                t1 = temp;
            }

            min = std::max(min, t0);
            max = std::min(max, t1);

            if (max < min){
                res = false;
            }
        }

        return res;
    }

    void toString(){
        std::cout << "bounding box to string" << std::endl;

        std::cout << "min_corner: " << min_corner[0] << "," << min_corner[1] << "," << min_corner[2] << std::endl;
        std::cout << "max_corner: " << max_corner[0] << "," << max_corner[1] << "," << max_corner[2] << std::endl;
    }
};


#endif // BOUNDINGBOX_H
