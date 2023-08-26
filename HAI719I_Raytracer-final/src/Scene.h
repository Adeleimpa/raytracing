#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <string>
#include <stack>
#include "Mesh.h"
#include "Sphere.h"
#include "Square.h"
#include "BoundingBox.h"


#include <GL/glut.h>
#include "src/matrixUtilities.h"


enum LightType {
    LightType_Spherical,
    LightType_Quad
};


struct Light {
    Vec3 material;
    bool isInCamSpace;
    LightType type;

    Vec3 pos;
    float radius;

    Mesh quad; // is a square

    float powerCorrection;

    Light() : powerCorrection(1.0) {}

};

struct TreeNode {
    std::vector<Triangle> node_data_triangle;
    TreeNode *leftChild; // pointer to the left subtree
    TreeNode *rightChild; // pointer to the right subtree
    bool isRoot; // false by default
    bool isLeaf; // false by default
    int depth; // depth of that node
    struct BoundingBox bounding_box;
    TreeNode() : isRoot(false) , isLeaf(false) {}
};

// returns boolean
struct RaySceneIntersection{
    bool intersectionExists;
    unsigned int typeOfIntersectedObject;
    unsigned int objectIndex;
    float t; // distance in ray
    Vec3 intersection;
    Vec3 normal;
    RayTriangleIntersection rayMeshIntersection;
    RaySphereIntersection raySphereIntersection;
    RaySquareIntersection raySquareIntersection;
    RaySceneIntersection() : intersectionExists(false) , t(FLT_MAX) {}
};



class Scene {
    std::vector< Mesh > meshes; // typeOfIntersectedObject = 0
    std::vector< Sphere > spheres;
    std::vector< Square > squares;
    std::vector< Light > lights;
    struct TreeNode* scene_tree;

public:

    Scene() {
    }

    void draw() {
        // iterer sur l'ensemble des objets, et faire leur rendu :
        for( unsigned int It = 0 ; It < meshes.size() ; ++It ) {
            Mesh const & mesh = meshes[It];
            mesh.draw();
            meshes[It].bounding_box.drawBoundingBox();
            meshes[It].bounding_box.constructCorners();
        }
        for( unsigned int It = 0 ; It < spheres.size() ; ++It ) {
            Sphere const & sphere = spheres[It];
            sphere.draw();
        }
        for( unsigned int It = 0 ; It < squares.size() ; ++It ) {
            Square const & square = squares[It];
            square.draw();
        }
    }

    //Compute the intersections with the objects of the scene and keep the closest one
    RaySceneIntersection computeIntersection(Ray const & ray, float znear) {
        RaySceneIntersection result; // default values are intersectionExists=false and t=FLT_MAX

        // FOR SPHERES
        int nr_spheres = spheres.size();

        for(int i = 0; i < nr_spheres; i++){
            RaySphereIntersection sphere_intersection = spheres[i].intersect(ray);

            float intersection_dist = sphere_intersection.t;
            if(sphere_intersection.intersectionExists and intersection_dist < result.t){
                result.intersectionExists = sphere_intersection.intersectionExists;
                result.typeOfIntersectedObject = 1;
                result.objectIndex = i;
                result.t = intersection_dist;
                result.raySphereIntersection = sphere_intersection;
                result.normal = sphere_intersection.normal;
                result.intersection = sphere_intersection.intersection;
            }
        }

        // FOR SQUARES
        int nr_squares = squares.size();

        for(int i = 0; i < nr_squares; i++){
            RaySquareIntersection square_intersection = squares[i].intersect(ray);

            float intersection_dist = square_intersection.t;
            if (square_intersection.intersectionExists and intersection_dist < result.t and
                intersection_dist > znear) {
                result.intersectionExists = square_intersection.intersectionExists;
                result.typeOfIntersectedObject = 2;
                result.objectIndex = i;
                result.t = intersection_dist;
                result.raySquareIntersection = square_intersection;
                result.normal = square_intersection.normal;
                result.intersection = square_intersection.intersection;
            }
        }

        // FOR MESHES
        int nr_meshes = meshes.size();

        for(int i = 0; i < nr_meshes; i++) {

            if (meshes[i].bounding_box.intersects(ray)) {
                RayTriangleIntersection mesh_intersection = meshes[i].intersect(ray);

                float intersection_dist = mesh_intersection.t;
                if (mesh_intersection.intersectionExists and intersection_dist < result.t and
                    intersection_dist > znear) {
                    result.intersectionExists = mesh_intersection.intersectionExists;
                    result.typeOfIntersectedObject = 0;
                    result.objectIndex = i;
                    result.t = intersection_dist;
                    result.rayMeshIntersection = mesh_intersection;
                    result.normal = mesh_intersection.normal;
                    result.intersection = mesh_intersection.intersection;
                }
            }
        }

        return result;
    }

    // TODO debug method to visit the kd tree
    // returns the intersection between the ray and the closest triangle belonging to the smallest leaf node
    /*RayTriangleIntersection recursiveVisitKdTree(Ray const & ray, TreeNode* current_node, float t_start, float t_end){

        if((*current_node).isLeaf){ // stop criteria
            //std::cout << "isLeaf==true, stop criteria." << std::endl;

            // find closest intersection among triangles
            RayTriangleIntersection rti;
            RayTriangleIntersection closestIntersection;
            closestIntersection.t = FLT_MAX;

            for(unsigned int i = 0; i < (*current_node).node_data_triangle.size() ; i++){
                rti = (*current_node).node_data_triangle[i].getIntersection(ray);

                if (rti.intersectionExists and rti.t < closestIntersection.t) {
                    closestIntersection = rti;
                }
            }
            return closestIntersection;
        }

        //(*(*current_node).leftChild).bounding_box.toString();
        RayBoxIntersection rbi = Square::intersectBox(ray,(*(*current_node).leftChild).bounding_box);
        float t = rbi.second_intersection;

        if (t <= t_end){ // CASE ONE traverse second box only
            //std::cout << "case one" << std::endl;
            return recursiveVisitKdTree(ray, (*current_node).rightChild, t_start, t_end);
        }else if (t >= t_end){ // CASE TWO traverse first box only
            //std::cout << "case two" << std::endl;
            return recursiveVisitKdTree(ray, (*current_node).leftChild, t_start, t_end);
        }else { // CASE THREE traverse both boxes
            //std::cout << "case three" << std::endl;
            RayTriangleIntersection rti_hit = recursiveVisitKdTree(ray, (*current_node).leftChild, t_start, t);
            if(rti_hit.intersectionExists and rti_hit.t <= t){
                return rti_hit; // early termination
            }
            return recursiveVisitKdTree(ray, (*current_node).rightChild, t, t_end);
        }
    }*/

    Vec3 phongLight(Light light, Ray ray, RaySceneIntersection rsi, Material material){
        //PHONG = ambient_material + diffuse_material + specular_material;

        Vec3 color;

        Vec3 ambient_material = material.ambient_material;
        Vec3 diffuse_material = material.diffuse_material;
        Vec3 specular_material = material.specular_material;

        // ambient light
        double I_saR = light.material[0];
        double I_saV = light.material[1];
        double I_saB = light.material[2];
        double K_aR = ambient_material[0];
        double K_aV = ambient_material[1];
        double K_aB = ambient_material[2];

        // make sure you normalize
        rsi.normal.normalize();
        ray.d.normalize(); // V

        Vec3 light_vector = light.pos - rsi.intersection;
        light_vector.normalize(); // L
        double cos_theta = Vec3::dot(light_vector, rsi.normal); // no need to divide by their length as they
        // are normalized (length = 1)

        // make sure you take positive values
        cos_theta = std::max(0.,cos_theta);

        Vec3 light_reflect_vector = light_vector - (2 * Vec3::dot( rsi.normal, light_vector) * rsi.normal);
        light_reflect_vector.normalize(); // R
        double cos_alpha = Vec3::dot(ray.d, light_reflect_vector);

        // make sure you take positive values
        cos_alpha = std::max(0.,cos_alpha);

        // diffuse light
        double I_sdR = light.material[0];
        double I_sdV = light.material[1];
        double I_sdB = light.material[2];
        double K_dR = diffuse_material[0];
        double K_dV = diffuse_material[1];
        double K_dB = diffuse_material[2];

        // specular light
        double I_ssR = light.material[0];
        double I_ssV = light.material[1];
        double I_ssB = light.material[2];
        double K_sR = specular_material[0];
        double K_sV = specular_material[1];
        double K_sB = specular_material[2];

        double n = material.shininess;

        double I_R = I_saR * K_aR + I_sdR * K_dR * cos_theta + I_ssR * K_sR * pow(cos_alpha,n);
        double I_V = I_saV * K_aV + I_sdV * K_dV * cos_theta + I_ssV * K_sV * pow(cos_alpha,n);
        double I_B = I_saB * K_aB + I_sdB * K_dB * cos_theta + I_ssB * K_sB * pow(cos_alpha,n);

        color[0] = I_R;
        color[1] = I_V;
        color[2] = I_B;

        return color;
    }

    Vec3 rayTraceRecursive(Ray const &ray , int NRemainingBounces, float znear) {
        RaySceneIntersection raySceneIntersection;

        raySceneIntersection = computeIntersection(ray, znear);

        Vec3 color(0.,0.,0.);
        if(raySceneIntersection.intersectionExists){

            // dot product between ray direction and the normal of the intersection
            float dp_dir_n = Vec3::dot(ray.direction(), raySceneIntersection.normal);

            if (raySceneIntersection.typeOfIntersectedObject == 1) { // sphere
                float im = spheres[raySceneIntersection.objectIndex].material.index_medium; // index of refraction

                // reflection for mirror
                if (spheres[raySceneIntersection.objectIndex].material.type == Material_Mirror and
                    NRemainingBounces > 0) {
                    Ray ray_refl = Ray::getReflectedRay(ray.direction(), raySceneIntersection.normal, raySceneIntersection.intersection, dp_dir_n);
                    color = rayTraceRecursive(ray_refl, NRemainingBounces - 1, 0.01);

                // invisibility
                } else if (spheres[raySceneIntersection.objectIndex].material.type == Material_Invisible and
                           NRemainingBounces > 0) {
                    Ray ray_rec = Ray::getInvisibletRay(ray,raySceneIntersection.normal,raySceneIntersection.intersection, dp_dir_n, im);
                    color = rayTraceRecursive(ray_rec, NRemainingBounces - 1, 0.0);

                // refraction for glass
                // TODO remove noise
                }else if(spheres[raySceneIntersection.objectIndex].material.type == Material_Glass and
                         NRemainingBounces > 0){
                    float n1 = 1.00029; // index of refraction air

                    Ray ray_refr;
                    ray_refr = Ray::getRefractedRay(ray.direction(), raySceneIntersection.normal, raySceneIntersection.intersection, n1, im);
                    color = rayTraceRecursive(ray_refr, NRemainingBounces - 1, 0.01);

                }else{
                    color = phongLight(lights[0], ray, raySceneIntersection, spheres[raySceneIntersection.objectIndex].material);
                    color = softShadow(color, raySceneIntersection, lights[0]);
                }

            }else if(raySceneIntersection.typeOfIntersectedObject == 2){ // square
                color = phongLight(lights[0], ray, raySceneIntersection, squares[raySceneIntersection.objectIndex].material);
                color = softShadow(color, raySceneIntersection, lights[0]);

            }else if(raySceneIntersection.typeOfIntersectedObject == 0) { // mesh
                // color = meshes[raySceneIntersection.objectIndex].material.diffuse_material;
                color = phongLight(lights[0], ray, raySceneIntersection, meshes[raySceneIntersection.objectIndex].material);
                color = softShadow(color, raySceneIntersection, lights[0]);
            }
        }
        else{
            color = {0.,0.,0.}; // background color
        }

        return color;
    }

    Vec3 softShadow(Vec3 initial_color, RaySceneIntersection rsi, Light light) {
        // generate a set of rays around initial ray to get soft shadow
        // the area of light is a square on the x and z axis
        // rays are traced randomly in that area
        // the more rays are intersected, the darker the area (using a percentage of darkness)

        int nr_rays = 20; // amount of rays to generate around initial ray
        int count_intersections = 0;
        int count_invisible_intersections = 0;

        float offset = 0.0001;
        float ignore = 0.9;

        double arealight_dim = 1.5;
        double min_x = light.pos[0]-arealight_dim;
        double max_x = light.pos[0]+arealight_dim;
        double min_z = light.pos[2]-arealight_dim;
        double max_z = light.pos[2]+arealight_dim;

        for(int i = 0; i < nr_rays; i++){ // loop through the amount of needed rays

            //generate random double for x and z
            double rand_x = min_x + ((double)rand() / RAND_MAX) * (max_x - min_x);
            double rand_z = min_z + ((double)rand() / RAND_MAX) * (max_z - min_z);

            // create new ray of light starting from the intersection to reach the source of light
            Vec3 new_ray_d(light.pos[0] + rand_x, light.pos[1], light.pos[2] + rand_z);
            Ray new_ray = Ray(rsi.intersection, new_ray_d - rsi.intersection);

            // Check if this new ray intersects with a sphere
            for(unsigned int i = 0; i < spheres.size(); i++){
                RaySphereIntersection sphere_intersection = spheres[i].intersect(new_ray);
                if(sphere_intersection.intersectionExists and sphere_intersection.t < ignore and sphere_intersection.t > offset){
                    if(spheres[i].material.type == Material_Invisible){
                        count_invisible_intersections++;
                    }else{
                        count_intersections++;
                    }
                    break;
                }
                else{ // if not with a sphere, check for a square

                    for(unsigned int i = 0; i < squares.size(); i++){
                        RaySquareIntersection square_intersection = squares[i].intersect(new_ray);
                        if(square_intersection.intersectionExists and square_intersection.t < ignore  and square_intersection.t > offset){
                            count_intersections++;
                            break;
                        }
                        else { // if not with a square, check for a mesh

                            for (unsigned int i = 0; i < meshes.size(); i++) {
                                RayTriangleIntersection triangle_intersection = meshes[i].intersect(new_ray);
                                if (triangle_intersection.intersectionExists and triangle_intersection.t < ignore and triangle_intersection.t > offset) {
                                    count_intersections++;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
        double final_percentage = 1. - ((double) (count_intersections+count_invisible_intersections/2.) / (double) nr_rays);

        // darken the initial color
        initial_color *= final_percentage;

        return initial_color;
    }

    // compute intersections for a given ray with all geometric objects that belong to the
    // smallest (and closest) leaf the ray passes through
    Vec3 rayTrace( Ray const & rayStart) {
        Vec3 color;

        // adapt znear to each ray
        Vec3 CameraDir, pos;
        screen_space_to_world_space_ray(0.5, 0.5, pos, CameraDir);
        float initial_z_near = 4.8/Vec3::dot(CameraDir,rayStart.d); // 4.15 ?? 4.8 ?? 4.1 ??

        // call rayTraceRecursive() to generate the tree of rays
        color = rayTraceRecursive(rayStart, 2, initial_z_near); // NRemainingBounces = depth of tree
        // Note: NRemainingBounces must be at least 1 for reflection
        return color;
    }

    void buildKdTree(){
        // build initial bounding box
        TreeNode t;
        scene_tree = &t;
        (*scene_tree).bounding_box.min_corner = Vec3(-3.,-3.,-3.);
        (*scene_tree).bounding_box.max_corner = Vec3(3.,3.,3.);
        (*scene_tree).bounding_box.constructCorners();
        (*scene_tree).bounding_box.drawBoundingBox();
        (*scene_tree).bounding_box.setDimensions();
        (*scene_tree).depth = 1;
        (*scene_tree).isRoot = true;

        // add all primitives of the scene
        for( unsigned int i = 0; i<meshes.size(); i++){

            for( unsigned int j = 0 ; j < meshes[i].triangles.size(); j++) {
                Vec3 c0 = meshes[i].vertices[meshes[i].triangles[j][0]].position;
                Vec3 c1 = meshes[i].vertices[meshes[i].triangles[j][1]].position;
                Vec3 c2 = meshes[i].vertices[meshes[i].triangles[j][2]].position;
                Triangle t = Triangle(c0, c1, c2);
                (*scene_tree).node_data_triangle.push_back(t);
            }
        }

        recursiveKdTree(scene_tree, (*scene_tree).depth, 20);
    }

    void recursiveKdTree(TreeNode * parent, int current_depth, int min_objects){
        //std::cout << "current depth " << current_depth << std::endl;
        //std::cout << "data_in_parent: " << (*parent).node_data_triangle.size() << std::endl;

        // split recursively in two

        // choose dimension to split in two
        int dim_int;
        // take largest dimension
        if ((*parent).bounding_box.dimensions[0] >= (*parent).bounding_box.dimensions[1] and (*parent).bounding_box.dimensions[0] >= (*parent).bounding_box.dimensions[2]) {
            dim_int = 0;
        } else if ((*parent).bounding_box.dimensions[1] >= (*parent).bounding_box.dimensions[0] and (*parent).bounding_box.dimensions[0] >= (*parent).bounding_box.dimensions[2]) {
            dim_int = 1;
        } else if ((*parent).bounding_box.dimensions[2] >= (*parent).bounding_box.dimensions[0] and (*parent).bounding_box.dimensions[2] >= (*parent).bounding_box.dimensions[1]) {
            dim_int = 2;
        }

        // create TreeNodes for children
        TreeNode * child1, * child2, c1, c2;
        child1 = &c1;
        child2 = &c2;
        (*child1).depth = current_depth;
        (*child2).depth = current_depth;

        // create bounding boxes of the two children

        (*child1).bounding_box.dimensions = Vec3((*parent).bounding_box.dimensions[0], (*parent).bounding_box.dimensions[1], (*parent).bounding_box.dimensions[2]);
        (*child1).bounding_box.dimensions[dim_int] = (*child1).bounding_box.dimensions[dim_int] / 2.;
        (*child1).bounding_box.min_corner = (*parent).bounding_box.min_corner;
        (*child1).bounding_box.max_corner = (*child1).bounding_box.min_corner + (*child1).bounding_box.dimensions;

        (*child1).bounding_box.constructCorners();
        (*child1).bounding_box.drawBoundingBox();

        (*child2).bounding_box.dimensions = (*child1).bounding_box.dimensions;
        (*child2).bounding_box.min_corner = (*child1).bounding_box.min_corner;
        (*child2).bounding_box.min_corner[dim_int] = (*child2).bounding_box.min_corner[dim_int] + (*child2).bounding_box.dimensions[dim_int];
        (*child2).bounding_box.max_corner = (*child2).bounding_box.min_corner + (*child2).bounding_box.dimensions;

        (*child2).bounding_box.constructCorners();
        (*child2).bounding_box.drawBoundingBox();

        // fill node data for each child
        bool inBB1, inBB2;
        for( unsigned int i=0; i < (*parent).node_data_triangle.size(); i++){
            inBB1 = (*parent).node_data_triangle[i].isInBoundingBox((*child1).bounding_box);
            inBB2 = (*parent).node_data_triangle[i].isInBoundingBox((*child2).bounding_box);

            if(inBB1){
                (*child1).node_data_triangle.push_back((*parent).node_data_triangle[i]);
            }
            if(inBB2){
                (*child2).node_data_triangle.push_back((*parent).node_data_triangle[i]);
            }
        }

        // set these as children
        (*parent).leftChild = child1;
        (*parent).rightChild = child2;

        int data_in_child1 = (*child1).node_data_triangle.size();
        //std::cout << "data in child 1 : " << data_in_child1 << std::endl;
        int data_in_child2 = (*child2).node_data_triangle.size();
        //std::cout << "data in child 2 : " << data_in_child2 << std::endl;

        if(data_in_child1 > min_objects) {
            recursiveKdTree(child1, current_depth++, min_objects);
        }else{
            (*child1).isLeaf = true;
        }
        if(data_in_child2 > min_objects) {
            recursiveKdTree(child2, current_depth++, min_objects);
        }else{
            (*child2).isLeaf = true;
        }

        //std::cout << "end of recursion loop" << std::endl;
    }

    void setup_single_sphere() {
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3(0.0, 1.5, 0.0); // -5,5,5
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }

        {
            spheres.resize( spheres.size() + 1 );
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(0. , 0. , 0.);
            s.m_radius = 0.4f;
            s.build_arrays();
            s.material.type = Material_Mirror;
            s.material.diffuse_material = Vec3( 1.,0.,0 );
            s.material.specular_material = Vec3( 0.2,0.2,0.2 );
            s.material.shininess = 20;
        }

        // add second sphere
        /*{
            spheres.resize( spheres.size() + 1 );
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(0. , 0. , 1.);
            s.m_radius = 1.f;
            s.build_arrays();
            s.material.type = Material_Mirror;
            s.material.diffuse_material = Vec3( 0.,1.,0 );
            s.material.specular_material = Vec3( 0.2,0.2,0.2 );
            s.material.shininess = 20;
        }*/
    }

    void setup_single_square() {
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3(-5,5,5);
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }

        {
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 0.8, 0.8, 0.8 ); // 0.8, 0.8, 0.8
            s.material.specular_material = Vec3( 0.8, 0.8, 0.8 ); // 0.8, 0.8, 0.8
            s.material.shininess = 20;
        }
    }

    void setup_cornell_box(){
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3( 0.0, 1.5, 0.0 );
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }

        { //Back Wall
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.scale(Vec3(2., 2., 1.));
            s.translate(Vec3(0., 0., -2.));
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.,1.,1. );
            s.material.specular_material = Vec3( 1.,1.,1. );
            s.material.shininess = 16;
        }

        { //Left Wall
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.scale(Vec3(2., 2., 1.));
            s.translate(Vec3(0., 0., -2.));
            s.rotate_y(90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.,0.,0. );
            s.material.specular_material = Vec3( 1.,0.,0. );
            s.material.shininess = 16;
        }

        { //Right Wall
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_y(-90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 0.0,1.0,0.0 );
            s.material.specular_material = Vec3( 0.0,1.0,0.0 );
            s.material.shininess = 16;
        }

        { //Floor
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_x(-90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.0,1.0,1.0 );
            s.material.specular_material = Vec3( 1.0,1.0,1.0 );
            s.material.shininess = 16;
        }

        { //Ceiling
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_x(90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.0,1.0,1.0 );
            s.material.specular_material = Vec3( 1.0,1.0,1.0 );
            s.material.shininess = 16;
        }

        { //Front Wall
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_y(180);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.0,1.0,1.0 );
            s.material.specular_material = Vec3( 1.0,1.0,1.0 );
            s.material.shininess = 16;
        }


        { //MIRRORED Sphere
            spheres.resize( spheres.size() + 1 );
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(-1.0, -1.25, -0.5);
            s.m_radius = 0.75f;
            s.build_arrays();
            s.material.type = Material_Mirror;
            s.material.diffuse_material = Vec3( 1.,0.,0. );
            s.material.specular_material = Vec3( 1.,0.,0. );
            s.material.shininess = 16;
            s.material.transparency = 0.0;
            s.material.index_medium = 0.0;
        }

        // MESH OBJECT
        {
            std::string filename("triangle.off");
            meshes.resize( meshes.size() + 1 );
            Mesh & m = meshes[meshes.size() - 1];
            m.loadOFF(filename);
            m.centerAndScaleToUnit();
            m.translate(Vec3(0.7,0.6,0.));
            m.build_arrays();
            m.build_bounding_box();
            m.material.diffuse_material = Vec3( 1.,0.,1. );
            m.material.specular_material = Vec3( 1.,0.,1. );
            m.material.shininess = 16;
        }


        { //GLASS Sphere
            spheres.resize( spheres.size() + 1 );
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(1.0, -1.24, 0.5);
            s.m_radius = 0.75f;
            s.build_arrays();
            s.material.type = Material_Glass;
            s.material.diffuse_material = Vec3( 1.,1.,1. );
            s.material.specular_material = Vec3(  1.,1.,1. );
            s.material.shininess = 16;
            s.material.transparency = 1.0;
            s.material.index_medium = 1.4;
        }

        /*{ //Invisible Sphere
            spheres.resize( spheres.size() + 1 );
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(1.5, 0.75, -0.5);
            s.m_radius = 0.45f;
            s.build_arrays();
            s.material.type = Material_Invisible;
            s.material.diffuse_material = Vec3( 1.,1.,1. );
            s.material.specular_material = Vec3(  1.,1.,1. );
            s.material.shininess = 16;
            s.material.transparency = 1.0;
            s.material.index_medium = 1.4;
        }*/
    }

    void setup_single_suz() {
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3( 0.0, 2.5, 0.0 );
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }

        {
            std::string filename("suzanne.off");
            meshes.resize( meshes.size() + 1 );
            Mesh & m = meshes[meshes.size() - 1];
            m.loadOFF(filename);
            m.centerAndScaleToUnit();
            m.translate(Vec3(-1.0,0.0,0.0));
            m.build_arrays();
            m.build_bounding_box();
            m.material.diffuse_material = Vec3( 1.,0.,1. );
            m.material.specular_material = Vec3( 1.,0.,1. );
            m.material.shininess = 16;
        }

        /*{
            std::string filename("suzanne.off");
            meshes.resize( meshes.size() + 1 );
            Mesh & m = meshes[meshes.size() - 1];
            m.loadOFF(filename);
            m.centerAndScaleToUnit();
            m.translate(Vec3(1.5,0.0,0.0));
            m.build_arrays();
            m.material.diffuse_material = Vec3( 1.,0.,1. );
            m.material.specular_material = Vec3( 1.,0.,1. );
            m.material.shininess = 16;
        }*/
    }
};



#endif
