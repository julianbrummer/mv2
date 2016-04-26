#ifndef DEF_H
#define DEF_H

#include <Eigen/Geometry>
#include <memory>

using namespace Eigen;
using namespace std;

const Vector3f X_AXIS = Vector3f(1,0,0);
const Vector3f Y_AXIS = Vector3f(0,1,0);
const Vector3f Z_AXIS = Vector3f(0,0,1);

struct PositionNormal {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vector3f position, normal;

    PositionNormal(const Vector3f& position, const Vector3f& normal)
        : position(position), normal(normal) {}
};

struct PositionColor {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vector3f position, color;

    PositionColor(const Vector3f& position, const Vector3f& color)
        : position(position), color(color) {}
};
typedef unsigned int uint;
typedef vector<PositionColor, aligned_allocator<PositionColor>> aligned_vectorPosColor;
typedef vector<PositionNormal, aligned_allocator<PositionNormal>> aligned_vectorPosNormal;
typedef std::vector<Vector3f, aligned_allocator<Vector3f>> aligned_vector3f;
typedef vector<vector<uint>> vector_2_uint;
typedef vector<vector_2_uint> vector_3_uint;
typedef vector<vector_3_uint> vector_4_uint;

#endif // DEF_H
