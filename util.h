#ifndef UTIL_H
#define UTIL_H
#include <stdint.h>
#include <Eigen/Core>
#include <QMatrix3x3>
#include <QMatrix4x4>
#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;


void normalMatrix(const Matrix4f& mvMat, Matrix3f& normalMat);

QMatrix4x4 qMat(const Matrix4f &M);

QMatrix3x3 qMat(const Matrix3f &M);

QVector3D qVec(const Vector3f& v);

QVector4D qVec(const Vector4f& v);

#endif // UTIL_H
