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

QMatrix4x4 qMat(Matrix4f &M);

QMatrix3x3 qMat(Matrix3f &M);

#endif // UTIL_H
