#include "util.h"

void normalMatrix(const Matrix4f& mvMat, Matrix3f& normalMat) {
    normalMat = mvMat.block<3,3>(0,0).inverse();
    normalMat.transposeInPlace();
}

QMatrix4x4 qMat(const Matrix4f &M) {
    return QMatrix4x4(M(0,0), M(0,1), M(0,2), M(0,3),
                      M(1,0), M(1,1), M(1,2), M(1,3),
                      M(2,0), M(2,1), M(2,2), M(2,3),
                      M(3,0), M(3,1), M(3,2), M(3,3));
}

QMatrix3x3 qMat(const Matrix3f &M) {
    QMatrix3x3 Q;
    float* d = Q.data();
    *d = M(0,0); d++; *d = M(1,0); d++; *d = M(2,0); d++;
    *d = M(0,1); d++; *d = M(1,1); d++; *d = M(2,1); d++;
    *d = M(0,2); d++; *d = M(1,2); d++; *d = M(2,2); d++;
    return Q;
}

QVector3D qVec(const Vector3f& v) {
    return QVector3D(v[0], v[1], v[2]);
}

QVector4D qVec(const Vector4f& v) {
    return QVector4D(v[0], v[1], v[2], v[3]);
}
