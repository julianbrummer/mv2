#include "scene.h"
#include <iostream>
#include <array>

SceneObject::SceneObject() {
    position.setZero(3);
    rotation.setIdentity();
}

SceneObject& SceneObject::setPosition(const Vector3f &pos) {
    position = pos;
    return *this;
}

SceneObject& SceneObject::translate(const Vector3f &t) {
    position += t;
    return *this;
}

SceneObject& SceneObject::translate(float x, float y, float z) {
    translate(Vector3f(x,y,z));
    return *this;
}

SceneObject& SceneObject::rotate(const Quaternionf& q) {
    rotation = q * rotation;
    rotation.normalize();
    return *this;
}

SceneObject& SceneObject::rotate(const AngleAxisf& angleAxis) {
    rotate(Quaternion<float>(angleAxis));
    return *this;
}

SceneObject& SceneObject::rotate(float angle, const Vector3f& axis) {
    rotate(Quaternion<float>(AngleAxisf(angle, axis)));
    return *this;
}

SceneObject& SceneObject::setRotation(const Quaternionf& q) {
    rotation = q;
    rotation.normalize();
    return *this;
}

SceneObject& SceneObject::setRotation(const Matrix3f& M) {
    rotation = Quaternionf(M);
    rotation.normalize();
    return *this;
}

SceneObject& SceneObject::setRotation(const Vector3f& x, const Vector3f& y, const Vector3f& z) {
    Matrix3f M;
    M.col(0) = x;
    M.col(1) = y;
    M.col(2) = z;
    rotation = Quaternionf(M);
    rotation.normalize();
    return *this;
}

SceneObject& SceneObject::setRotation(const AngleAxisf &angleAxis) {
    rotation = Quaternionf(angleAxis);
    rotation.normalize();
    return *this;
}

SceneObject& SceneObject::setRotation(float angle, const Vector3f& axis) {
    setRotation(AngleAxisf(angle, axis));
    return *this;
}

Model::Model(const aligned_vector3f &positions, const vector<uint> &triangles, bool centerAndScale, float size)
        : SceneObject(), mode(GL_TRIANGLES), positions(positions) {
    aligned_vectorPosNormal posAndNormals;
    for (uint i = 0; i < triangles.size(); i+=3) {
        const Vector3f& v0 = positions[triangles[i]];
        const Vector3f& v1 = positions[triangles[i+1]];
        const Vector3f& v2 = positions[triangles[i+2]];
        Vector3f n = (v1-v0).cross(v2-v0);
        n.normalize();
        posAndNormals.push_back(PositionNormal(v0, n));
        posAndNormals.push_back(PositionNormal(v1, n));
        posAndNormals.push_back(PositionNormal(v2, n));
    }
    VBOInfo posNormal(0,sizeof(Vector3f));
    vbo = shared_ptr<VBO>(new VBO(posAndNormals, posNormal, GL_STATIC_DRAW));
    if (centerAndScale)
        init(size);
    else {
        center.setZero(3);
        scale = 1.0;
    }
}

Model::Model(const aligned_vector3f &positions, const aligned_vector3f &normals, int mode, bool centerAndScale, float size)
        : SceneObject(), mode(mode), positions(positions) {
    aligned_vectorPosNormal posAndNormals;
    for (uint i = 0; i < positions.size(); ++i) {
        posAndNormals.push_back(PositionNormal(positions[i], normals[i]));
    }
    VBOInfo posNormal(0,sizeof(Vector3f));
    vbo = shared_ptr<VBO>(new VBO(posAndNormals, posNormal, GL_STATIC_DRAW));
    if (centerAndScale)
        init(size);
    else {
        center.setZero(3);
        scale = 1.0;
    }
}

Model::Model(const aligned_vector3f &positions, const aligned_vector3f &colors, bool centerAndScale, int mode, float size)
        : SceneObject(), mode(mode), positions(positions) {
    aligned_vectorPosColor posAndColors;
    for (uint i = 0; i < positions.size(); ++i) {
        posAndColors.push_back(PositionColor(positions[i], colors[i]));
    }
    VBOInfo posColor(0,-1,sizeof(Vector3f));
    vbo = shared_ptr<VBO>(new VBO(posAndColors, posColor, GL_STATIC_DRAW));
    if (centerAndScale)
        init(size);
    else {
        center.setZero(3);
        scale = 1.0;
    }
}

Model::Model(const aligned_vector3f &positions, int mode, bool centerAndScale, float size)
        : SceneObject(), mode(mode), positions(positions) {
    VBOInfo posOnly(0,-1,-1);
    vbo = shared_ptr<VBO>(new VBO(positions, posOnly, GL_STATIC_DRAW));
    if (centerAndScale)
        init(size);
    else {
        center.setZero(3);
        scale = 1.0;
    }
}

void Model::centerAndScale(float size) {
    center = (min+max)*0.5;
    ext = max - min;
    scale = size/std::max(std::max(ext.x(),ext.y()),ext.z());
}

void Model::init(float size, const Trafo& T) {
    init(size);
    aligned_vector3f positions;
    positions.resize(T.size()*8);
    array<Vector4f, 8> v {
        Vector4f(min[0], min[1], min[2], 1),
        Vector4f(min[0], min[1], max[2], 1),
        Vector4f(min[0], max[1], min[2], 1),
        Vector4f(min[0], max[1], max[2], 1),
        Vector4f(max[0], min[1], min[2], 1),
        Vector4f(max[0], min[1], max[2], 1),
        Vector4f(max[0], max[1], min[2], 1),
        Vector4f(max[0], max[1], max[2], 1),
    };


    for (uint t = 0; t < T.size(); ++t) {
        for (int i = 0; i < 8; ++i) {
            positions[8*t+i] = (T[t] * v[i]).block<3,1>(0,0);
        }
    }
    aabb(positions, min[0], min[1], min[2], max[0], max[1], max[2]);
    centerAndScale(size);
}

void Model::init(float size) {
    aabb(positions, min[0], min[1], min[2], max[0], max[1], max[2]);
    centerAndScale(size);
}

void Model::aabb(const aligned_vector3f& positions, const Trafo& T,
                 float& x1, float& y1, float& z1, float& x2, float& y2, float& z2) {
    double inf = std::numeric_limits<double>::infinity();

    x1 = y1 = z1 =  inf;
    x2 = y2 = z2 = -inf;
    for (uint t = 0; t < T.size(); ++t) {
        for(uint i = 0; i < positions.size(); i++) {
            Vector3f v = (T[t] * positions[i].homogeneous()).block<3,1>(0,0);
            if (v.x() < x1) x1 = v.x();
            if (v.x() > x2) x2 = v.x();
            if (v.y() < y1) y1 = v.y();
            if (v.y() > y2) y2 = v.y();
            if (v.z() < z1) z1 = v.z();
            if (v.z() > z2) z2 = v.z();
        }
    }
}

void Model::aabb(const aligned_vector3f& positions,
                 float& x1, float& y1, float& z1, float& x2, float& y2, float& z2) {
    double inf = std::numeric_limits<double>::infinity();

    x1 = y1 = z1 =  inf;
    x2 = y2 = z2 = -inf;

    for(uint i = 0; i < positions.size(); i++) {
        const Vector3f& v = positions[i];
        if (v.x() < x1) x1 = v.x();
        if (v.x() > x2) x2 = v.x();
        if (v.y() < y1) y1 = v.y();
        if (v.y() > y2) y2 = v.y();
        if (v.z() < z1) z1 = v.z();
        if (v.z() > z2) z2 = v.z();
    }
}

Matrix4f Model::getModelMatrix() const {
    Matrix4f M;
    Matrix3f R = rotation.toRotationMatrix();
    M.setIdentity();
    // then rotate
    M.block<3,3>(0,0) = R;
    // then scale
    M = M * Scaling(Vector4f(scale, scale, scale, 1));
    // translate to center first
    Affine3f a(Translation3f(-center));
    M = M * a.matrix();

    // finally set position
    M.block<3,1>(0,3) += position;

    return M;
}

void Model::render(QGLShaderProgram &program) const {
    vbo->render(program, mode);
}

void Model::render(QGLShaderProgram &program, GLint first, GLint count) const {
    vbo->render(program, mode, first, count);
}

void Model::renderInstanced(QGLShaderProgram &program, GLsizei instanceCount, GLuint baseInstance) const {
    vbo->renderInstanced(program, mode, instanceCount, baseInstance);
}

void Model::renderInstanced(QGLShaderProgram &program, GLint first, GLint count, GLsizei instanceCount, GLuint baseInstance) const {
    vbo->renderInstanced(program, mode, first, count, instanceCount, baseInstance);
}

void Camera::lookAt(const Vector3f &center, const Vector3f &eye, Vector3f &up) {
    Vector3f z = eye - center;
    z.normalize();
    up.normalize();
    Vector3f x = up.cross(z);
    x.normalize();
    Vector3f y = z.cross(x);
    Matrix3f M;
    M.col(0) = x;
    M.col(1) = y;
    M.col(2) = z;
    std::cout << M << std::endl;
    rotation = Quaternionf(M);
    position = eye;
}

void Camera::perspective(float fovy, float aspect, float zNear, float zFar) {
    float top    =  tan(fovy/2.0f) * zNear;
    float bottom =  -top;
    float left   = aspect * bottom;
    float right  = aspect * top;
    frustum(left, right, bottom, top, zNear, zFar);
}

void Camera::frustum(float left, float right, float bottom,
                     float top, float zNear, float zFar) {

    float dx = right - left;
    float dy = top - bottom;
    float dz = zFar - zNear;
    projection <<
         2*zNear/dx,    0, 			(right+left)/dx,  0,
         0, 			2*zNear/dy, (top+bottom)/dy,  0,
         0, 			0, 			-(zFar+zNear)/dz, -2*zFar*zNear/dz,
         0, 			0, 			-1, 			  0;
}

Vector3f Camera::forwd() const {
    return rotation._transformVector(-Z_AXIS);
}

Vector3f Camera::upwd() const {
    return rotation.toRotationMatrix().transpose()*Y_AXIS;
}

Vector3f Camera::sidewd() const {
    return rotation.toRotationMatrix().transpose()*X_AXIS;
}

Matrix4f Camera::getViewMatrix() const {
    Affine3f view;
    view.setIdentity();
    view.translate(-zoom*Z_AXIS).rotate(rotation).translate(-center);
    return view.matrix();
}
