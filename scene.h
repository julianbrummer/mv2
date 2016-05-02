#ifndef MODEL_H
#define MODEL_H

#include <Eigen/Geometry>
#include <memory>
#include "bufferobject.h"
#include "def.h"

using namespace Eigen;
using namespace std;


class SceneObject {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SceneObject();
    SceneObject& setPosition(const Vector3f& pos);
    SceneObject& translate(const Vector3f& t);
    SceneObject& translate(float x, float y, float z);
    SceneObject& rotate(const Quaternionf& q);
    SceneObject& rotate(const AngleAxisf& angleAxis);
    SceneObject& rotate(float angle,const Vector3f& axis);
    SceneObject& setRotation(const Quaternionf& q);
    SceneObject& setRotation(const Matrix3f& M);
    SceneObject& setRotation(const Vector3f& x, const Vector3f& y, const Vector3f& z);
    SceneObject& setRotation(const AngleAxisf& angleAxis);
    SceneObject& setRotation(float angle, const Vector3f& axis);

    Vector3f position;
    Quaternionf rotation;
};

class Model : public SceneObject {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Model(const aligned_vector3f& positions,
          const vector<uint>& triangles, bool centerAndScale = true, float size = 1.0f);
    Model(const aligned_vector3f& positions,
          const aligned_vector3f& normals,
          int mode = GL_TRIANGLES, bool centerAndScale = true, float size = 1.0f);
    Model(const aligned_vector3f& positions,
          const aligned_vector3f& colors,
          bool centerAndScale = true, int mode = GL_TRIANGLES, float size = 1.0f);
    Model(const aligned_vector3f& positions,
          int mode = GL_TRIANGLES, bool centerAndScale = true, float size = 1.0f);
    void render(QGLShaderProgram& program);
    void render(QGLShaderProgram &program, GLint first, GLint count);
    Matrix4f getModelMatrix() const;
    Vector3f center;
    float scale;
private:
    void init(const aligned_vector3f &positions, float size);
    void aabb(const aligned_vector3f& positions,
              float& x1, float& y1, float& z1, float& x2, float& y2, float& z2);


    int mode;
    Vector3f min, max, ext;

    shared_ptr<VBO> vbo;

};


class Camera : public SceneObject {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Camera() : SceneObject(), zoom(1.0f) {
        projection.setZero(4,4);
        center.setZero(3);
    }
    Matrix4f getViewMatrix() const;
    void lookAt(const Vector3f& center, const Vector3f& eye, Vector3f& up);
    void perspective(float fovy, float aspect, float zNear, float zFar);
    void frustum(float left, float right, float bottom,
                         float top, float zNear, float zFar);
    Vector3f upwd() const;
    Vector3f sidewd() const;
    Vector3f forwd() const;
    float zoom;
    Matrix4f projection;
    Vector3f center;

};

#endif // MODEL_H
