#ifndef BUFFEROBJECT_H
#define BUFFEROBJECT_H

#include <Eigen/Core>
#include <QOpenGLFunctions_2_0>
#include <QOpenGLFunctions_3_1>
#include <QGLShaderProgram>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

using namespace Eigen;
using namespace std;

class UBO  : private QOpenGLFunctions_3_1 {
public:
    UBO(int bindingPoint, int std140Size);

    template <typename UBO_DATA>
    UBO& set(int offset, UBO_DATA data) {
        glBindBuffer(GL_UNIFORM_BUFFER, id);
        glBufferSubData(GL_UNIFORM_BUFFER, offset, sizeof(UBO_DATA), &data);
        glBindBuffer(GL_UNIFORM_BUFFER, 0);
        return *this;
    }

    ~UBO();

private:
    GLuint id;
};

struct VBOInfo {
    uint position_offset;
    int normal_offset, color_offset;
    uint size, elemSize;
    VBOInfo(uint position_offset, int normal_offset = -1, int color_offset = -1)
        : position_offset(position_offset), normal_offset(normal_offset), color_offset(color_offset),
          size(0), elemSize(0){}
};


class VBO : private QOpenGLFunctions_2_0 {
public:

    template <typename VBO_DATA>
    VBO(const vector<VBO_DATA, aligned_allocator<VBO_DATA>>& data, VBOInfo& info) : info(info) {
        initializeOpenGLFunctions();
        this->info.elemSize = sizeof(VBO_DATA);
        this->info.size = data.size();
        glGenBuffers(1,&id);
        glBindBuffer(GL_ARRAY_BUFFER,id);
        glBufferData(GL_ARRAY_BUFFER,data.size()*this->info.elemSize,data.data(), GL_STATIC_DRAW);
    }

    void render(QGLShaderProgram& program, int mode);
    void render(QGLShaderProgram& program, int mode, GLint first, GLint count);
    ~VBO();

    VBOInfo info;
    GLuint id;
};

#endif // BUFFEROBJECT_H
