#ifndef BUFFEROBJECT_H
#define BUFFEROBJECT_H

#include <Eigen/Core>
#include <QOpenGLFunctions_4_3_Core>
#include <QOpenGLFunctions_4_2_Core>
#include <QGLShaderProgram>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

using namespace Eigen;
using namespace std;

class SSBO : private QOpenGLFunctions_4_3_Core {
public:
    SSBO(int bindingPoint, int std140Size, int usage);

    SSBO& bufferData(GLsizeiptr size, const GLvoid* data, GLenum usage);
    SSBO& bufferSubData(GLintptr offset, GLsizeiptr size, const GLvoid* data);
    void bind();
    void unBind();
    ~SSBO();
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


class VBO : private QOpenGLFunctions_4_2_Core {
private:
    void bindAttibutes(QGLShaderProgram& program, int positionLocation, int normalLocation, int colorLocation);
    void unbindAttibutes(QGLShaderProgram& program, int positionLocation, int normalLocation, int colorLocation);
public:

    template <typename VBO_DATA>
    VBO(const vector<VBO_DATA, aligned_allocator<VBO_DATA>>& data, VBOInfo& info, int usage) : info(info) {
        initializeOpenGLFunctions();
        this->info.elemSize = sizeof(VBO_DATA);
        this->info.size = data.size();
        glGenBuffers(1,&id);
        glBindBuffer(GL_ARRAY_BUFFER,id);
        glBufferData(GL_ARRAY_BUFFER,data.size()*this->info.elemSize,data.data(), usage);
    }

    void render(QGLShaderProgram& program, int mode);
    void render(QGLShaderProgram& program, int mode, GLint first, GLsizei count);
    void renderInstanced(QGLShaderProgram& program, int mode, GLsizei instanceCount, GLuint baseInstance = 0);
    void renderInstanced(QGLShaderProgram& program, int mode, GLint first, GLsizei count, GLsizei instanceCount, GLuint baseInstance = 0);
    ~VBO();

    VBOInfo info;
    GLuint id;
};

#endif // BUFFEROBJECT_H
