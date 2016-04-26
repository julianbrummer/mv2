#include "bufferobject.h"
#include <iostream>
using namespace Eigen;

UBO::UBO(int bindingPoint, int std140Size) {
    initializeOpenGLFunctions();
    glGenBuffers(1, &id);
    glBindBuffer(GL_UNIFORM_BUFFER, id);
    glBufferData(GL_UNIFORM_BUFFER, std140Size, 0, GL_STATIC_DRAW);
    glBindBufferBase(GL_UNIFORM_BUFFER, bindingPoint, id);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

UBO::~UBO() {
    glDeleteBuffers(1, &id);
}

void VBO::render(QGLShaderProgram& program, int mode) {
    render(program, mode, 0, info.size);
}

void VBO::render(QGLShaderProgram& program, int mode, GLint first, GLint count) {
    glBindBuffer(GL_ARRAY_BUFFER, id);
    int positionLocation = program.attributeLocation("a_position");
    if (positionLocation >= 0) {
        program.enableAttributeArray(positionLocation);
        glVertexAttribPointer(positionLocation, 3, GL_FLOAT, GL_FALSE, info.elemSize, (const void*) info.position_offset);
    }

    int normalLocation = program.attributeLocation("a_normal");
    if (normalLocation >= 0 && info.normal_offset >= 0) {
        program.enableAttributeArray(normalLocation);
        glVertexAttribPointer(normalLocation, 3, GL_FLOAT, GL_FALSE, info.elemSize, (const void*) info.normal_offset);
    }

    int colorLocation = program.attributeLocation("a_color");
    if (colorLocation >= 0 && info.color_offset >= 0) {
        program.enableAttributeArray(colorLocation);
        glVertexAttribPointer(colorLocation, 3, GL_FLOAT, GL_FALSE, info.elemSize, (const void*) info.color_offset);
    }

    glDrawArrays(mode,first,count);

    if (colorLocation >= 0 && info.color_offset >= 0)
        program.disableAttributeArray(colorLocation);
    if (normalLocation >= 0 && info.normal_offset >= 0)
        program.disableAttributeArray(normalLocation);
    if (positionLocation >= 0)
        program.disableAttributeArray(positionLocation);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

VBO::~VBO() {
    glDeleteBuffers(1, &id);
}
