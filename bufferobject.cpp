#include "bufferobject.h"
#include <iostream>
using namespace Eigen;

ACBO::ACBO(int bindingPoint, int size, int usage) : usage(usage) {
    initializeOpenGLFunctions();
    glGenBuffers(1, &id);
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, id);
    glBufferData(GL_ATOMIC_COUNTER_BUFFER, size, 0, usage);
    glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, bindingPoint, id);
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, 0);
}

ACBO& ACBO::bufferData(GLsizeiptr size, const GLvoid* data) {
    glBufferData(GL_ATOMIC_COUNTER_BUFFER, size, data, usage);
    return *this;
}

ACBO& ACBO::bufferSubData(GLintptr offset, GLsizeiptr size, const GLvoid* data) {
    glBufferSubData(GL_ATOMIC_COUNTER_BUFFER, offset, size, data);
    return *this;
}

ACBO& ACBO::getBufferSubData(GLintptr offset, GLsizeiptr size, GLvoid* data) {
    glGetBufferSubData(GL_ATOMIC_COUNTER_BUFFER, offset, size, data);
    return *this;
}

void ACBO::bind() {
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, id);
}
void ACBO::unBind() {
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, 0);
}

ACBO::~ACBO() {
    glDeleteBuffers(1, &id);
}

SSBO::SSBO(int bindingPoint, int stdSize, int usage) : usage(usage) {
    initializeOpenGLFunctions();
    glGenBuffers(1, &id);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, id);
    glBufferData(GL_SHADER_STORAGE_BUFFER, stdSize, 0, usage);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, bindingPoint, id);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

SSBO& SSBO::bufferData(GLsizeiptr size, const GLvoid* data) {
    glBufferData(GL_SHADER_STORAGE_BUFFER, size, data, usage);
    return *this;
}

SSBO& SSBO::bufferSubData(GLintptr offset, GLsizeiptr size, const GLvoid* data) {
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, offset, size, data);
    return *this;
}

SSBO& SSBO::getBufferSubData(GLintptr offset, GLsizeiptr size, GLvoid* data) {
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, offset, size, data);
    return *this;
}

void SSBO::bind() {
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, id);
}
void SSBO::unBind() {
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

SSBO::~SSBO() {
    glDeleteBuffers(1, &id);
}

void VBO::bindAttibutes(QGLShaderProgram& program, int positionLocation, int normalLocation, int colorLocation) {

    if (positionLocation >= 0) {
        program.enableAttributeArray(positionLocation);
        glVertexAttribPointer(positionLocation, 3, GL_FLOAT, GL_FALSE, info.elemSize, (const void*) info.position_offset);
    }

    if (normalLocation >= 0 && info.normal_offset >= 0) {
        program.enableAttributeArray(normalLocation);
        glVertexAttribPointer(normalLocation, 3, GL_FLOAT, GL_FALSE, info.elemSize, (const void*) info.normal_offset);
    }

    if (colorLocation >= 0 && info.color_offset >= 0) {
        program.enableAttributeArray(colorLocation);
        glVertexAttribPointer(colorLocation, 3, GL_FLOAT, GL_FALSE, info.elemSize, (const void*) info.color_offset);
    }
}

void VBO::unbindAttibutes(QGLShaderProgram& program, int positionLocation, int normalLocation, int colorLocation) {
    if (colorLocation >= 0 && info.color_offset >= 0)
        program.disableAttributeArray(colorLocation);
    if (normalLocation >= 0 && info.normal_offset >= 0)
        program.disableAttributeArray(normalLocation);
    if (positionLocation >= 0)
        program.disableAttributeArray(positionLocation);
}

void VBO::render(QGLShaderProgram& program, int mode) {
    render(program, mode, 0, info.size);
}

void VBO::render(QGLShaderProgram& program, int mode, GLint first, GLsizei count) {
    glBindBuffer(GL_ARRAY_BUFFER, id);
    int positionLocation = program.attributeLocation("a_position");
    int normalLocation = program.attributeLocation("a_normal");
    int colorLocation = program.attributeLocation("a_color");
    bindAttibutes(program, positionLocation, normalLocation, colorLocation);

    glDrawArrays(mode,first,count);

    unbindAttibutes(program, positionLocation, normalLocation, colorLocation);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void VBO::renderInstanced(QGLShaderProgram& program, int mode, GLsizei instanceCount, GLuint baseInstance) {
    renderInstanced(program, mode, 0, info.size, instanceCount, baseInstance);
}

void VBO::renderInstanced(QGLShaderProgram& program, int mode, GLint first, GLsizei count, GLsizei instanceCount, GLuint baseInstance) {
    glBindBuffer(GL_ARRAY_BUFFER, id);
    int positionLocation = program.attributeLocation("a_position");
    int normalLocation = program.attributeLocation("a_normal");
    int colorLocation = program.attributeLocation("a_color");
    bindAttibutes(program, positionLocation, normalLocation, colorLocation);

    glDrawArraysInstancedBaseInstance(mode, first, count, instanceCount, baseInstance);

    unbindAttibutes(program, positionLocation, normalLocation, colorLocation);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}


VBO::~VBO() {
    glDeleteBuffers(1, &id);
}
