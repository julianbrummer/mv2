#version 430
#extension GL_ARB_shader_draw_parameters : require
#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

in vec3 a_position;

uniform mat4 uMVPMat;

subroutine mat4 getMVPMat();
subroutine uniform getMVPMat mvpMatFromLocation;

layout(std140, binding = 1) buffer trafo {
    mat4 mvpMat[];
};

layout(index=0) subroutine (getMVPMat) mat4 mvpMatFromUniform() {
    return uMVPMat;
}

layout(index=1) subroutine (getMVPMat) mat4 mvpMatFromBuffer() {
    return mvpMat[gl_InstanceID + gl_BaseInstanceARB];
}

void main() {
    gl_Position = mvpMatFromLocation() * vec4(a_position,1.0);
}
