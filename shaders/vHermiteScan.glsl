#version 430
#extension GL_ARB_shader_draw_parameters : require
#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

in vec3 a_position;
in vec3 a_normal;

uniform mat4 uMVPMat;
uniform mat3 uNMat;

out vec3 n;

subroutine mat4 getMVPMat();
subroutine mat3 getNMat();
subroutine uniform getMVPMat mvpMatFromLocation;
subroutine uniform getNMat nMatFromLocation;

layout(std140, binding = 1) buffer trafo {
    mat4 mvpMat[];
};

layout(std140, binding = 2) buffer trafo_normal {
    mat3 nMat[];
};

layout(index=0) subroutine (getMVPMat) mat4 mvpMatFromUniform() {
    return uMVPMat;
}

layout(index=1) subroutine (getMVPMat) mat4 mvpMatFromBuffer() {
    return mvpMat[gl_InstanceID + gl_BaseInstanceARB];
}

layout(index=2) subroutine (getNMat) mat3 nMatFromUniform() {
    return uNMat;
}

layout(index=3) subroutine (getNMat) mat3 nMatFromBuffer() {
    return nMat[gl_InstanceID + gl_BaseInstanceARB];
}

void main() {
    n = normalize(nMatFromLocation() * a_normal);
    gl_Position = mvpMatFromLocation() * vec4(a_position,1.0);
}
