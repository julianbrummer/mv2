#version 430
#extension GL_ARB_shader_draw_parameters : require
#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

in vec3 a_position;
in vec3 a_normal;

uniform mat4 uVPMat;
uniform mat4 uMMat;
uniform mat3 uNMat;

flat out vec3 n;

subroutine mat4 getMMat();
subroutine mat3 getNMat();
subroutine uniform getMMat mMatFromLocation;
subroutine uniform getNMat nMatFromLocation;

layout(std140, binding = 1) buffer trafo {
    mat4 mMat[];
};

layout(std140, binding = 2) buffer trafo_normal {
    mat3 nMat[];
};

layout(index=0) subroutine (getMMat) mat4 mMatFromUniform() {
    return uMMat;
}

layout(index=1) subroutine (getMMat) mat4 mMatFromBuffer() {
    return mMat[gl_InstanceID + gl_BaseInstanceARB];
}

layout(index=2) subroutine (getNMat) mat3 nMatFromUniform() {
    return uNMat;
}

layout(index=3) subroutine (getNMat) mat3 nMatFromBuffer() {
    return nMat[gl_InstanceID + gl_BaseInstanceARB];
}

void main() {
    n = normalize(nMatFromLocation() * a_normal);
    gl_Position = uVPMat * mMatFromLocation() * vec4(a_position,1.0);
}
