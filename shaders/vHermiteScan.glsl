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

flat out vec3 n;

subroutine mat4 getMMat();
subroutine uniform getMMat mMatFromLocation;

layout(std140, binding = 1) buffer trafo {
    mat4 mMat[];
};

layout(index=0) subroutine (getMMat) mat4 mMatFromUniform() {
    return uMMat;
}

layout(index=1) subroutine (getMMat) mat4 mMatFromBuffer() {
    return mMat[gl_InstanceID + gl_BaseInstanceARB];
}

void main() {
    n = normalize(mat3(mMatFromLocation()) * a_normal);
    gl_Position = uVPMat * mMatFromLocation() * vec4(a_position,1.0);
}
