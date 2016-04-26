#version 430
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

void main() {
    n = normalize(uNMat * a_normal);
    gl_Position = uMVPMat * vec4(a_position,1.0);
}
