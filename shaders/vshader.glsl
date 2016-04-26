#version 150
#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

in vec3 a_position;
in vec3 a_normal;

uniform mat4 uMVPMat;
uniform mat3 uNMat;

out float vIntensity;

void main() {
    gl_Position = uMVPMat * vec4(a_position,1.0);
    vec3 n = normalize(uNMat * a_normal);
    vIntensity = abs(n.z);
}
