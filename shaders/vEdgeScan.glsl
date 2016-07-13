#version 430
#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

in vec3 a_position;

uniform mat4 uMVPMat;


void main() {
    gl_Position = uMVPMat * vec4(a_position,1.0);
}
