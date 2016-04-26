#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

in vec3 a_position;
in vec3 a_color;

uniform mat4 uMVPMat;
uniform vec4 uColor;
uniform bool useVertexColor;

out vec3 color;

void main() {
    if (useVertexColor)
        color = a_color;
    else
        color = uColor;
    gl_Position = uMVPMat * vec4(a_position,1.0);
}

