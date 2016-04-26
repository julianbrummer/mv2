#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif
out vec4 out_color;
in vec3 color;

void main()
{
    out_color = vec4(abs(color), 1.0);
}

