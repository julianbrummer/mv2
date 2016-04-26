#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif
out vec4 out_color;
uniform vec4 uColor;
in float vIntensity;

void main()
{
    /*
    if (gl_FrontFacing)
        out_color = vIntensity*vec4(0.0,0.0,1.0,0.0);
    else
        out_color = vIntensity*vec4(1.0,0.0,0.0,0.0);
    */
    out_color = vIntensity*uColor;
}

