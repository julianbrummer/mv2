#version 430
#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

layout(r32ui) uniform uimage3D tex[2];
uniform int res;

in vec3 n;

void writeToTexture(ivec3 coord, uvec3 normal, uint d) {
    d = d << 24; // distance from grid point [0..1) -> [1..255] 1. byte
    d += (normal.x << 16); // 2. byte
    d += (normal.y << 8); // 3. byte
    d += normal.z; // 4. byte
    if(gl_FrontFacing) {
        if (imageAtomicCompSwap(tex[0], coord, 0, d) != 0) //just write if current = 0
            imageAtomicMin(tex[0], coord, d); // current != 0
    } else {
        imageAtomicMax(tex[1], coord, d);
    }
}

void main() {

    // snap -> +----|--+
    float real = gl_FragCoord.z*res;
    int snap = int(floor(real));
    // 0        1
    // |--------|
    // 1       255
    uint d = uint((real-snap)*254+1.5); //+0.5 to round
    uvec3 normal = uvec3((normalize(n)+vec3(1))*0.5*255+0.5); // [-1..1]^3 -> [0..255]^3
    //  255 = 1      255 = 1
    //-----|------------|------
    if (d == 1) {
        writeToTexture(ivec3(gl_FragCoord.x, gl_FragCoord.y, snap), normal, 1);
        writeToTexture(ivec3(gl_FragCoord.x, gl_FragCoord.y, snap-1), normal, 255);
    } else if (d == 255) {
        writeToTexture(ivec3(gl_FragCoord.x, gl_FragCoord.y, snap), normal, 255);
        writeToTexture(ivec3(gl_FragCoord.x, gl_FragCoord.y, snap+1), normal, 1);
    } else {
        writeToTexture(ivec3(gl_FragCoord.x, gl_FragCoord.y, snap), normal, d);
    }

    //imageAtomicExchange(tex[0], ivec3(1,1,1), 100);

}

