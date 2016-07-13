#version 430
#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

subroutine void writeToTexture(int snap, uint deviation, uvec3 n);
subroutine uniform writeToTexture writeToTextureFromView;

layout(r32ui) uniform uimage3D tex[2];
uniform int res;
in vec3 n;

//uint eps = max(res/256,1);
const uint eps = 4;

void writeToTex(ivec3 coord, uvec3 normal, uint d) {
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

layout(index=0) subroutine (writeToTexture) void writeFromXView(int snap, uint deviation, uvec3 n) {
    //  255 = 1      255 = 1
    //-----|------------|------
    if (deviation <= eps) {
        writeToTex(ivec3(snap-1,gl_FragCoord.y,gl_FragCoord.x), n, 255);
    } else if (deviation >= 256-eps) {
        writeToTex(ivec3(snap+1,gl_FragCoord.y,gl_FragCoord.x), n, 1);
    }
    writeToTex(ivec3(snap,gl_FragCoord.y,gl_FragCoord.x), n, deviation);
}

layout(index=1) subroutine (writeToTexture) void writeFromYView(int snap, uint deviation, uvec3 n) {
    //  255 = 1      255 = 1
    //-----|------------|------
    if (deviation <= eps) {
        writeToTex(ivec3(gl_FragCoord.x,snap-1,gl_FragCoord.y), n, 255);
    } else if (deviation >= 256-eps) {
        writeToTex(ivec3(gl_FragCoord.x,snap+1,gl_FragCoord.y), n, 1);
    }
    writeToTex(ivec3(gl_FragCoord.x,snap,gl_FragCoord.y), n, deviation);
}

layout(index=2) subroutine (writeToTexture) void writeFromZView(int snap, uint deviation, uvec3 n) {
    //  255 = 1      255 = 1
    //-----|------------|------
    if (deviation <= eps) {
        writeToTex(ivec3(res-int(gl_FragCoord.x),gl_FragCoord.y, snap-1), n, 255);
    } else if (deviation >= 256-eps) {
        writeToTex(ivec3(res-int(gl_FragCoord.x),gl_FragCoord.y, snap+1), n, 1);
    }
    writeToTex(ivec3(res-int(gl_FragCoord.x),gl_FragCoord.y, snap), n, deviation);
}



void main() {
    // snap -> +----|--+
    //float depth = gl_FragCoord.z*res;
    float depth = gl_FragCoord.z*(res+1)-0.5;
    int snap = int(floor(depth));
    // 0        1
    // |--------|
    // 1       255
    uint d = uint((depth-snap)*254+1.5); //+0.5 to round
    

    uvec3 normal = uvec3((normalize(n)+vec3(1))*0.5*255+0.5); // [-1..1]^3 -> [0..255]^3
    writeToTextureFromView(snap, d, normal);

    //imageAtomicExchange(tex[0], ivec3(159,29,254), 100);

}

