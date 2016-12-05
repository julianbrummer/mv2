#version 430
#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

subroutine void writeToTexture(int snap, float deviation);
subroutine uniform writeToTexture writeToTextureFromView;

layout(r32ui) uniform uimage3D tex[2];
uniform int res;

const uint data[32] = uint[] (1,2,4,8,16,32,64,128,
                             256,512,1024,2048,4096,8192,16384,32768,
                             65536,131072,262144,524288,1048576,2097152,4194304,8388608,
                             16777216,33554432,67108864,134217728,268435456,536870912,1073741824,2147483648);

const float eps = 0.015; // works well for 1024
//const float eps = 0.00;

layout(index=0) subroutine (writeToTexture) void writeFromXView(int snap, float deviation) {
    int zTexCoord = int(gl_FragCoord.x / 32);
    int offset = int(gl_FragCoord.x) - 32*zTexCoord;

    if (deviation <= eps) {
        imageAtomicOr(tex[gl_FrontFacing? 0 : 1], ivec3(snap-1,gl_FragCoord.y,zTexCoord), data[offset]);
    } else if (deviation >= 1-eps) {
        imageAtomicOr(tex[gl_FrontFacing? 0 : 1], ivec3(snap+1,gl_FragCoord.y,zTexCoord), data[offset]);
    }
    imageAtomicOr(tex[gl_FrontFacing? 0 : 1], ivec3(snap,gl_FragCoord.y,zTexCoord), data[offset]);
}

layout(index=1) subroutine (writeToTexture) void writeFromYView(int snap, float deviation) {
    int zTexCoord = int(gl_FragCoord.y / 32);
    int offset = int(gl_FragCoord.y) - 32*zTexCoord;

    if (deviation <= eps) {
        imageAtomicOr(tex[gl_FrontFacing? 0 : 1], ivec3(gl_FragCoord.x,snap-1,zTexCoord), data[offset]);
    } else if (deviation >= 1-eps) {
        imageAtomicOr(tex[gl_FrontFacing? 0 : 1], ivec3(gl_FragCoord.x,snap+1,zTexCoord), data[offset]);
    }

    imageAtomicOr(tex[gl_FrontFacing? 0 : 1], ivec3(gl_FragCoord.x,snap,zTexCoord), data[offset]);
}

void writeToZ(int z) {
    int zTexCoord = z / 32;
    int offset = z - 32*zTexCoord;
    //imageAtomicOr(tex[gl_FrontFacing? 0 : 1], ivec3(res-int(gl_FragCoord.x),gl_FragCoord.y, zTexCoord), data[offset]);
    imageAtomicOr(tex[gl_FrontFacing? 0 : 1], ivec3(gl_FragCoord.x,gl_FragCoord.y, zTexCoord), data[offset]);
}

layout(index=2) subroutine (writeToTexture) void writeFromZView(int snap, float deviation) {

    if (deviation <= eps) {
        writeToZ(snap-1);
    } else if (deviation >= 1-eps) {
        writeToZ(snap+1);
    }
    writeToZ(snap);
}



void main() {
    float depth = gl_FragCoord.z*res;
    int snap = int(floor(depth));
    float deviation = depth - snap;

    if (deviation <= eps) {
        writeToZ(snap-1);
    } else if (deviation >= 1-eps) {
        writeToZ(snap+1);
    }
    writeToZ(snap);

    //writeToTextureFromView(snap, depth-snap);
}

