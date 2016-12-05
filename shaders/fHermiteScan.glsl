#version 430
#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

subroutine ivec4 vecFromView(int x, int y, int z);
subroutine uniform vecFromView getVecFromView;

struct Intersection {
    ivec4 e;
    uint d;
};

coherent layout(std140, binding = 3) buffer data {
    Intersection cut[];
};

//coherent layout(std140, binding = 4) buffer sync {
//     uint lock[];
//};

uniform int res;
flat in vec3 n;
const float eps = 0.015; // works well for 1024
//const float eps = 0.00;

bool greater(ivec4 a, ivec4 b) {
    for (int i = 3; i >= 0; i--) {
        if (a[i] > b[i])
                return true;
        if (a[i] < b[i])
                return false;
    }
    return false;
}

int contributes(ivec4 e){
	
    int left = 0;
    int right = cut.length()-1;
    int m = 0;

    while (left <= right) {
        m = (right + left) / 2;
        if (all(equal(e, cut[m].e))) {
                return m;
        }
        if (greater(e, cut[m].e)){
                left = m + 1;
        } else {
                right = m - 1;
        }
    }

    return -1;
}

layout(index=0) subroutine (vecFromView) ivec4 vecFromXView(int x, int y, int z) {
        return ivec4(z,y+1,x+1, gl_FrontFacing? 1 : -1);
}

layout(index=1) subroutine (vecFromView) ivec4 vecFromYView(int x, int y, int z) {
        return ivec4(x+1,z,y+1,gl_FrontFacing? 2 : -2);
}

layout(index=2) subroutine (vecFromView) ivec4 vecFromZView(int x, int y, int z) {
        return ivec4(res-x-1,y+1,z,gl_FrontFacing? 3 : -3);
}

uint code(float deviation, vec3 normal) {
    uint d = uint(deviation*255+0.5); //+0.5 to round
    uvec3 n = uvec3((normal+vec3(1))*0.5*255+vec3(0.5)); // [-1..1]^3 -> [0..255]^3
    d = d << 24; // distance from grid point [0..1) -> [0..255] 1. byte
    d += (n.x << 16); // 2. byte
    d += (n.y << 8); // 3. byte
    d += n.z; // 4. byte
    return d;
}

void tryWrite(ivec4 edge, float deviation) {
    int index = contributes(edge);
    if (index >= 0) {
        uint d = code(deviation, n);
        if(gl_FrontFacing) {
            if (atomicCompSwap(cut[index].d, 0, d) != 0) //just write if current = 0
                atomicMin(cut[index].d, d); // current != 0
        } else {
            atomicMax(cut[index].d, d);
        }
    }
}

void main() {
    // snap -> +----|--+
    float d = gl_FragCoord.z*res;
    int snap = int(floor(d));
    float deviation = d - snap;
    if (deviation <= eps) {
        tryWrite(getVecFromView(int(gl_FragCoord.x), int(gl_FragCoord.y), snap-1), 1.0);
    } else if (deviation >= 1-eps) {
        tryWrite(getVecFromView(int(gl_FragCoord.x), int(gl_FragCoord.y), snap+1), 0.0);
    }
    tryWrite(getVecFromView(int(gl_FragCoord.x), int(gl_FragCoord.y), snap), deviation);
}

