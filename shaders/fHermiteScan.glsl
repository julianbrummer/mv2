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
    vec3 n;
    float d;
};

layout(std140, binding = 3) buffer data {
    Intersection cut[];
};

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
/*
int contributes(ivec4 e) {
    for(int i = 0; i < cut.length(); i++) {
        if (all(equal(e, cut[i].e))) {
                return i;
        }
    }
    return -1;
}
*/
layout(index=0) subroutine (vecFromView) ivec4 vecFromXView(int x, int y, int z) {
        return ivec4(z,y,x, gl_FrontFacing? 1 : -1);
}

layout(index=1) subroutine (vecFromView) ivec4 vecFromYView(int x, int y, int z) {
        return ivec4(x,z,y,gl_FrontFacing? 2 : -2);
}

layout(index=2) subroutine (vecFromView) ivec4 vecFromZView(int x, int y, int z) {
        return ivec4(res-x,y,z,gl_FrontFacing? 3 : -3);
}

void tryWrite(ivec4 edge, float deviation) {
    int index = contributes(edge);
    if (index >= 0) {
        if (cut[index].d < 0.0 || (gl_FrontFacing && cut[index].d > deviation) || (!gl_FrontFacing && cut[index].d < deviation)) {
            cut[index].d = deviation;
            cut[index].n = normalize(n);
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

