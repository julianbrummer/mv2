#include "conturing.h"

bool isPowerOfTwo (uint x) {
  return ((x != 0) && !(x & (x - 1)));
}

uint nextPowerOf2(uint n) {
    unsigned int p = 1;
    if (n && !(n & (n - 1)))
        return n;

    while (p < n) {
        p <<= 1;
    }
    return p;
}

uint8_t i_log2(unsigned int n) {
    uint8_t result = 0;
    while (n >>= 1)
        ++result;
    return result;
}


uint pow2(uint e) {
    if (e < sizeof(pow2Table)/sizeof(uint)) {
        return pow2Table[e];
    }
    uint result = 1;
    for (uint i = 0; i < e; ++i) {
        result *= 2;
    }
    return result;
}

double cellSize(uint level) {
    if (level < sizeof(cellSizeTable)) {
        return (cellSizeTable)[level];
    }
    return 1.0/pow2(level);
}



Index operator *(const Index& index, uint s) {
    return Index(s*index.x, s*index.y, s*index.z);
}
Index operator *(uint s, const Index& index) {
    return Index(s*index.x, s*index.y, s*index.z);
}

Index Index::operator /(uint d) const{
    return Index(x/d, y/d, z/d);
}

Index Index::operator +(const Index& index) const {
    return Index(x+index.x, y+index.y, z+index.z);
}

Index Index::operator -(const Index& index) const {
    return Index(x-index.x, y-index.y, z-index.z);
}

inline bool Index::operator <(uint xyz) const {
    return x < xyz && y < xyz && z < xyz;
}

inline bool Index::operator >(uint xyz) const {
    return x > xyz && y > xyz && z > xyz;
}

uint& Index::operator [](uint orientation) {
    return orientation == 0? x : (orientation == 1? y : z);
}

Index Index::shiftX(int shift) const {
    return Index(x+shift,y,z);
}

Index Index::shiftY(int shift) const {
    return Index(x,y+shift,z);
}

Index Index::shiftZ(int shift) const {
    return Index(x,y,z+shift);
}

GridSampler::GridSampler(uint res) : res(res), size(res+1) {
    leaf_level = i_log2(res);
}

inline uint GridSampler::nodeSize(uint level) const {
    return 1 << (leaf_level - level);
}

inline bool SignSampler::sign(const Index& index) const {
    return sign(index.x, index.y, index.z);
}

uint8_t SignSampler::signConfig(const Index &node_origin) const {
    uint8_t config = 0;
    uint max_pos = size-1;
    if (node_origin < max_pos) {
        for (int i = 0; i < 8; ++i) {
            if (sign(node_origin+corner_delta[i]))
                config |= 1 << i;
        }
    }
    return config;
}


void SignSampler::inside(float voxelGridRadius, aligned_vector3f &positions) {
    float o = -voxelGridRadius+voxelGridRadius/size;
    Vector3f origin(o,o,o);
    float cellSize = 2*voxelGridRadius/size;
    for (uint x = 0; x < size; ++x) {
        for (uint y = 0; y < size; ++y) {
            for (uint z = 0; z < size; ++z) {
                if (sign(x,y,z)) {
                    positions.push_back(origin+cellSize*Vector3f(x,y,z));
                }
            }
        }
    }
}

bool HermiteDataSampler::hasCut(const Index &cellOrigin) const {
    for (int i = 0; i < 12; ++i) {
        if (hasCut(edge_orientation[i], cellOrigin + corner_delta[edge_corners[i][0]]))
            return true;
    }
    return false;
}

bool HermiteDataSampler::hasCut(uint orientation, const Index &from) const {
    return hasFrontCut(orientation, from) || hasBackCut(orientation, from);
}

void HermiteDataSampler::edgeIntersections(float voxelGridRadius, aligned_vector3f& positions, aligned_vector3f& colors) {
    float o = -voxelGridRadius+voxelGridRadius/257;
    float cellSize = 2*voxelGridRadius/257;
    Vector3f origin(o,o,o);
    float d = 0.0;
    Vector3f n(0,0,0);
    for (uint x = 0; x < res; ++x) {
        for (uint y = 0; y <= res; ++y) {
            for (uint z = 0; z <= res; ++z) {
                if (frontEdgeInfo(0,Index(x,y,z),d,n)) {
                    Vector3f v = Vector3f(x+d,y,z);
                    positions.push_back(origin+cellSize*v);
                    colors.push_back(n);
                }
                if (backEdgeInfo(0,Index(x,y,z),d,n)) {
                    Vector3f v = Vector3f(x+d,y,z);
                    positions.push_back(origin+cellSize*v);
                    colors.push_back(n);
                }
            }
        }
    }
    for (uint x = 0; x <= res; ++x) {
        for (uint y = 0; y < res; ++y) {
            for (uint z = 0; z <= res; ++z) {
                if (frontEdgeInfo(1,Index(x,y,z),d,n)) {
                    Vector3f v = Vector3f(x,y+d,z);
                    positions.push_back(origin+cellSize*v);
                    colors.push_back(n);
                }
                if (backEdgeInfo(1,Index(x,y,z),d,n)) {
                    Vector3f v = Vector3f(x,y+d,z);
                    positions.push_back(origin+cellSize*v);
                    colors.push_back(n);
                }
            }
        }
    }
    for (uint x = 0; x <= res; ++x) {
        for (uint y = 0; y <= res; ++y) {
            for (uint z = 0; z < res; ++z) {
                if (frontEdgeInfo(2,Index(x,y,z),d,n)) {
                    Vector3f v = Vector3f(x,y,z+d);
                    positions.push_back(origin+cellSize*v);
                    colors.push_back(n);
                }
                if (backEdgeInfo(2,Index(x,y,z),d,n)) {
                    Vector3f v = Vector3f(x,y,z+d);
                    positions.push_back(origin+cellSize*v);
                    colors.push_back(n);
                }
            }
        }
    }
}
