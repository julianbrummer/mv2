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
    if (e < sizeof(pow2Table)) {
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

bool OctreeNode::sign(uint i) {
    return signConfig & (1 << i);
}

bool OctreeNode::signChange(uint e) {
    return sign(edge_corners[e][0]) != sign(edge_corners[e][1]);
}

const uint Sampler::MIN_RESOULTION = 2;

Sampler::Sampler(uint res) {
    this->res = std::max(nextPowerOf2(res), MIN_RESOULTION);
    leaf_level = i_log2(this->res);
    size = this->res + 1;
    signs = vector<vector<vector<bool>>>(size, vector<vector<bool>>(size, vector<bool>(size, true)));
}

void Sampler::setSign(const Index &index, bool b) {
    if (index < size)
        signs[index.x][index.y][index.z] = b;
}

inline bool Sampler::sign(uint x, uint y, uint z) const {
    return x < size && y < size && z < size && signs[x][y][z];
}

inline bool Sampler::sign(const Index& index) const {
    return sign(index.x, index.y, index.z);
}

inline uint Sampler::nodeSize(uint level) const {
    return 1 << (leaf_level - level);
}

void Sampler::applySigns(OctreeNode &node, const Index &node_origin) const {
    uint node_size = nodeSize(node.level);
    uint max_pos = size-node_size;
    array<bool,8> signs;
    if (node_origin < max_pos) {
        signs[0] = sign(node_origin);
        signs[1] = sign(node_origin.shiftX(node_size));
        signs[2] = sign(node_origin+Index(node_size, 0, node_size));
        signs[3] = sign(node_origin.shiftZ(node_size));
        signs[4] = sign(node_origin.shiftY(node_size));
        signs[5] = sign(node_origin+Index(node_size, node_size, 0));
        signs[6] = sign(node_origin+Index(node_size, node_size, node_size));
        signs[7] = sign(node_origin+Index(0, node_size, node_size));
        for (int i = 0; i < 8; ++i) {
            if (signs[i])
                node.signConfig |= 1 << i;
        }
    }
}


void Sampler::inside(float voxelGridRadius, aligned_vector3f &positions) {
    float o = -voxelGridRadius+voxelGridRadius/size;
    Vector3f origin(o,o,o);
    float cellSize = 2*voxelGridRadius/size;
    for (uint x = 0; x < size; ++x) {
        for (uint y = 0; y < size; ++y) {
            for (uint z = 0; z < size; ++z) {
                if (signs[x][y][z])
                    positions.push_back(origin+cellSize*Vector3f(x,y,z));
            }
        }
    }
}

void Sampler::outside(float voxelGridRadius, aligned_vector3f &positions) {
    float o = -voxelGridRadius+voxelGridRadius/size;
    Vector3f origin(o,o,o);
    float cellSize = 2*voxelGridRadius/size;
    for (uint x = 0; x < size; ++x) {
        for (uint y = 0; y < size; ++y) {
            for (uint z = 0; z < size; ++z) {
                if (!signs[x][y][z])
                    positions.push_back(origin+cellSize*Vector3f(x,y,z));
            }
        }
    }
}

void HermiteDataSampler::edgeIntersections(float voxelGridRadius, aligned_vector3f& positions, aligned_vector3f& colors) {
    float o = -voxelGridRadius+voxelGridRadius/size;
    Vector3f origin(o,o,o);
    float cellSize = 2*voxelGridRadius/size;
    float d = 0.0;
    Vector3f n(0,0,0);
    for (uint x = 0; x < res; ++x) {
        for (uint y = 0; y < size; ++y) {
            for (uint z = 0; z < size; ++z) {
                if (frontEdgeInfo(0,Index(x,y,z),d,n)) {
                    positions.push_back(origin+cellSize*Vector3f(x+d,y,z));
                    colors.push_back(n);
                }
                if (backEdgeInfo(0,Index(x,y,z),d,n)) {
                    positions.push_back(origin+cellSize*Vector3f(x+d,y,z));
                    colors.push_back(n);
                }
            }
        }
    }
    for (uint x = 0; x < size; ++x) {
        for (uint y = 0; y < res; ++y) {
            for (uint z = 0; z < size; ++z) {
                if (frontEdgeInfo(1,Index(x,y,z),d,n)) {
                    positions.push_back(origin+cellSize*Vector3f(x,y+d,z));
                    colors.push_back(n);
                }
                if (backEdgeInfo(1,Index(x,y,z),d,n)) {
                    positions.push_back(origin+cellSize*Vector3f(x,y+d,z));
                    colors.push_back(n);
                }
            }
        }
    }
    for (uint x = 0; x < size; ++x) {
        for (uint y = 0; y < size; ++y) {
            for (uint z = 0; z < res; ++z) {
                if (frontEdgeInfo(2,Index(x,y,z),d,n)) {
                    positions.push_back(origin+cellSize*Vector3f(x,y,z+d));
                    colors.push_back(n);
                }
                if (backEdgeInfo(2,Index(x,y,z),d,n)) {
                    positions.push_back(origin+cellSize*Vector3f(x,y,z+d));
                    colors.push_back(n);
                }
            }
        }
    }
}
