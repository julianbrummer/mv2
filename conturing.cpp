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
    return Index(s*index.x(), s*index.y(), s*index.z());
}
Index operator *(uint s, const Index& index) {
    return Index(s*index.x(), s*index.y(), s*index.z());
}

std::ostream& operator <<(std::ostream& os, const Index &obj) {
    os << obj.x() << " " << obj.y() << " " << obj.z();
    return os;
}

Index Index::operator /(uint d) const{
    return Index(v[0]/d, v[1]/d, v[2]/d);
}

Index Index::operator +(const Index& index) const {
    return Index(v[0]+index.v[0], v[1]+index.v[1], v[2]+index.v[2]);
}

Index Index::operator -(const Index& index) const {
    return Index(v[0]-index.v[0], v[1]-index.v[1], v[2]-index.v[2]);
}

inline bool Index::operator <(uint xyz) const {
    return v[0] < xyz && v[1] < xyz && v[2] < xyz;
}

inline bool Index::operator >(uint xyz) const {
    return v[0] > xyz && v[1] > xyz && v[2] > xyz;
}

bool Index::operator == (const Index& rhs) const{
    return v[0] == rhs.v[0] && v[1] == rhs.v[1] && v[2] == rhs.v[2];
}

bool Index::operator != (const Index& rhs) const{
    return !(*this == rhs);
}

uint& Index::operator [](uint dir) {
    return v[dir];
}

Index Index::shiftX(int shift) const {
    return Index(v[0]+shift,v[1],v[2]);
}

Index Index::shiftY(int shift) const {
    return Index(v[0],v[1]+shift,v[2]);
}

Index Index::shiftZ(int shift) const {
    return Index(v[0],v[1],v[2]+shift);
}

Index Index::shift(uint dir, int shift) const {
    Index i(v[0],v[1],v[2]);
    i.v[dir] += shift;
    return i;
}

bool Intersection::operator == (const Intersection& rhs) const{
    return status == rhs.status && edge == rhs.edge;
}

bool Intersection::operator != (const Intersection& rhs) const{
    return !(*this == rhs);
}

Direction Intersection::dir() {
    return Direction(abs(status)-1);
}

Orientation Intersection::orientation() {
    return status > 0? FRONT : BACK;
}

void Intersection::setDir(Direction dir) {
    status = orientation() == FRONT? dir+1 : -(dir+1);
}

void Intersection::setOrientation(Orientation orientation) {
    status = orientation * abs(status);
}

std::ostream& operator <<(std::ostream& os, const Intersection &obj) {
    os << obj.status << " " << obj.edge;
    return os;
}


GridSampler::GridSampler(uint res) : res(res), size(res+1) {
    leaf_level = i_log2(res);
}

inline uint GridSampler::nodeSize(uint level) const {
    return 1 << (leaf_level - level);
}

inline bool SignSampler::sign(const Index& index) const {
    return sign(index.x(), index.y(), index.z());
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

bool SignSampler::contributes(const Index &cell_origin) const {
    return contributingCells.count(cell_origin) > 0;
}


void SignSampler::inside(float voxelGridRadius, aligned_vector3f &positions) {
    float o = -voxelGridRadius;
    Vector3f origin(o,o,o);
    float cellSize = 2*voxelGridRadius/res;
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

bool HermiteDataSampler::edgeInfo(Direction dir, Orientation orientation, const Index& edge, float& d, Vector3f& n) const {
    HermiteData* data = edgeInfo(dir, orientation, edge);
    if (!data)
        return false;

    n = data->n;
    d = data->d;
    return true;
}

bool HermiteDataSampler::hasCut(const Index &cellOrigin) const {
    for (int i = 0; i < 12; ++i) {
        if (hasCut(edge_dir[i], cellOrigin + corner_delta[edge_corners[i][0]]))
            return true;
    }
    return false;
}

bool HermiteDataSampler::hasCut(Direction dir, const Index &edge) const {
    return hasCut(dir, FRONT, edge) || hasCut(dir, BACK, edge);
}

void HermiteDataSampler::edgeIntersections(float voxelGridRadius, aligned_vector3f& positions, aligned_vector3f& colors) {
    float cellSize = 2*voxelGridRadius/res;
    float o = -voxelGridRadius;
    Vector3f origin(o,o,o);
    float d = 0.0;
    Vector3f n(0,0,0);
    for (uint x = 0; x < res; ++x) {
        for (uint y = 0; y <= res; ++y) {
            for (uint z = 0; z <= res; ++z) {
                if (edgeInfo(X, FRONT, Index(x,y,z),d,n)) {
                    Vector3f v = Vector3f(x+d,y,z);
                    positions.push_back(origin+cellSize*v);
                    colors.push_back(n);
                }
                if (edgeInfo(X, BACK, Index(x,y,z),d,n)) {
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
                if (edgeInfo(Y, FRONT, Index(x,y,z),d,n)) {
                    Vector3f v = Vector3f(x,y+d,z);
                    positions.push_back(origin+cellSize*v);
                    colors.push_back(n);
                }
                if (edgeInfo(Y, BACK, Index(x,y,z),d,n)) {
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
                if (edgeInfo(Z, FRONT, Index(x,y,z),d,n)) {
                    Vector3f v = Vector3f(x,y,z+d);
                    positions.push_back(origin+cellSize*v);
                    colors.push_back(n);
                }
                if (edgeInfo(Z, BACK, Index(x,y,z),d,n)) {
                    Vector3f v = Vector3f(x,y,z+d);
                    positions.push_back(origin+cellSize*v);
                    colors.push_back(n);
                }
            }
        }
    }
}
