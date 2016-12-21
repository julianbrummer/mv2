#ifndef CONTURING_H
#define CONTURING_H

#include <vector>
#include <Eigen/Core>
#include <array>
#include <set>
#include <memory>
#include <iostream>
#include "def.h"

using namespace std;
using namespace Eigen;

typedef unsigned int uint;

enum Direction {
    X,Y,Z
};

enum Orientation {
    BACK = - 1, FRONT = 1
};

struct Index {
    uint v[3];
    Index(uint x, uint y, uint z) {v[0] = x; v[1] = y; v[2] = z;}
    Index(uint xyz) : Index(xyz, xyz, xyz) {}
    Index() : Index(0) {}
    Index operator +(const Index& index) const;
    Index operator -(const Index& index) const;
    Index operator /(uint d) const;
    friend Index operator *(const Index& index, uint s);
    friend Index operator *(uint s, const Index& index);
    friend std::ostream& operator <<(std::ostream& os, const Index &obj);
    inline bool operator <(uint xyz) const;
    inline bool operator >(uint xyz) const;
    bool operator == (const Index& rhs) const;
    bool operator != (const Index& rhs) const;
    uint& operator [](uint dir);
    Index shiftX(int shift) const;
    Index shiftY(int shift) const;
    Index shiftZ(int shift) const;
    Index shift(uint dir, int shift) const;
    uint x() const {return v[0];}
    uint y() const {return v[1];}
    uint z() const {return v[2];}
};

inline bool operator <(const Index& lhs, const Index& rhs)
{
    for (int i = 2; i >= 0; --i) {
        if (lhs.v[i] < rhs.v[i])
            return true;
        if (lhs.v[i] > rhs.v[i])
            return false;
    }
    return false;
}

struct Intersection {
    Index edge;
    int status; // -3, -2, -1, 1, 2, 3 (-z, -y, -x, x, y, z)
    Intersection() : edge(0), status(1) {}
    Intersection(const Index& edge, Direction dir, Orientation orientation)
        : edge(edge) {
        setDir(dir);
        setOrientation(orientation);
    }

    friend std::ostream& operator <<(std::ostream& os, const Intersection& obj);
    bool operator == (const Intersection& rhs) const;
    bool operator != (const Intersection& rhs) const;
    Direction dir();
    Orientation orientation();
    void setDir(Direction dir);
    void setOrientation(Orientation orientation);
};


inline bool operator <(const Intersection& lhs, const Intersection& rhs)
{
    if (lhs.status < rhs.status)
        return true;
    if (lhs.status > rhs.status)
        return false;
    return lhs.edge < rhs.edge;
}

struct HermiteData {
    Vector3f n;
    float d;
    HermiteData() : n(0,0,0), d(0.0f) {}
    HermiteData(Vector3f& n, float d) : n(n), d(d) {}
};

struct HermiteEdgeData {
    Vector3f n;
    Vector3f p;
    uint e;
    Orientation orientation;
    HermiteEdgeData() : n(0,0,0), p(0,0,0), e(0), orientation(FRONT) {}
    HermiteEdgeData(Vector3f& n, Vector3f p, uint e, Orientation orientation) : n(n), p(p), e(e), orientation(orientation) {}
};

// shift of child cell origin from parent cell origin
const Index child_origin[8] = {
    Index(0,0,0), Index(1,0,0), Index(1,1,0), Index(0,1,0),
    Index(0,0,1), Index(1,0,1), Index(1,1,1), Index(0,1,1)
};

// shift of innder face origin from parent cell origin
const Index cell_face_origin[3][4] = {
    {Index(1,0,0), Index(1,0,1), Index(1,1,1), Index(1,1,0)},
    {Index(0,1,0), Index(1,1,0), Index(1,1,1), Index(0,1,1)},
    {Index(0,0,1), Index(0,1,1), Index(1,1,1), Index(1,0,1)}
};

// shift of inner edge origin from parent cell origin
const Index cell_edge_origin[3][2] = {
    {Index(0,1,1), Index(1,1,1)},
    {Index(1,0,1), Index(1,1,1)},
    {Index(1,1,0), Index(1,1,1)}
};

// shift of inner face origin from parent face origin
const Index face_face_origin[3][4] = {
    {Index(0,0,0), Index(0,0,1), Index(0,1,1), Index(0,1,0)},
    {Index(0,0,0), Index(1,0,0), Index(1,0,1), Index(0,0,1)},
    {Index(0,0,0), Index(0,1,0), Index(1,1,0), Index(1,0,0)}
};

// shift of inner edge origin from parent face origin
const Index face_edge_origin[3][4] = {
    {Index(0,0,1), Index(0,1,1), Index(0,1,0), Index(0,1,1)},
    {Index(0,0,1), Index(1,0,1), Index(1,0,0), Index(1,0,1)},
    {Index(0,1,0), Index(1,1,0), Index(1,0,0), Index(1,1,0)}
};

// shift of cell corner from cell origin
const Index corner_delta[8] = {
    Index(0,0,0), Index(1,0,0), Index(1,0,1), Index(0,0,1),
    Index(0,1,0), Index(1,1,0), Index(1,1,1), Index(0,1,1)
};

const Index cells_contain_edge_delta[3][4] = {
    {Index(0,0,1), Index(0,1,1), Index(0,1,0), Index(0,0,0)},
    {Index(1,0,0), Index(1,0,1), Index(0,0,1), Index(0,0,0)},
    {Index(0,0,0), Index(0,1,0), Index(1,1,0), Index(1,0,0)}
};


// edge defined by 2 corners of the cube
const uint edge_corners[12][2] = {
    {0,1},{1,2},{3,2},{0,3},
    {4,5},{5,6},{7,6},{4,7},
    {0,4},{1,5},{2,6},{3,7}
};

// edges connected to corner [corner][3 edges]
const uint corner_edges[8][3] = {
    {0,8,3},
    {0,9,1},
    {2,10,1},
    {2,11,3},
    {4,8,7},
    {4,9,5},
    {6,10,5},
    {6,11,7}
};

// edge directions of a cell (x, y, z)
const Direction edge_dir[12] = {
    X,Z,X,Z,X,Z,X,Z,Y,Y,Y,Y
};

// power_of_2
const uint pow2Table[32] = {1,2,4,8,16,32,64,128,
                            256,512,1024,2048,4096,8192,16384,32768,
                            65536,131072,262144,524288,1048576,2097152,4194304,8388608,
                            16777216,33554432,67108864,134217728,268435456,536870912,1073741824,2147483648
};

// cell size per level
const double cellSizeTable[12] = {
    1,0.5,0.25,0.125,
    0.0625,0.03125,0.015625,0.0078125,
    0.00390625,0.001953125,0.0009765625,0.00048828125
};


/*
               *--------**--------*
              /|       /|        /|
             / |      / |       / |
            /  |     /  |      /  |
           *--------**--------*   |
          /|   |  3/|   |   2/|   |
         / |   *--/-|---**--/-|---*
        /  |  /| /  |  /|  /  |  /|
       *--------**--------*   | / |
       |   |/  ||   |/  | |   |/  |
       |   *----|---**----|---*   |
       |  /| 7 ||  /|  6| |  /|   |
       | / |   *|-/-|---**|-/-|---*
       |/  |  / |/0 |  /  |/1 |  /
       *--------**--------*   | /
       |   |/   |   |/    |   |/
       |   *----|---**----|---*
       |  /  4  |  /   5  |  /
       | /      | /       | /
       |/       |/        |/
       *--------**--------*

       *---4----*
      /|       /|
     7 |      5 |
    /  8     /  9
   *----6---*   |
   |   |    |   |
   |   O---0|---*
   11 /     10 /
   | 3      | 1
   |/       |/
   *---2----*


       4--------5
      /|       /|
     / |      / |
    /  |     /  |
   7--------6   |
   |   |    |   |
   |   0----|---1
   |  /     |  /
   | /      | /
   |/       |/
   3--------2

*/
// some util methods
bool isPowerOfTwo(uint x);
uint nextPowerOf2(uint n);
uint8_t i_log2(unsigned int n);
double cellSize(uint level);
uint pow2(uint e);


class GridSampler {
public:
    uint res;
    uint8_t leaf_level;
    uint size; // res+1

    GridSampler(uint res);
    uint nodeSize(uint level) const;
};

class SignSampler : public GridSampler {
public:
    set<Intersection> contributingIntersections;
    set<Index> contributingCells;
    SignSampler(uint res) : GridSampler(res) {}

    virtual bool sign(uint x, uint y, uint z) const = 0;
    virtual bool frontface_cut(uint dir, const Index &index) const = 0;
    virtual bool backface_cut(uint dir, const Index &index) const = 0;
    bool sign(const Index& index) const;
    uint8_t signConfig(const Index& node_origin) const;
    bool contributes(const Index& cell_origin) const;

    void inside(float voxelGridRadius, aligned_vector3f &positions);
    void outside(float voxelGridRadius, aligned_vector3f &positions);
    virtual ~SignSampler() {}
};

class HermiteDataSampler : public GridSampler  {

public:
    HermiteDataSampler(uint res) : GridSampler(res) {}

    virtual HermiteData* edgeInfo(Direction dir, Orientation orientation, const Index& edge) const = 0;
    virtual bool hasCut(Direction dir, Orientation orientation, const Index& edge) const = 0;

    bool edgeInfo(Direction dir, Orientation orientation, const Index& edge, float& d, Vector3f& n) const;
    bool hasCut(const Index &cellOrigin) const;
    bool hasCut(Direction dir, const Index &edge) const;
    void edgeIntersections(float voxelGridRadius, aligned_vector3f &positions, aligned_vector3f &colors);

    virtual ~HermiteDataSampler() = default;
};


#endif // CONTURING_H
