#ifndef CONTURING_H
#define CONTURING_H

#include <vector>
#include <Eigen/Core>
#include <array>
#include <memory>
#include "def.h"

using namespace std;
using namespace Eigen;



typedef unsigned int uint;

struct Index {
    unsigned int x, y, z;
    Index(uint x, uint y, uint z) : x(x), y(y), z(z) {}
    Index(uint xyz) : x(xyz), y(xyz), z(xyz) {}
    Index() : Index(0) {}
    Index operator +(const Index& index) const;
    Index operator /(uint d) const;
    friend Index operator *(const Index& index, uint s);
    friend Index operator *(uint s, const Index& index);
    inline bool operator <(uint xyz) const;
    inline bool operator >(uint xyz) const;
    uint& operator [](uint orientation);
    Index shiftX(int shift) const;
    Index shiftY(int shift) const;
    Index shiftZ(int shift) const;
};

// shift of child origin from parent origin
const Index child_origin[8] = {
    Index(0,0,0), Index(1,0,0), Index(1,1,0), Index(0,1,0),
    Index(0,0,1), Index(1,0,1), Index(1,1,1), Index(0,1,1)
};

// shift of cell corner from cell origin
const Index corner_delta[8] = {
    Index(0,0,0), Index(1,0,0), Index(1,0,1), Index(0,0,1),
    Index(0,1,0), Index(1,1,0), Index(1,1,1), Index(0,1,1)
};


// edge defined by 2 corners of the cube
const uint edge_corners[12][2] = {
    {0,1},{1,2},{3,2},{0,3},
    {4,5},{5,6},{7,6},{4,7},
    {0,4},{1,5},{2,6},{3,7}
};

// power_of_2
const uint pow2Table[12] = {
    1,2,4,8,16,32,64,128,256,512,1024,2048
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

class OctreeNode {
public:
    uint8_t level;
    uint8_t signConfig;

    OctreeNode(uint8_t level) : level(level), signConfig(0) {}
    bool sign(uint i);
    bool signChange(uint e);
    bool frontface(uint e);
    bool backface(uint e);
};

class Sampler {
public:
    static const uint MIN_RESOULTION;
    uint res;
    uint8_t leaf_level;
    uint size; // res+1

    Sampler(uint res);
    bool sign(uint x, uint y, uint z) const;
    bool sign(const Index& index) const;
    void applySigns(OctreeNode& node,  const Index& node_origin) const;
    uint nodeSize(uint level) const;

    void inside(float voxelGridRadius, aligned_vector3f &positions);
    void outside(float voxelGridRadius, aligned_vector3f &positions);

protected:


    vector<vector<vector<bool>>> signs;

    void setSign(const Index& index, bool b);
};

class HermiteDataSampler : public Sampler {
public:
    HermiteDataSampler(uint res) : Sampler(res) {}

    virtual bool frontEdgeInfo(uint orientation, const Index& from, float& d, Vector3f& n) const = 0;
    virtual bool backEdgeInfo(uint orientation, const Index& from, float& d, Vector3f& n) const = 0;
    virtual bool intersectsEdge(uint orientation, const Index& from, float& d) const = 0;
    virtual bool intersectsEdge(uint orientation, const Index& from, float& d, Vector3f& n) const = 0;

    void edgeIntersections(float voxelGridRadius, aligned_vector3f &positions, aligned_vector3f &colors);
};


#endif // CONTURING_H
