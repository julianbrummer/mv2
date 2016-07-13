#ifndef SCAN_H
#define SCAN_H
#include <QOpenGLFunctions_4_3_Core>
#include <QGLShaderProgram>
#include <queue>
#include <iostream>
#include "conturing.h"
using namespace std;



struct CompressedHermiteData {
    vector_2_uint frontface_cuts, backface_cuts;
    uint res, size;

    static bool unpack(uint data, float &d);
    static bool unpack(uint data, float &d, Vector3f& n);

    CompressedHermiteData(uint res);
    inline uint frontface_cut(uint orientation, const Index& index) const;
    inline uint backface_cut(uint orientation, const Index& index) const;
};

struct CompressedEdgeData {
    vector_2_uint cuts;
    uint res, depth, size;

    CompressedEdgeData(uint res);
    inline bool cut(uint orientation, const Index& index) const;
    inline bool cut(uint orientation, uint x, uint y, uint z) const;
};

struct CompressedSignData {
    vector<uint> data;
    uint res,size,depth;

    CompressedSignData(uint res, bool defaultSign = true);
    inline bool sign(const Index& index) const;
    inline bool sign(uint x, uint y, uint z) const;  
    void setInside(const Index& index);
    void setInside(uint x, uint y, uint z);
    void setOutside(const Index& index);
    void setOutside(uint x, uint y, uint z);
};

enum ScannerMode {
    // scans FRONT_FACES_X, FRONT_FACES_Y, FRONT_FACES_Z,
    // BACK_FACES_X, BACK_FACES_Y, BACK_FACES_Z
    FRONT_XYZ_BACK_XYZ = 1,

    // scans FRONT_AND_BACK_FACES_X, FRONT_AND_BACK_FACES_Y, FRONT_AND_BACK_FACES_Z
    FRONT_AND_BACK_XYZ = 2,

};

enum Direction {
    X,Y,Z
};


class Renderable {
public:
    virtual void render(QGLShaderProgram& program, const QMatrix4x4& projection, const QMatrix4x4& view) const = 0;
};

class Scanner :  protected QOpenGLFunctions_4_3_Core  {
protected:
    vector<GLuint> textures;
    uint slices;
    uint texRes;
    virtual void transferData(Direction dir) = 0;
    virtual void configureProgram(Direction dir) {}
private:
    GLuint fbo;
    void bindTextures();
    void resetTextures();
    void unbindImages();
public:

    QGLShaderProgram* program;
    QMatrix4x4 projection;
    void scan(const Renderable* scene, Direction dir);
    Scanner(uint texRes, uint slices, int texCount);
    virtual ~Scanner();
};

class CompressedEdgeScanner : public Scanner {
public:
    shared_ptr<CompressedEdgeData> data;
    CompressedEdgeScanner(uint res)
        : Scanner(res+1, res/32 + 1, 1), data(new CompressedEdgeData(res)) {}
    void transferData(Direction dir) override;
    void configureProgram(Direction dir) override;
};

class CompressedHermiteScanner :  public Scanner {

public:
    shared_ptr<CompressedHermiteData> data;
    CompressedHermiteScanner(uint res)
        : Scanner(res+1, res+1, 2), data(new CompressedHermiteData(res)) {}
    void transferData(Direction dir) override;
    void configureProgram(Direction dir) override;
};

class CompressedSignSampler : public SignSampler {
private:

    CompressedEdgeData* edgeData;
    inline void step(uint orientation, const Index& edge, const Index &to, queue<Index>& indices);
    void floodFill();
public:
    shared_ptr<CompressedSignData> data;
    CompressedSignSampler(CompressedEdgeData* data);

    bool sign(uint x, uint y, uint z) const override;
};

class CompressedHermiteSampler : public HermiteDataSampler {
private:
    Index origin;
public:
    shared_ptr<CompressedHermiteData> data;

    CompressedHermiteSampler(shared_ptr<CompressedHermiteData> data, const Index& origin = Index(0)) : data(data), origin(origin), HermiteDataSampler(data->res) {}
    bool frontEdgeInfo(uint orientation, const Index& from, float& d, Vector3f& n) const override;
    bool backEdgeInfo(uint orientation, const Index& from, float& d, Vector3f& n) const override;
    bool hasFrontCut(uint orientation, const Index &from) const override;
    bool hasBackCut(uint orientation, const Index &from) const override;

};

#endif // SCAN_H
