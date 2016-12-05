#ifndef SCAN_H
#define SCAN_H
#include <QOpenGLFunctions_4_3_Core>
#include <QGLShaderProgram>
#include <queue>
#include <map>
#include <iostream>
#include "conturing.h"
#include "bufferobject.h"
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


struct HermiteIntersectionData {
    map<Intersection, HermiteData> intersections;
    uint res;

    HermiteIntersectionData(uint res) : res(res) {}
};

struct CompressedEdgeData {
    vector_2_uint frontface_cuts, backface_cuts;
    uint res, depth, size;

    CompressedEdgeData(uint res);
    inline bool frontface_cut(uint dir, const Index &index) const;
    inline bool backface_cut(uint dir, const Index &index) const;
    inline bool frontface_cut(uint dir, uint x, uint y, uint z) const;
    inline bool backface_cut(uint dir,  uint x, uint y, uint z) const;
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



class RenderStrategy {
public:
    virtual bool initShaders(QGLShaderProgram& programEdgeScan, QGLShaderProgram& programHermiteScan) const = 0;
    virtual void render(QGLShaderProgram& program, const QMatrix4x4& projection, const QMatrix4x4& view) = 0;
    virtual ~RenderStrategy() {}
};

class ScanTarget {
public:
    virtual void startScan() {}
    virtual void endScan() {}
    virtual void startScan(Direction dir) {}
    virtual void endScan(Direction dir) {}
    virtual ~ScanTarget() {}
};

class Scanner :  protected QOpenGLFunctions_4_3_Core  {
protected:
    ScanTarget* target;
    uint res;
    virtual void transferData() {}
    virtual void transferData(Direction dir) {}
    virtual void configureProgram() {}
    virtual void configureProgram(Direction dir) {}
private:
    GLuint fbo;
    void scan(RenderStrategy* scene, Direction dir);
public:

    QGLShaderProgram* program;
    QMatrix4x4 projection;
    void scan(RenderStrategy* scene);
    Scanner(ScanTarget* target, uint res);
    virtual ~Scanner();
};

class TextureTarget :  public ScanTarget, QOpenGLFunctions_4_3_Core  {
protected:
    vector<GLuint> textures;
    uint slices;
    uint texRes;
    uint texCount;
private:
    void bindTextures();
    void resetTextures();
    void unbindImages();
public:
    void startScan(Direction dir) override;
    void endScan(Direction dir) override;
    TextureTarget(uint texRes, uint slices, int texCount);
    virtual ~TextureTarget();
};

class SSBOTarget : public ScanTarget, QOpenGLFunctions_4_3_Core {
public:
    unique_ptr<SSBO> buffer;
    SSBOTarget(set<Intersection>& intersections);
};

class CompressedEdgeScanner : public Scanner {
public:
    CompressedEdgeData* data;
    CompressedEdgeScanner(uint res)
        : Scanner(new TextureTarget(res-1, res/32, 2), res), data(new CompressedEdgeData(res)) {}
    void transferData(Direction dir) override;
    void configureProgram() override;
    void configureProgram(Direction dir) override;
};

class HermiteScanner :  public Scanner {
private:
    uint intersectCount;
    HermiteData decode(uint code) const;
public:
    HermiteIntersectionData* data;
    HermiteScanner(uint res, set<Intersection>& intersections)
        : Scanner(new SSBOTarget(intersections), res),
          intersectCount(intersections.size()), data(new HermiteIntersectionData(res)) {}
    void transferData() override;
    void configureProgram(Direction dir) override;
};

class CompressedSignSampler : public SignSampler {
private:

    CompressedEdgeData* edgeData;
    inline void stepForward(uint dir, const Index& edge, const Index &to, queue<Index>& indices);
    inline void stepBackward(uint dir, const Index& edge, const Index &to, queue<Index>& indices);
    void floodFill();
    void floodFill1(CompressedSignData* data);
public:
    CompressedSignData* data;
    CompressedSignSampler(CompressedEdgeData* data);

    bool sign(uint x, uint y, uint z) const override;
    bool frontface_cut(uint dir, const Index &index) const override;
    bool backface_cut(uint dir, const Index &index) const override;
    virtual ~CompressedSignSampler();
};

class SparseHermiteSampler : public HermiteDataSampler {

public:
    HermiteIntersectionData* data;

    SparseHermiteSampler(HermiteIntersectionData* data) :  HermiteDataSampler(data->res), data(data) {}
    HermiteData* edgeInfo(Direction dir, Orientation orientation, const Index& edge) const override;
    bool hasCut(Direction dir, Orientation orientation, const Index& edge) const override;
    virtual ~SparseHermiteSampler();
};

#endif // SCAN_H
