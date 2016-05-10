#ifndef SCAN_H
#define SCAN_H
#include <QOpenGLFunctions_4_2_Core>
#include <QGLShaderProgram>
#include <queue>
#include "conturing.h"
using namespace std;



struct CompressedHermiteData {
    vector_4_uint frontface_cuts, backface_cuts;
    uint res;

    static bool unpack(uint data, float &d);
    static bool unpack(uint data, float &d, Vector3f& n);

    CompressedHermiteData(uint res);
    inline uint frontface_cut(uint orientation, const Index& index) const;
    inline uint backface_cut(uint orientation, const Index& index) const;
};

enum ScannerMode {
    // scans FRONT_FACES_X, FRONT_FACES_Y, FRONT_FACES_Z,
    // BACK_FACES_X, BACK_FACES_Y, BACK_FACES_Z
    FRONT_XYZ_BACK_XYZ = 1,

    // scans FRONT_AND_BACK_FACES_X, FRONT_AND_BACK_FACES_Y, FRONT_AND_BACK_FACES_Z
    FRONT_AND_BACK_XYZ = 2,

};

enum ScanMode {
    NONE,
    FRONT_FACES_X,
    FRONT_FACES_Y,
    FRONT_FACES_Z,
    BACK_FACES_X,
    BACK_FACES_Y,
    BACK_FACES_Z,
    FRONT_AND_BACK_FACES_X,
    FRONT_AND_BACK_FACES_Y,
    FRONT_AND_BACK_FACES_Z,
};

class CompressedHermiteScanner :  public QOpenGLFunctions_4_2_Core {

    vector<GLuint> textures;
    ScannerMode mode;
    ScanMode scanMode;
    uint res;
    uint size; // res+1
    QGLShaderProgram* program;
    void bindTextures();
    void resetTextures();
public:
    shared_ptr<CompressedHermiteData> data;
    QMatrix4x4 projection;
    CompressedHermiteScanner(ScannerMode mode, uint res, float voxelGridRadius, QGLShaderProgram& scanShader);
    void begin(ScanMode mode, QMatrix4x4& V);
    void end();
    virtual ~CompressedHermiteScanner();
};

class CompressedHermiteSampler : public HermiteDataSampler {
private:
    CompressedHermiteData* data;
    inline void stepForward(uint orientation, const Index& edge, const Index &to, queue<Index>& indices);
    inline void stepBackward(uint orientation, const Index& edge, const Index &to, queue<Index>& indices);
    void floodFill();
    uint compressedEdgeData(uint orientation, const Index& from) const;
public:
    CompressedHermiteSampler(CompressedHermiteData& data);
    bool frontEdgeInfo(uint orientation, const Index& from, float& d, Vector3f& n) const override;
    bool backEdgeInfo(uint orientation, const Index& from, float& d, Vector3f& n) const override;
    bool intersectsEdge(uint orientation, const Index &from, float& d, Vector3f& n) const override;
    bool intersectsEdge(uint orientation, const Index &from, float& d) const override;

};

#endif // SCAN_H
