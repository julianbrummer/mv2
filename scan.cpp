#include "scan.h"
#include <iostream>

CompressedHermiteData::CompressedHermiteData(uint res) : res(res) {
    frontface_cuts.resize(3);
    backface_cuts.resize(3);
    frontface_cuts[0] = vector_3_uint(res, vector_2_uint(res+1, vector<uint>(res+1, 0)));
    backface_cuts[0] = vector_3_uint(res, vector_2_uint(res+1, vector<uint>(res+1, 0)));
    frontface_cuts[1] = vector_3_uint(res+1, vector_2_uint(res, vector<uint>(res+1, 0)));
    backface_cuts[1] = vector_3_uint(res+1, vector_2_uint(res, vector<uint>(res+1, 0)));
    frontface_cuts[2] = vector_3_uint(res+1, vector_2_uint(res+1, vector<uint>(res, 0)));
    backface_cuts[2] = vector_3_uint(res+1, vector_2_uint(res+1, vector<uint>(res, 0)));
}

uint CompressedHermiteData::frontface_cut(uint orientation, const Index &index) const {
    return frontface_cuts[orientation][index.x][index.y][index.z];
}

uint CompressedHermiteData::backface_cut(uint orientation, const Index &index) const {
    return backface_cuts[orientation][index.x][index.y][index.z];
}

CompressedHermiteScanner::CompressedHermiteScanner(ScannerMode mode, uint res, float voxelGridRadius, QGLShaderProgram& scanShader)
    : mode(mode), scanMode(NONE), res(res), size(res+1), program(&scanShader), data(new CompressedHermiteData(res)) {
    initializeOpenGLFunctions();
    textures.resize(mode);
    glGenTextures(mode, &textures[0]);

    //shift near and far plane about a half voxel cell
    //because rays are casted from the pixel center
    projection.ortho(-voxelGridRadius, voxelGridRadius,
                     -voxelGridRadius, voxelGridRadius,
                     -voxelGridRadius + voxelGridRadius/size,
                     voxelGridRadius - voxelGridRadius/size);
}

void CompressedHermiteScanner::resetTextures() {

    std::vector<GLint> emptyData(size*size*res);
    glEnable(GL_TEXTURE_3D);
    for (uint i = 0; i < textures.size(); ++i) {
        glBindTexture(GL_TEXTURE_3D, textures[i]);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAX_LEVEL, 0);
        glTexImage3D(GL_TEXTURE_3D, 0, GL_R32UI, size, size, res, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, &emptyData[0]);
    }
    glBindTexture(GL_TEXTURE_3D, 0);
}

void CompressedHermiteScanner::bindTextures() {

    // bind textures/images
    for (uint i = 0; i < textures.size(); ++i) {
        glActiveTexture(GL_TEXTURE0+i);
        glBindTexture(GL_TEXTURE_3D, textures[i]);
        glBindImageTexture(i, textures[i], 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32UI);
    }
}

void CompressedHermiteScanner::begin(ScanMode mode, QMatrix4x4& V) {

    switch (mode) {
        case FRONT_FACES_X: case FRONT_FACES_Y: case FRONT_FACES_Z:
            assert(this->mode == FRONT_XYZ_BACK_XYZ);
            glEnable(GL_CULL_FACE);
            glCullFace(GL_BACK);
            break;
        case BACK_FACES_X: case BACK_FACES_Y: case BACK_FACES_Z:
            assert(this->mode == FRONT_XYZ_BACK_XYZ);
            glEnable(GL_CULL_FACE);
            glCullFace(GL_FRONT);
            break;
        case FRONT_AND_BACK_FACES_X: case FRONT_AND_BACK_FACES_Y: case FRONT_AND_BACK_FACES_Z:
            assert(this->mode == FRONT_AND_BACK_XYZ);
            glDisable(GL_CULL_FACE);
            break;
        case NONE:
            return;
    }
    scanMode = mode;

    resetTextures();
    bindTextures();
    program->bind();
    /*
    for (uint i = 0; i < textures.size(); ++i) {
        program->setUniformValue(("tex["+to_string(i)+"]").c_str(), i);
    }
*/
    program->setUniformValue("tex[0]", 0);
    program->setUniformValue("tex[1]", 1);

    glDisable(GL_DEPTH_TEST);

    // set viewport to grid resolution
    glViewport(0,0,size,size);

    V.setToIdentity();
    switch (mode) {
        case FRONT_FACES_X: case BACK_FACES_X: case FRONT_AND_BACK_FACES_X:
            V.rotate(90,0,1,0); break;
        case FRONT_FACES_Y: case BACK_FACES_Y: case FRONT_AND_BACK_FACES_Y:
            V.rotate(-90,1,0,0); break;
        case FRONT_FACES_Z: case BACK_FACES_Z: case FRONT_AND_BACK_FACES_Z:
            V.rotate(180,0,1,0); break;
        default:
            break;
    }

    program->setUniformValue("res", res);
}

void CompressedHermiteScanner::end() {
    glFinish();
    glFlush();
    // unbind images
    for (uint i = 0; i < textures.size(); ++i) {
        glActiveTexture(GL_TEXTURE0+i);
        glBindImageTexture(i, 0, 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32UI);
    }
    vector<GLuint> texData;
    ulong dataSize = size*size*res;
    texData.resize(dataSize);
    glActiveTexture(GL_TEXTURE0);
    glGetTexImage(GL_TEXTURE_3D, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, &texData[0]);
    ulong i = 0;
    switch (scanMode) {
        case FRONT_FACES_X: case FRONT_AND_BACK_FACES_X:
            for (uint x = 0; x < res; ++x)
                for (uint y = 0; y < size; ++y)
                    for (uint z = 0; z < size; ++z)
                        data->frontface_cuts[0][x][y][z] = texData[i++];
            break;
        case FRONT_FACES_Y: case FRONT_AND_BACK_FACES_Y:
            for (uint y = 0; y < res; ++y)
                for (uint z = 0; z < size; ++z)
                    for (uint x = 0; x < size; ++x)
                        data->frontface_cuts[1][x][y][z] = texData[i++];
            break;
        case FRONT_FACES_Z: case FRONT_AND_BACK_FACES_Z:
            for (uint z = 0; z < res; ++z)
                for (uint y = 0; y < size; ++y)
                    for (int x = size-1; x >= 0; --x)
                        data->frontface_cuts[2][x][y][z] = texData[i++];
            break;
        default:
            break;
    }

    if (mode == FRONT_AND_BACK_XYZ) {
        texData.clear();
        texData.resize(dataSize);
        glActiveTexture(GL_TEXTURE1);
        glGetTexImage(GL_TEXTURE_3D, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, &texData[0]);
    }
    i = 0;
    switch (scanMode) {
        case BACK_FACES_X: case FRONT_AND_BACK_FACES_X:
            for (uint x = 0; x < res; ++x)
                for (uint y = 0; y < size; ++y)
                    for (uint z = 0; z < size; ++z)
                        data->backface_cuts[0][x][y][z] = texData[i++];
            break;
        case BACK_FACES_Y: case FRONT_AND_BACK_FACES_Y:
            for (uint y = 0; y < res; ++y)
                for (uint z = 0; z < size; ++z)
                    for (uint x = 0; x < size; ++x)
                        data->backface_cuts[1][x][y][z] = texData[i++];
            break;
        case BACK_FACES_Z: case FRONT_AND_BACK_FACES_Z:
            for (uint z = 0; z < res; ++z)
                for (uint y = 0; y < size; ++y)
                    for (int x = size-1; x >= 0; --x)
                        data->backface_cuts[2][x][y][z] = texData[i++];
            break;
        default:
            break;
    }

    glEnable(GL_DEPTH_TEST);
}

CompressedHermiteScanner::~CompressedHermiteScanner() {
    glDeleteTextures(textures.size(), &textures[0]);
    textures.clear();
}

CompressedHermiteSampler::CompressedHermiteSampler(CompressedHermiteData *data) : HermiteDataSampler(data->res), data(data) {
    floodFill();
}

bool CompressedHermiteData::unpack(uint data, float& d) {
    if (data == 0)
        return false;
    d = (float)(((data >> 24) & 255)-1)/254.0; // distance from grid point 0..1
    return true;
}

bool CompressedHermiteData::unpack(uint data, float& d, Vector3f& n) {
    if (data == 0)
        return false;
    n[2] = (data & 255)*2/255.0 - 1;
    n[1] = ((data >> 8) & 255)*2/255.0 - 1;
    n[0] = ((data >> 16) & 255)*2/255.0 - 1;
    d = (float)(((data >> 24) & 255)-1)/254.0; // distance from grid point 0..1
    return true;
}

inline void CompressedHermiteSampler::stepForward(uint orientation, const Index& edge, const Index& to, queue<Index> &indices) {
    if (sign(to) && data->frontface_cut(orientation, edge) == 0) {
        setSign(to, false);
        indices.push(to);
    }
}

inline void CompressedHermiteSampler::stepBackward(uint orientation, const Index& edge, const Index& to, queue<Index> &indices) {
    if (sign(to) && data->backface_cut(orientation, edge) == 0) {
        setSign(to, false);
        indices.push(to);
    }
}

void CompressedHermiteSampler::floodFill() {
    queue<Index> indices;
    Index start(0,0,0);
    indices.push(start);
    setSign(start, false);
    Index edge(0,0,0);
    while (!indices.empty()) {
        Index& from = indices.front();
        stepForward(0, from, from.shiftX(1), indices);
        stepForward(1, from, from.shiftY(1), indices);
        stepForward(2, from, from.shiftZ(1), indices);

        edge = from.shiftX(-1);
        stepBackward(0, edge, edge, indices);
        edge = from.shiftY(-1);
        stepBackward(1, edge, edge, indices);
        edge = from.shiftZ(-1);
        stepBackward(2, edge, edge, indices);
        indices.pop();
    }
}

uint CompressedHermiteSampler::compressedEdgeData(uint orientation, const Index& from) const {
    Index to = from;
    to[orientation] += 1;

    uint compressedData = 0;
                                  //    ff
    if (!sign(from) && sign(to))  // O--|--X
        compressedData = data->frontface_cut(orientation, from);
                                       //    bf
    else if (sign(from) && !sign(to))  // X--|--O
        compressedData = data->backface_cut(orientation, from);

    return compressedData;
}


bool CompressedHermiteSampler::frontEdgeInfo(uint orientation, const Index& from, float& d, Vector3f& n) const {
    uint compressedData = data->frontface_cut(orientation, from);
    return CompressedHermiteData::unpack(compressedData, d, n);
}

bool CompressedHermiteSampler::backEdgeInfo(uint orientation, const Index& from, float& d, Vector3f& n) const {
    uint compressedData = data->backface_cut(orientation, from);
    return CompressedHermiteData::unpack(compressedData, d, n);
}

bool CompressedHermiteSampler::intersectsEdge(uint orientation, const Index& from, float& d, Vector3f& n) const {
    return CompressedHermiteData::unpack(compressedEdgeData(orientation, from), d, n);
}

bool CompressedHermiteSampler::intersectsEdge(uint orientation, const Index &from, float& d) const {
    return CompressedHermiteData::unpack(compressedEdgeData(orientation, from), d);
}

