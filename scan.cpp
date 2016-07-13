#include "scan.h"
#include <iostream>

CompressedHermiteData::CompressedHermiteData(uint res) : res(res), size(res+1) {
    frontface_cuts.resize(3);
    backface_cuts.resize(3);
    for (int i = 0; i < 3; ++i) {
        frontface_cuts[i].resize(size*size*size);
        backface_cuts[i].resize(size*size*size);
    }
}

uint CompressedHermiteData::frontface_cut(uint orientation, const Index &index) const {
    return frontface_cuts[orientation][size*(index.y+index.z*size)+index.x];
}

uint CompressedHermiteData::backface_cut(uint orientation, const Index &index) const {
    return backface_cuts[orientation][size*(index.y+index.z*size)+index.x];
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

CompressedSignData::CompressedSignData(uint res, bool defaultSign) : res(res), size(res+1), depth(res/32+1) {
    if (defaultSign)
        data = vector<uint>(size*size*depth, std::numeric_limits<unsigned int>::max());
    else
        data = vector<uint>(size*size*depth, 0);
}

bool CompressedSignData::sign(const Index &index) const {
    return sign(index.x, index.y, index.z);
}

bool CompressedSignData::sign(uint x, uint y, uint z) const {
    if (x > res || y > res || z > res)
        return false;
    uint zCoord = z/32;
    uint offset = z - 32*zCoord;
    uint d = data[size*(size*zCoord+y)+x];
    d = d & pow2(offset);
    return d > 0;
}

void CompressedSignData::setInside(uint x, uint y, uint z) {
    if (x > res || y > res || z > res)
        return;
    uint zCoord = z/32;
    uint offset = z - 32*zCoord;
    uint d = pow2(offset);
    data[size*(size*zCoord+y)+x] |= d;

}
void CompressedSignData::setInside(const Index &index) {
    setInside(index.x, index.y, index.z);
}

void CompressedSignData::setOutside(uint x, uint y, uint z) {
    uint zCoord = z/32;
    uint offset = z - 32*zCoord;
    data[size*(size*zCoord+y)+x] &= ~(pow2(offset));
}
void CompressedSignData::setOutside(const Index &index) {
    setOutside(index.x, index.y, index.z);
}

CompressedEdgeData::CompressedEdgeData(uint res) : res(res), size(res+1), depth(res/32 + 1) {
    cuts.resize(3);
    for (int i = 0; i < 3; ++i) {
        cuts[i].resize(size*size*depth);
        cuts[i].resize(size*size*depth);
    }
}

bool CompressedEdgeData::cut(uint orientation, uint x, uint y, uint z) const {
    if (x > res || y > res || z >= res)
        return false;
    uint zCoord = z/32;
    uint offset = z - 32*zCoord;
    uint d = cuts[orientation][size*(size*zCoord+y)+x];
    d = d & pow2(offset);
    return d > 0;
}

bool CompressedEdgeData::cut(uint orientation, const Index &index) const {
    return cut(orientation, index.x, index.y, index.z);
}

Scanner::Scanner(uint texRes, uint slices, int texCount)
    : texRes(texRes), slices(slices) {
    initializeOpenGLFunctions();
    textures.resize(texCount);
    glGenTextures(texCount, &textures[0]);
    projection.setToIdentity();
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glFramebufferParameteri(GL_FRAMEBUFFER, GL_FRAMEBUFFER_DEFAULT_WIDTH, texRes);
    glFramebufferParameteri(GL_FRAMEBUFFER, GL_FRAMEBUFFER_DEFAULT_HEIGHT, texRes);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Scanner::resetTextures() {

    std::vector<GLint> emptyData(texRes*texRes*slices);
    glEnable(GL_TEXTURE_3D);
    for (uint i = 0; i < textures.size(); ++i) {
        glBindTexture(GL_TEXTURE_3D, textures[i]);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAX_LEVEL, 0);
        glTexImage3D(GL_TEXTURE_3D, 0, GL_R32UI, texRes, texRes, slices, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, &emptyData[0]);
    }
    glBindTexture(GL_TEXTURE_3D, 0);
}

void Scanner::bindTextures() {

    // bind textures/images
    for (uint i = 0; i < textures.size(); ++i) {
        glActiveTexture(GL_TEXTURE0+i);
        glBindTexture(GL_TEXTURE_3D, textures[i]);
        glBindImageTexture(i, textures[i], 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32UI);
    }
}

void Scanner::unbindImages() {
    for (uint t = 0; t < textures.size(); ++t) {
        glActiveTexture(GL_TEXTURE0+t);
        glBindImageTexture(t, 0, 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32UI);
    }
}


void Scanner::scan(const Renderable* scene, Direction dir) {

    resetTextures();
    bindTextures();
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    QMatrix4x4 view;
    view.setToIdentity();
    switch (dir) {
        case X:
            view.rotate(90,0,1,0); break;
        case Y:
            view.rotate(-90,1,0,0); break;
        case Z:
            view.rotate(180,0,1,0); break;
        default:
            break;
    }
    program->bind();
    for (int i = 0; i < textures.size(); ++i) {
        program->setUniformValue(("tex["+to_string(i)+"]").c_str(), i);
    }
    program->setUniformValue("res", texRes-1);
    configureProgram(dir);
    glViewport(0,0,texRes,texRes);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    scene->render(*program, projection, view);
    glFinish();
    glFlush();
    unbindImages();
    transferData(dir);
    glEnable(GL_DEPTH_TEST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

Scanner::~Scanner() {
    glDeleteTextures(textures.size(), &textures[0]);
    textures.clear();
}

void CompressedEdgeScanner::transferData(Direction dir) {
    glActiveTexture(GL_TEXTURE0);
    glGetTexImage(GL_TEXTURE_3D, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, &data->cuts[dir][0]);
}

void CompressedEdgeScanner::configureProgram(Direction dir) {
    uint index = dir;
    glUniformSubroutinesuiv(GL_FRAGMENT_SHADER, 1, &index);
}

/*
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
    for (uint i = 0; i < textures.size(); ++i) {
        program->setUniformValue(("tex["+to_string(i)+"]").c_str(), i);
    }

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
*/
void CompressedHermiteScanner::transferData(Direction dir) {

    glActiveTexture(GL_TEXTURE0);
    glGetTexImage(GL_TEXTURE_3D, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, &data->frontface_cuts[dir][0]);
    glActiveTexture(GL_TEXTURE1);
    glGetTexImage(GL_TEXTURE_3D, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, &data->backface_cuts[dir][0]);
    /*
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
    } */

}

void CompressedHermiteScanner::configureProgram(Direction dir) {
    uint index = dir;
    glUniformSubroutinesuiv(GL_FRAGMENT_SHADER, 1, &index);
}


CompressedSignSampler::CompressedSignSampler(CompressedEdgeData *data) : SignSampler(data->res), edgeData(data), data(new CompressedSignData(res)) {
    floodFill();
    edgeData = nullptr;
}

void CompressedSignSampler::step(uint orientation, const Index& edge, const Index& to, queue<Index> &indices) {
    if (data->sign(to) && !edgeData->cut(orientation, edge)) {
        data->setOutside(to);
        indices.push(to);
    }
}

void CompressedSignSampler::floodFill() {
    queue<Index> indices;
    Index start(0,0,0);
    indices.push(start);
    data->setInside(start);
    Index edge(0,0,0);
    while (!indices.empty()) {
        Index& from = indices.front();
        step(0, from, from.shiftX(1), indices);
        step(1, from, from.shiftY(1), indices);
        step(2, from, from.shiftZ(1), indices);

        edge = from.shiftX(-1);
        step(0, edge, edge, indices);
        edge = from.shiftY(-1);
        step(1, edge, edge, indices);
        edge = from.shiftZ(-1);
        step(2, edge, edge, indices);
        indices.pop();
    }
}

bool CompressedSignSampler::sign(uint x, uint y, uint z) const {
    return data->sign(x,y,z);
}

bool CompressedHermiteSampler::frontEdgeInfo(uint orientation, const Index& from, float& d, Vector3f& n) const {
    uint compressedData = data->frontface_cut(orientation, from-origin);
    return CompressedHermiteData::unpack(compressedData, d, n);
}

bool CompressedHermiteSampler::backEdgeInfo(uint orientation, const Index& from, float& d, Vector3f& n) const {
    uint compressedData = data->backface_cut(orientation, from-origin);
    return CompressedHermiteData::unpack(compressedData, d, n);
}

bool CompressedHermiteSampler::hasFrontCut(uint orientation, const Index &from) const {
    return data->frontface_cut(orientation, from-origin) > 0;
}

bool CompressedHermiteSampler::hasBackCut(uint orientation, const Index &from) const {
    return data->backface_cut(orientation, from-origin) > 0;
}

