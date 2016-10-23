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
    frontface_cuts.resize(3);
    backface_cuts.resize(3);
    for (int i = 0; i < 3; ++i) {
        frontface_cuts[i].resize(size*size*depth);
        backface_cuts[i].resize(size*size*depth);
    }
}

bool CompressedEdgeData::frontface_cut(uint orientation, uint x, uint y, uint z) const {
    if (x > res || y > res || z >= res)
        return false;
    uint zCoord = z/32;
    uint offset = z - 32*zCoord;
    uint d = frontface_cuts[orientation][size*(size*zCoord+y)+x];
    d = d & pow2(offset);
    return d > 0;
}

bool CompressedEdgeData::frontface_cut(uint orientation, const Index &index) const {
    return frontface_cut(orientation, index.x, index.y, index.z);
}

bool CompressedEdgeData::backface_cut(uint orientation, uint x, uint y, uint z) const {
    if (x > res || y > res || z >= res)
        return false;
    uint zCoord = z/32;
    uint offset = z - 32*zCoord;
    uint d = backface_cuts[orientation][size*(size*zCoord+y)+x];
    d = d & pow2(offset);
    return d > 0;
}

bool CompressedEdgeData::backface_cut(uint orientation, const Index &index) const {
    return backface_cut(orientation, index.x, index.y, index.z);
}



TextureTarget::TextureTarget(uint texRes, uint slices, int texCount)
    : texRes(texRes), slices(slices), texCount(texCount) {
    initializeOpenGLFunctions();
    textures.resize(texCount);
    glGenTextures(texCount, &textures[0]);
}

void TextureTarget::resetTextures() {

    std::vector<GLint> emptyData(texRes*texRes*slices);
    glEnable(GL_TEXTURE_3D);
    for (uint i = 0; i < textures.size(); ++i) {
        glBindTexture(GL_TEXTURE_3D, textures[i]);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAX_LEVEL, 0);
        glTexImage3D(GL_TEXTURE_3D, 0, GL_R32UI, texRes, texRes, slices, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, &emptyData[0]);
    }
    glBindTexture(GL_TEXTURE_3D, 0);
}

void TextureTarget::bindTextures() {

    // bind textures/images
    for (uint i = 0; i < textures.size(); ++i) {
        glActiveTexture(GL_TEXTURE0+i);
        glBindTexture(GL_TEXTURE_3D, textures[i]);
        glBindImageTexture(i, textures[i], 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32UI);
    }
}

void TextureTarget::unbindImages() {
    for (uint t = 0; t < textures.size(); ++t) {
        glActiveTexture(GL_TEXTURE0+t);
        glBindImageTexture(t, 0, 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32UI);
    }
}

void TextureTarget::startScan(Direction dir) {
    resetTextures();
    bindTextures();
}

void TextureTarget::endScan(Direction dir) {
    glMemoryBarrier(GL_TEXTURE_UPDATE_BARRIER_BIT);
    unbindImages();
}

TextureTarget::~TextureTarget() {
    glDeleteTextures(textures.size(), &textures[0]);
    textures.clear();
}

SSBOTarget::SSBOTarget(set<Intersection> &intersections)
    : buffer(new SSBO(3, intersections.size()*32, GL_DYNAMIC_DRAW)) {

    buffer->bind();
    uint i = 0;
    int values[4];
    float d = -1.0f;
    foreach (const Intersection& cut, intersections) {
        values[0] = cut.edge.x;
        values[1] = cut.edge.y;
        values[2] = cut.edge.z;
        values[3] = cut.status;
        buffer->bufferSubData(i*32, 16, values);
        buffer->bufferSubData(i*32+28, 4, &d);
        i++;
    }
    buffer->unBind();
}

void SSBOTarget::endScan() {
    //glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
}

Scanner::Scanner(ScanTarget *target, uint res)
    : target(target), res(res) {
    initializeOpenGLFunctions();
    projection.setToIdentity();
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glFramebufferParameteri(GL_FRAMEBUFFER, GL_FRAMEBUFFER_DEFAULT_WIDTH, res);
    glFramebufferParameteri(GL_FRAMEBUFFER, GL_FRAMEBUFFER_DEFAULT_HEIGHT, res);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Scanner::scan(RenderStrategy *scene, Direction dir) {
    target->startScan(dir);
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
    configureProgram(dir);
    scene->render(*program, projection, view);
    target->endScan(dir);
    transferData(dir);
}

void Scanner::scan(RenderStrategy *scene) {
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glViewport(0,0,res+1,res+1);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    target->startScan();
    program->bind();
    program->setUniformValue("res", res);
    configureProgram();
    scan(scene, X);
    scan(scene, Y);
    scan(scene, Z);
    target->endScan();
    transferData();
    glEnable(GL_DEPTH_TEST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

Scanner::~Scanner() {
    delete target;
    target = nullptr;
}

void CompressedEdgeScanner::configureProgram() {
    program->setUniformValue("tex[0]", 0);
    program->setUniformValue("tex[1]", 1);
}

void CompressedEdgeScanner::configureProgram(Direction dir) {
    uint index = dir;
    glUniformSubroutinesuiv(GL_FRAGMENT_SHADER, 1, &index);
}

void CompressedEdgeScanner::transferData(Direction dir) {
    glActiveTexture(GL_TEXTURE0);
    glGetTexImage(GL_TEXTURE_3D, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, &data.frontface_cuts[dir][0]);
    glActiveTexture(GL_TEXTURE1);
    glGetTexImage(GL_TEXTURE_3D, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, &data.backface_cuts[dir][0]);
}

void HermiteScanner::configureProgram(Direction dir) {
    uint index = dir;
    glUniformSubroutinesuiv(GL_FRAGMENT_SHADER, 1, &index);
}

void HermiteScanner::transferData() {
    SSBOTarget* t = (SSBOTarget*) target;
    t->buffer->bind();
    int e[4];
    Vector3f n;
    float d;
    Intersection key;
    for (uint i = 0; i < intersectCount; ++i) {
        t->buffer->getBufferSubData(i*32, 16, e);
        t->buffer->getBufferSubData(i*32+16, 12, n.data());
        t->buffer->getBufferSubData(i*32+28, 4, &d);
        key.edge.x = e[0];
        key.edge.y = e[1];
        key.edge.z = e[2];
        key.status = e[3];

        data->intersections[key] = HermiteData(n, d);
    }
    t->buffer->unBind();
}

CompressedSignSampler::CompressedSignSampler(CompressedEdgeData& data) : SignSampler(data.res), edgeData(&data), data(new CompressedSignData(res)) {
    floodFill();
    edgeData = nullptr;
}

void CompressedSignSampler::stepForward(uint dir, const Index& edge, const Index& to, queue<Index> &indices) {
    if (edgeData->frontface_cut(dir, edge)) {
        contributingIntersections.insert(Intersection(edge, Direction(dir), FRONT));
        for (int i = 0; i < 4; ++i) {
            contributingCells.insert(edge + cells_contain_edge_delta[dir][i]);
        }
    } else if (data->sign(to)) {
        data->setOutside(to);
        indices.push(to);
    }
}

void CompressedSignSampler::stepBackward(uint dir, const Index& edge, const Index& to, queue<Index> &indices) {
    if (edgeData->backface_cut(dir, edge)) {
        contributingIntersections.insert(Intersection(edge, Direction(dir), BACK));
        for (int i = 0; i < 4; ++i) {
            contributingCells.insert(edge + cells_contain_edge_delta[dir][i]);
        }
    } else if (data->sign(to)) {
        data->setOutside(to);
        indices.push(to);
    }
}

void CompressedSignSampler::floodFill() {
    queue<Index> indices;
    for (uint i = 0; i <= res; ++i) {
        indices.push(Index(i, 0, 0));
        data->setOutside(i, 0, 0);
        indices.push(Index(0, i, 0));
        data->setOutside(0, i, 0);
        indices.push(Index(0, 0, i));
        data->setOutside(0,0,i);
    }
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
    std::cout << "contributing intersections " << contributingIntersections.size() << std::endl;
}
bool CompressedSignSampler::sign(uint x, uint y, uint z) const {
    return data->sign(x,y,z);
}

CompressedSignSampler::~CompressedSignSampler() {
    delete data;
    data = nullptr;
}

HermiteData* SparseHermiteSampler::edgeInfo(Direction dir, Orientation orientation, const Index& edge) const {
    Intersection key(edge, dir, orientation);
    if (!data->intersections.count(key))
        return nullptr;

    return &data->intersections[key];
}

bool SparseHermiteSampler::hasCut(Direction dir, Orientation orientation, const Index& edge) const {
    return data->intersections.count(Intersection(edge, dir, orientation));
}

SparseHermiteSampler::~SparseHermiteSampler() {
    delete data;
    data = nullptr;
}

