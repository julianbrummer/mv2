#include "dmc.h"
#include <iostream>
#include <queue>
#include <stack>

QEF::QEF() {
    for (int i = 0; i < 6; ++i) {
        a[i] = 0.0;
    }
    b.setZero(3);
    c = 0.0;
    m.setZero(3);
    dimension = SolutionSpace::UNDEFINED;
}

void QEF::add(const Vector3f& normal, const Vector3f& point) {
    //E(x) = (n^T(x - p))^2
    //E(x) = (n^Tx - n^Tp)^2 = x^T(nn^T)x + 2d*n^Tx + d^2
    //grad(E(x)) = 2Ax + 2b = 0
    // Ax = -b
    Vector3d n = normal.cast<double>();
    Vector3d p = point.cast<double>();
    double d = - n.transpose() * p; // n^T p
    Matrix3d A = n * n.transpose(); //nn^T
    a[0] += A(0,0);
    a[1] += A(0,1);
    a[2] += A(0,2);
    a[3] += A(1,1);
    a[4] += A(1,2);
    a[5] += A(2,2);
    b += d*n;
    c += d*d;
}

void QEF::add(const QEF& qef) {
    for (int i = 0; i < 6; ++i) {
        a[i] += qef.a[i];
    }
    b += qef.b;
    c += qef.c;
}

SolutionSpace QEF::solve(const Vector3f &m, float truncation, Vector3d &c) {
    // solve for minimizer x = c+m of Ax = -b with minimal distance to m
    // -b = A(c+m) = Ac + Am
    // -Am - b = Ac
    // c = A^+ (-Am - b)
    Matrix3d A;
    A << a[0], a[1], a[2],
         a[1], a[3], a[4],
         a[2], a[4], a[5];
    JacobiSVD<Matrix3Xd> svd(A, ComputeThinU | ComputeThinV); // U D V^T
    svd.setThreshold(truncation/svd.singularValues()[0]); // set singular values <= truncation to zero
                            // to avoid high values in diagonal matrix D^+

    // solve for c
    c = svd.solve(-A*m.cast<double>()-b);
    dimension = static_cast<SolutionSpace>(svd.rank());
    return dimension;
}

SolutionSpace QEF::solve(float truncation, Vector3d& c) {
    return solve(m, truncation, c);
}

double QEF::evaluate(const Vector3f& v) const {
    Matrix3d A;
    Vector3d x = v.cast<double>();
    A << a[0], a[1], a[2],
         a[1], a[3], a[4],
         a[2], a[4], a[5];
    double dA = x.transpose()*A*x;
    double db = 2*b.transpose()*x;
    return dA + db + c;
}

void VertexNode::computeError() {
    error = qef.evaluate(v);
}

bool compareSurfaceIndex(shared_ptr<VertexNode> v1, shared_ptr<VertexNode> v2) {
    return v1->surfaceIndex < v2->surfaceIndex;
}

bool DMCOctreeCell::isHomogeneous() const {
    if (hasChildren()) {
        for (int i = 0; i < 8; ++i)
            if (!child(i)->isHomogeneous())
                return false;
        return true;
    }
    return signConfig == 0 || signConfig == 255;
}

DMCOctreeNode::DMCOctreeNode(uint8_t level) : DMCOctreeCell(level)  {
    for (int i = 0; i < 8; ++i) {
        children[i] = nullptr;
    }
}

DMCOctreeNode::~DMCOctreeNode() {
    removeChildren();
}

bool DMCOctreeNode::hasChildren() const {
    return children[0].get();
}

DMCOctreeCell* DMCOctreeNode::child(uint i) const {
    if (hasChildren())
        return children[i].get();

    return nullptr;
}

void DMCOctreeNode::removeChildren() {
    for (int i = 0; i < 8; ++i) {
        children[i] = nullptr;
    }
}

DMCOctreeLeaf::DMCOctreeLeaf(uint8_t level) : DMCOctreeCell(level) {
    for (uint i = 0; i < 12; ++i) {
        edgeVertices[i] = -1;
    }
}

void DMCVerticesBuilder::handle(const DMCOctreeCell *node, const Index &index) {
    // push vertices
    offset[node->level][index.x][index.y][index.z] = positions.size();
    count[node->level][index.x][index.y][index.z] = node->vertices.size() + node->strayVertices.size();
    for (const shared_ptr<VertexNode> vNode : node->vertices) {
        positions.push_back(vNode->v);
        colors.push_back(vNode->collapsable? Color::WHITE : Color::RED);
    }
    for (const shared_ptr<VertexNode> vNode : node->strayVertices) {
        positions.push_back(vNode->v);
        colors.push_back(Color::BLUE);
    }
}

void DMCCellBuilder::handle(const DMCOctreeCell *node, const Index &index) {
    offset[node->level][index.x][index.y][index.z] = positions.size();
    // calculate cell corners
    array<Vector3f, 8> corners;
    for (int i = 0; i < 8; ++i) {
        Index corner = index + corner_delta[i];
        corners[i] = (1.0/pow2(node->level))*Vector3f(corner.x, corner.y, corner.z);
    }
    // push cell edges
    for (int i = 0; i < 12; ++i) {
      positions.push_back(corners[edge_corners[i][0]]);
      positions.push_back(corners[edge_corners[i][1]]);
    }
}


void DualMarchingCubes::assignSurface(const vector<shared_ptr<VertexNode>>& vertices, int from, int to) const {
    for (shared_ptr<VertexNode> v : vertices) {
        if (v->surfaceIndex == from) {
            v->surfaceIndex = to;
        }
    }
}

void DualMarchingCubes::clusterEdge(array<DMCOctreeCell* , 4> nodes, uint orientation,
                                    uint& maxSurfaceIndex, const vector<shared_ptr<VertexNode>>& vertices) {

    if (nodes[0]->hasChildren() && nodes[1]->hasChildren() && nodes[2]->hasChildren() && nodes[3]->hasChildren()) {
        for (uint i = 0; i < 2; ++i) { // subdivided edge
            clusterEdge(array<DMCOctreeCell*, 4>{
                            nodes[0]->child(edge_edge_mask[orientation][i][0]),
                            nodes[1]->child(edge_edge_mask[orientation][i][1]),
                            nodes[2]->child(edge_edge_mask[orientation][i][2]),
                            nodes[3]->child(edge_edge_mask[orientation][i][3])},
                        orientation, maxSurfaceIndex, vertices);
        }

    } else if (!nodes[0]->hasChildren() && !nodes[1]->hasChildren() && !nodes[2]->hasChildren() && !nodes[3]->hasChildren()) {

        uint edgeIndex = edge_of_four_cells[orientation][0];
        if (!nodes[0]->signChange(edgeIndex))
            return;


        array<VertexNode*, 4> v{nullptr, nullptr, nullptr, nullptr};
        for (uint i = 0; i < 4; ++i) {
            // get vertices assigned to this edge
            uint edgeIndex = edge_of_four_cells[orientation][i];
            v[i] = nodes[i]->vertexAssignedTo(edgeIndex);
            assert(v[i]);
            // folow parent pointers up the vertex tree
            while (!v[i]->parent.expired())
                v[i] = v[i]->parent.lock().get();
        }

        // cluster vertices connected by surfaces dual to this edge
        int surfaceIndex = -1;
        for (uint i = 0; i < 4; ++i) {
            if (v[i]->surfaceIndex != -1) {
                if (surfaceIndex == -1)
                    surfaceIndex = v[i]->surfaceIndex;
                else if (surfaceIndex != v[i]->surfaceIndex) {
                    assignSurface(vertices, v[i]->surfaceIndex, surfaceIndex);
                }
            }
        }

        if (surfaceIndex == -1)
            surfaceIndex = maxSurfaceIndex++;

        for (uint i = 0; i < 4; ++i) {
            if (v[i]->surfaceIndex == -1)
                v[i]->surfaceIndex = surfaceIndex;
        }
    }

}

void DualMarchingCubes::clusterFace(array<DMCOctreeCell*, 2> nodes, uint orientation,
                                    uint& maxSurfaceIndex, const vector<shared_ptr<VertexNode>>& vertices) {
    for (int i = 0; i < 4; ++i) { // 4 faces tiling coarse face
        // 2 child cells define 1 face
        if (nodes[0]->child(face_face_mask[orientation][i][0])->hasChildren()
                    && nodes[1]->child(face_face_mask[orientation][i][1])->hasChildren()) {
                clusterFace(array<DMCOctreeCell*, 2>{
                                nodes[0]->child(face_face_mask[orientation][i][0]),
                                nodes[1]->child(face_face_mask[orientation][i][1])},
                            orientation, maxSurfaceIndex, vertices);
        }
    }


    for (uint i = 0; i < 4; ++i) { // 4 edges in coarse face
        // 4 child cells define 1 edge
        clusterEdge(array<DMCOctreeCell*, 4>{
                        nodes[face_edge_mask[orientation][i][0][0]]->child(face_edge_mask[orientation][i][0][1]),
                        nodes[face_edge_mask[orientation][i][1][0]]->child(face_edge_mask[orientation][i][1][1]),
                        nodes[face_edge_mask[orientation][i][2][0]]->child(face_edge_mask[orientation][i][2][1]),
                        nodes[face_edge_mask[orientation][i][3][0]]->child(face_edge_mask[orientation][i][3][1])},
                    face_edge_orientation[orientation][i], maxSurfaceIndex, vertices);
    }

}

void DualMarchingCubes::clusterCell(const Index& cell_index, DMCOctreeCell* node) {
    if (node->hasChildren()) {
        // cluster children cells first
        for (uint i = 0; i < 8; ++i) {
            Index child_index = cell_index+sampler->nodeSize(node->level+1)*child_origin[i];
            clusterCell(child_index, node->child(i));
        }

        // gather vertices from direct children
        vector<shared_ptr<VertexNode>> vertices;
        for (uint i = 0; i < 8; ++i) {
            for (shared_ptr<VertexNode> v : node->child(i)->vertices) {
                vertices.push_back(v);
            }
            for (shared_ptr<VertexNode> v : node->child(i)->strayVertices) {
                vertices.push_back(v);
            }
        }
        uint maxSurfaceIndex = 0;

        // cluster inner faces
        for (uint i = 0; i < 3; ++i) { // 3 main inner planes (yz, xz, xy)
            for (uint j = 0; j < 4; ++j) { // each tiled by 4 faces
                if (node->child(cell_face_mask[i][j][0])->hasChildren()
                        && node->child(cell_face_mask[i][j][1])->hasChildren()) {
                    clusterFace(array<DMCOctreeCell*, 2>{
                                    node->child(cell_face_mask[i][j][0]),
                                    node->child(cell_face_mask[i][j][1])},
                                i, maxSurfaceIndex, vertices);
                }
            }
        }

        // cluster inner edges
        for (int i = 0; i < 3; ++i) { // 3 edge orientations (x, y, z)
            for (int j = 0; j < 2; ++j) { // each tiled by 2 edges
                clusterEdge(array<DMCOctreeCell*, 4>{
                                node->child(cell_edge_mask[i][j][0]),
                                node->child(cell_edge_mask[i][j][1]),
                                node->child(cell_edge_mask[i][j][2]),
                                node->child(cell_edge_mask[i][j][3])},
                            i, maxSurfaceIndex, vertices);
            }
        }

        sort(vertices.begin(), vertices.end(), compareSurfaceIndex);

        int8_t last_index = -1;
        // the count of QEF masspoints per cluster/surface
        vector<uint> mCount;
        for (shared_ptr<VertexNode> v : vertices) {
            int8_t index = v->surfaceIndex;
            if (index == -1) // stray vertex may be clustered in higher level
               node->strayVertices.push_back(v);
            else {
                if (index > last_index) { // new surface/cluster?
                    node->vertices.push_back(shared_ptr<VertexNode>(new VertexNode()));
                    mCount.push_back(0);
                    last_index = index;
                }
                QEF& qef = node->vertices.back()->qef;
                // add QEFs of connected vertices
                qef.add(v->qef);
                if (qef.dimension == v->qef.dimension) {
                    qef.m += v->qef.m;
                    mCount.back()++;
                } else if (qef.dimension < v->qef.dimension) {
                    qef.m = v->qef.m;
                    qef.dimension = v->qef.dimension;
                    mCount.back() = 1;
                }
                v->parent = node->vertices.back();
            }

        }


        // generate vertices
        for (uint i = 0; i < node->vertices.size(); ++i) {
            VertexNode* vNode = node->vertices[i].get();
            // average masspoint from QEFs of connected vertices with highest dimension
            vNode->qef.m /= (float) mCount[i];
            generateVertex(cell_index, node->level, vNode->qef, vNode->v);
            vNode->computeError();
        }

    }
}

void DualMarchingCubes::updateCollapsableFlag(const DMCOctreeCell *node, float max_error) {
    // reset vertex indices
    for (shared_ptr<VertexNode> vNode : node->vertices)
        vNode->vertexIndex = -1;

    if (node->hasChildren()) { // only update non-leaf cells
        for (shared_ptr<VertexNode> vNode : node->vertices) {
            if (vNode->error <= max_error)
                vNode->collapsable = true;
            else
                vNode->collapsable = false;
        }
        for (int i = 0; i < 8; ++i)
            updateCollapsableFlag(node->child(i), max_error);
    }
}

vector<VertexNode*> DualMarchingCubes::makeUnique(const array<VertexNode*, 4> v) {
    vector<VertexNode*> result;
    for (int i = 0; i < 4; ++i) {
        if (find(result.begin(), result.end(), v[i]) == result.end()) {
            result.push_back(v[i]);
        }
    }
    return result;
}

void DualMarchingCubes::triangle(const vector<VertexNode*> v, initializer_list<int> index_order, aligned_vector3f& positions, vector<uint>& indices) {
    for (int i = 0; i < 3; ++i) {
        int index = *(index_order.begin()+i);
        if (v[index]->vertexIndex == -1) {
            v[index]->vertexIndex = positions.size();
            positions.push_back(v[index]->v);
        }
        indices.push_back(v[index]->vertexIndex);
    }
}

bool DualMarchingCubes::checkNormal(const vector<VertexNode*> v, initializer_list<int> normal_order, const Vector3f& dir) {
    const int* p = normal_order.begin();
    Vector3f n1 = (v[*(p++)]->v - v[*(p++)]->v)
           .cross(v[*(p++)]->v - v[*(p++)]->v);
    Vector3f n2 = (v[*(p++)]->v - v[*(p++)]->v)
           .cross(v[*(p++)]->v - v[*(p++)]->v);
    return n1.transpose() * dir >= 0 && n2.transpose() * dir >= 0;
}

void DualMarchingCubes::triangulate(const vector<VertexNode*> v, bool front_face, const uint orientation, aligned_vector3f& positions, vector<uint>& indices) {
    Vector3f dir(0,0,0);
    dir[orientation] = front_face? 1 : -1;
    float d0_2 = (v[0]->v - v[2]->v).squaredNorm();
    float d1_3 = (v[1]->v - v[3]->v).squaredNorm();
    if (front_face) {
        if ((d0_2 < d1_3 && checkNormal(v, {1,0,2,0, 2,0,3,0}, dir)) ||
            (d0_2 >= d1_3 && !checkNormal(v, {1,0,3,0, 2,1,3,1}, dir))) {
            triangle(v, {0,1,2}, positions, indices);
            triangle(v, {0,2,3}, positions, indices);
        } else {
            triangle(v, {0,1,3}, positions, indices);
            triangle(v, {1,2,3}, positions, indices);
        }
    } else {
        if ((d0_2 < d1_3 && checkNormal(v, {2,0,1,0, 3,0,2,0}, dir)) ||
            (d0_2 >= d1_3 && !checkNormal(v, {3,0,1,0, 3,1,2,1}, dir))) {
            triangle(v, {0,2,1}, positions, indices);
            triangle(v, {0,3,2}, positions, indices);
        } else {
            triangle(v, {0,3,1}, positions, indices);
            triangle(v, {1,3,2}, positions, indices);
        }
    }
}

void DualMarchingCubes::edgeProc(array<DMCOctreeCell* , 4> nodes, uint orientation, aligned_vector3f& positions, vector<uint>& indices) {

    if (nodes[0]->hasChildren() && nodes[1]->hasChildren() && nodes[2]->hasChildren() && nodes[3]->hasChildren()) {
        for (uint i = 0; i < 2; ++i) { // subdivided edge
            edgeProc(array<DMCOctreeCell*, 4>{
                            nodes[0]->child(edge_edge_mask[orientation][i][0]),
                            nodes[1]->child(edge_edge_mask[orientation][i][1]),
                            nodes[2]->child(edge_edge_mask[orientation][i][2]),
                            nodes[3]->child(edge_edge_mask[orientation][i][3])},
                        orientation, positions, indices);
        }

    } else if (!nodes[0]->hasChildren() && !nodes[1]->hasChildren() && !nodes[2]->hasChildren() && !nodes[3]->hasChildren()) {

        uint edgeIndex = edge_of_four_cells[orientation][0];
        if (!nodes[0]->signChange(edgeIndex))
            return;


        array<VertexNode*, 4> vNodes{nullptr, nullptr, nullptr, nullptr};
        for (uint i = 0; i < 4; ++i) {
            // get vertices assigned to this edge
            uint edgeIndex = edge_of_four_cells[orientation][i];
            vNodes[i] = nodes[i]->vertexAssignedTo(edgeIndex);
            assert(vNodes[i]);
            // folow parent pointers up the vertex tree to the last vertex marked as collapsable
            while (!vNodes[i]->parent.expired() && vNodes[i]->parent.lock()->collapsable)
                vNodes[i] = vNodes[i]->parent.lock().get();
        }
        vector<VertexNode*> v = makeUnique(vNodes); // remove doubles (e.g vNodes have same ancestor)

        if (nodes[0]->frontface(edgeIndex)) {
            if (v.size() == 4) { // quad
                triangulate(v, true, orientation, positions, indices);
            } else if (v.size() == 3) { // triangle
                triangle(v, {0,1,2}, positions, indices);
            }
        } else if (nodes[0]->backface(edgeIndex)) {
            if (v.size() == 4) { // quad
                triangulate(v, false, orientation, positions, indices);
            } else if (v.size() == 3) { // triangle
                triangle(v, {0,2,1}, positions, indices);
            }
        }
    }

}

void DualMarchingCubes::faceProc(array<DMCOctreeCell*, 2> nodes, uint orientation, aligned_vector3f& positions, vector<uint>& indices) {
    for (uint i = 0; i < 4; ++i) { // 4 faces tiling coarse face
        // 2 child cells define 1 face
        if (nodes[0]->child(face_face_mask[orientation][i][0])->hasChildren()
                    && nodes[1]->child(face_face_mask[orientation][i][1])->hasChildren()) {
                faceProc(array<DMCOctreeCell*, 2>{
                                nodes[0]->child(face_face_mask[orientation][i][0]),
                                nodes[1]->child(face_face_mask[orientation][i][1])},
                            orientation, positions, indices);
        }
    }


    for (uint i = 0; i < 4; ++i) { // 4 edges in coarse face
        // 4 child cells define 1 edge
        edgeProc(array<DMCOctreeCell*, 4>{
                        nodes[face_edge_mask[orientation][i][0][0]]->child(face_edge_mask[orientation][i][0][1]),
                        nodes[face_edge_mask[orientation][i][1][0]]->child(face_edge_mask[orientation][i][1][1]),
                        nodes[face_edge_mask[orientation][i][2][0]]->child(face_edge_mask[orientation][i][2][1]),
                        nodes[face_edge_mask[orientation][i][3][0]]->child(face_edge_mask[orientation][i][3][1])},
                    face_edge_orientation[orientation][i], positions, indices);
    }
}

void DualMarchingCubes::cellProc(const DMCOctreeCell *node, aligned_vector3f& positions, vector<uint>& indices) {
    if (node->hasChildren()) {
        // inner cells
        for (uint i = 0; i < 8; ++i) {
            cellProc(node->child(i), positions, indices);
        }
        // inner faces
        for (uint i = 0; i < 3; ++i) { // 3 main inner planes (yz, xz, xy)
            for (uint j = 0; j < 4; ++j) { // each tiled by 4 faces
                if (node->child(cell_face_mask[i][j][0])->hasChildren()
                        && node->child(cell_face_mask[i][j][1])->hasChildren()) {
                    faceProc(array<DMCOctreeCell*, 2>{
                                    node->child(cell_face_mask[i][j][0]),
                                    node->child(cell_face_mask[i][j][1])},
                                i, positions, indices);
                }
            }
        }
        // inner edges
        for (uint i = 0; i < 3; ++i) { // 3 edge orientations (x, y, z)
            for (uint j = 0; j < 2; ++j) { // each tiled by 2 edges
                edgeProc(array<DMCOctreeCell*, 4>{
                                node->child(cell_edge_mask[i][j][0]),
                                node->child(cell_edge_mask[i][j][1]),
                                node->child(cell_edge_mask[i][j][2]),
                                node->child(cell_edge_mask[i][j][3])},
                            i, positions, indices);
            }
        }
    }

}

bool DualMarchingCubes::inCell(Vector3d& pos, const Vector3d& cellOrigin, const double size) const {
    return pos[0] <= size+cellOrigin[0] && pos[1] <= size+cellOrigin[1] && pos[2] <= size+cellOrigin[2] &&
           pos[0] >= cellOrigin[0] && pos[1] >= cellOrigin[1] && pos[2] >= cellOrigin[2];
}

void DualMarchingCubes::generateVertex(const Index& cell_index, uint8_t level, QEF& qef, Vector3f& v) {
    Vector3d vQEF(0,0,0);
    qef.solve(truncation, vQEF); // relative to masscenter
    vQEF += qef.m.cast<double>();  // relative to cellGrid origin
    Vector3d cell_origin(cell_index.x,cell_index.y, cell_index.z);
    cell_origin /= sampler->res;
    double cell_size = cellSize(level);
    if (inCell(vQEF, cell_origin, cell_size)) {
        v = vQEF.cast<float>();
    } else
        v = qef.m;

    //TODO better vertex placement
}


void DualMarchingCubes::initQEF(const int edges[], uint count, const Index& cell_index, QEF& qef) const {
    for (uint i = 0; i < count; ++i) {
        int e = edges[i];
        Index edge = cell_index + edge_origin[e];
        Vector3f n;
        float d;
        uint orientation = edge_orientation[e];
        assert(sampler->intersectsEdge(orientation, edge, d, n));
        Vector3f p = Vector3f(edge.x,edge.y,edge.z);
        p[orientation] += d;
        p /= sampler->res;
        qef.add(n, p);
        qef.m += p;
    }
    qef.m /= count;
}

void DualMarchingCubes::createVertexNodes(DMCOctreeLeaf& leaf, const Index& leaf_index) {
    uint vCount = vertexCount[leaf.signConfig];
    if (vCount == 0)
        return;
    uint eIndex = 0;
    int vEdges[8] = {-1,-1,-1,-1,-1,-1,-1,-1};
    for (uint i = 0; i < 16; ++i) {
        int edge = edgeTable[leaf.signConfig][i];
        if (edge < 0) {
            VertexNode* vNode = new VertexNode();
            initQEF(vEdges, eIndex, leaf_index, vNode->qef);
            generateVertex(leaf_index, sampler->leaf_level, vNode->qef, vNode->v);
            vNode->computeError();
            vNode->collapsable = true; // a leaf node is always collapsable
            // assign vertex index to edges
            for (uint e = 0; e < eIndex; ++e) {
                leaf.edgeVertices[vEdges[e]] = leaf.vertices.size();
            }
            leaf.vertices.push_back(shared_ptr<VertexNode>(vNode));
            eIndex = 0;
            if (edge == -2)
                break;
        } else
            vEdges[eIndex++] = edge;
    }
}

void DualMarchingCubes::createOctreeNodes(DMCOctreeNode& parent, uint parent_size, const Index& parent_index) {
    uint child_size = parent_size/2;

    if (parent.level == sampler->leaf_level-1) {
        // create leaves
        for (int i = 0; i < 8; ++i) {
            Index child_index = parent_index + child_size*child_origin[i];
            unique_ptr<DMCOctreeLeaf> child(new DMCOctreeLeaf(parent.level+1));
            sampler->applySigns(*child, child_index);
            if (!child->isHomogeneous())
                createVertexNodes(*child, child_index);
            parent.children[i] = move(child);
        }
    } else {
        // create nodes
        for (int i = 0; i < 8; ++i) {
            Index child_index = parent_index + child_size*child_origin[i];
            unique_ptr<DMCOctreeNode> child(new DMCOctreeNode(parent.level+1));
            createOctreeNodes(*child, child_size, child_index);
            sampler->applySigns(*child, child_index);
            parent.children[i] = move(child);
        }
    }
    if (parent.isHomogeneous())
        parent.removeChildren();
}

void DualMarchingCubes::createOctree() {
    root = unique_ptr<DMCOctreeNode>(new DMCOctreeNode(0));
    createOctreeNodes(*root, sampler->res, Index(0));
}

void DualMarchingCubes::createVertexTree() {
    clusterCell(Index(0), root.get());
}

void DualMarchingCubes::collapse(float max_error) {
    updateCollapsableFlag(root.get(), max_error);
}

void DualMarchingCubes::createMesh(aligned_vector3f& positions, vector<uint>& indices) {
    cellProc(root.get(), positions, indices);
}


void DualMarchingCubes::dfs(DMCOctreeAction &action, bool processEmptyNodes) const {

    // empty cells for homogeneous nodes without children
    vector<DMCOctreeNode*> dummy(sampler->leaf_level+1, nullptr);
    if (processEmptyNodes) {
        for (int i = 0; i <= sampler->leaf_level; ++i)
            dummy[i] = new DMCOctreeNode(i);
    }

    // start with root
    stack<DMCOctreeCell*> nextNodes;
    stack<Index> nextIndices;
    nextNodes.push(root.get());
    nextIndices.push(Index(0));

    while (!nextNodes.empty()) {
        DMCOctreeCell* node = nextNodes.top();
        Index& index = nextIndices.top();

        action.handle(node, index);

        // append children
        if (node->hasChildren()) {
            for (int i = 0; i < 8; ++i) {
                nextNodes.push(node->child(i));
                nextIndices.push(index*2+child_origin[i]);
            }
        } else if (processEmptyNodes && node->level < sampler->leaf_level) { // homogeneous node
            for (int i = 0; i < 8; ++i) {
                nextNodes.push(dummy[node->level+1]);
                nextIndices.push(index*2+child_origin[i]);
            }
        }

        nextIndices.pop();
        nextNodes.pop();
    }

    if (processEmptyNodes) {
        // free dummies
        for (int i = 0; i <= sampler->leaf_level; ++i) {
            delete dummy[i];
            dummy[i] = nullptr;
        }
    }
}

void DualMarchingCubes::bfs(DMCOctreeAction& action, bool processEmptyNodes) const {

    // empty cells for homogeneous nodes without children
    vector<DMCOctreeNode*> dummy(sampler->leaf_level+1, nullptr);
    if (processEmptyNodes) {
        for (int i = 0; i <= sampler->leaf_level; ++i)
            dummy[i] = new DMCOctreeNode(i);
    }

    // start with root
    queue<DMCOctreeCell*> nextNodes;
    queue<Index> nextIndices;
    nextNodes.push(root.get());
    nextIndices.push(Index(0));

    while (!nextNodes.empty()) {
        DMCOctreeCell* node = nextNodes.front();
        Index& index = nextIndices.front();

        action.handle(node, index);

        // append children
        if (node->hasChildren()) {
            for (int i = 0; i < 8; ++i) {
                nextNodes.push(node->child(i));
                nextIndices.push(index*2+child_origin[i]);
            }
        } else if (processEmptyNodes && node->level < sampler->leaf_level) { // homogeneous node
            for (int i = 0; i < 8; ++i) {
                nextNodes.push(dummy[node->level+1]);
                nextIndices.push(index*2+child_origin[i]);
            }
        }

        nextIndices.pop();
        nextNodes.pop();
    }

    if (processEmptyNodes) {
        // free dummies
        for (int i = 0; i <= sampler->leaf_level; ++i) {
            delete dummy[i];
            dummy[i] = nullptr;
        }
    }

}

void DualMarchingCubes::initVector(vector_4_uint& v) const {
    v.resize(sampler->leaf_level+1);
    uint size = 1;
    for (uint i = 0; i <= sampler->leaf_level; ++i) {
        v[i] = vector_3_uint(size, vector_2_uint(size, vector<uint>(size, 0)));
        size *= 2;
    }
}

void DualMarchingCubes::vertices(aligned_vector3f &positions, aligned_vector3f &colors,
                                 vector_4_uint& offset, vector_4_uint& count) const { 
    initVector(offset);
    initVector(count);
    DMCVerticesBuilder builder(positions, colors, offset, count);
    bfs(builder, true);
}

void DualMarchingCubes::cells(aligned_vector3f &positions,
                               vector_4_uint& offset) const {
    initVector(offset);
    DMCCellBuilder builder(sampler->res, positions, offset);
    bfs(builder, true);
}


