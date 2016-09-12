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
    error = qef->evaluate(v);
}

bool compareSurfaceIndex(shared_ptr<VertexNode> v1, shared_ptr<VertexNode> v2) {
    return v1->surfaceIndex < v2->surfaceIndex;
}

DMCOctreeNode::DMCOctreeNode(uint8_t level) : DMCOctreeCell(level) {
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

DMCOctreeLeaf::DMCOctreeLeaf(uint8_t level) : DMCOctreeCell(level), signConfig(0) {
    for (uint i = 0; i < 12; ++i) {
        edgeVertices[i] = -1;
    }
    collapsed = true;
}

VertexNode* DMCOctreeLeaf::vertexAssignedTo(uint edgeIndex) const {
    int8_t vIndex = -1;
    vIndex = edgeVertices[edgeIndex];
    return vIndex < 0? nullptr : vertices[vIndex].get();
}

bool DMCOctreeLeaf::sign(uint i) const {
    return (signConfig & (1 << i)) > 0;
}

bool DMCOctreeLeaf::signChange(uint e) const {
    return sign(edge_corners[e][0]) != sign(edge_corners[e][1]);
}

bool DMCOctreeLeaf::frontface(uint e) const {
    return !sign(edge_corners[e][0]) && sign(edge_corners[e][1]);
}

bool DMCOctreeLeaf::backface(uint e) const {
    return sign(edge_corners[e][0]) && !sign(edge_corners[e][1]);
}

bool DMCOctreeLeaf::homogeneousSigns() const {
    return in() || out();
}

bool DMCOctreeLeaf::in() const {
    return signConfig == 255;
}

bool DMCOctreeLeaf::out() const {
    return signConfig == 0;
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

void DualMarchingCubes::assignSurface(const vector<shared_ptr<VertexNode>>& vertices, int from, int to) const {
    for (shared_ptr<VertexNode> v : vertices) {
        if (v->surfaceIndex == from) {
            v->surfaceIndex = to;
        }
    }
}

void DualMarchingCubes::clusterIndices(array<DMCOctreeCell* , 4> nodes, uint orientation, uint& maxSurfaceIndex,
                                    const vector<shared_ptr<VertexNode>>& vertices) {
    array<VertexNode*, 4> v{nullptr, nullptr, nullptr, nullptr};
    for (uint i = 0; i < 4; ++i) {
        uint e = edge_of_four_cells[orientation][i];
        v[i] = nodes[i]->vertexAssignedTo(e);
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

void DualMarchingCubes::clusterBoundaryIndices(array<DMCOctreeCell* , 2> nodes, uint orientation, uint face_orientation, uint plane, uint& maxSurfaceIndex,
                                    const vector<shared_ptr<VertexNode>>& vertices) {
    array<VertexNode*, 2> v{nullptr, nullptr};
    for (uint i = 0; i < 2; ++i) {
        uint e = edge_of_two_cells[face_orientation][plane][orientation][i];
        v[i] = nodes[i]->vertexAssignedTo(e);
        // folow parent pointers up the vertex tree
        while (!v[i]->parent.expired())
            v[i] = v[i]->parent.lock().get();
    }

    // cluster vertices connected by surfaces dual to this edge
    int surfaceIndex = -1;
    for (uint i = 0; i < 2; ++i) {
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

    for (uint i = 0; i < 2; ++i) {
        if (v[i]->surfaceIndex == -1)
            v[i]->surfaceIndex = surfaceIndex;
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

        uint e = edge_of_four_cells[orientation][0];
        bool s0 = nodes[0]->sign(edge_corners[e][0]);
        bool s1 = nodes[0]->sign(edge_corners[e][1]);
        if (!s0 && s1 || s0 && !s1)
            clusterIndices(nodes, orientation, maxSurfaceIndex, vertices);

    }

}

void DualMarchingCubes::clusterBoundaryEdge(array<DMCOctreeCell* , 2> nodes,  uint orientation, uint face_orientation, uint plane,
                                    uint& maxSurfaceIndex, const vector<shared_ptr<VertexNode>>& vertices) {

    if (nodes[0]->hasChildren() && nodes[1]->hasChildren()) {
        for (uint i = 0; i < 2; ++i) { // subdivided edge
            clusterBoundaryEdge(array<DMCOctreeCell*, 2>{
                            nodes[0]->child(boundary_edge_edge_mask[face_orientation][plane][orientation][i][0]),
                            nodes[1]->child(boundary_edge_edge_mask[face_orientation][plane][orientation][i][1])},
                            orientation, face_orientation, plane, maxSurfaceIndex, vertices);
        }

    } else if (!nodes[0]->hasChildren() && !nodes[1]->hasChildren()) {

        uint e = edge_of_two_cells[face_orientation][plane][orientation][0];
        bool s0 = nodes[0]->sign(edge_corners[e][0]);
        bool s1 = nodes[0]->sign(edge_corners[e][1]);
        if (!s0 && s1 || s0 && !s1)
            clusterBoundaryIndices(nodes, orientation, face_orientation, plane, maxSurfaceIndex, vertices);

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

void DualMarchingCubes::clusterBoundaryFace(DMCOctreeCell* node, uint orientation, uint plane,
                                            uint& maxSurfaceIndex, const vector<shared_ptr<VertexNode>>& vertices) {

    for (int i = 0; i < 4; ++i) { // 4 faces tiling coarse face
        // cell_edge_mask is equal to boundary_face_face_mask
        if (node->child(cell_edge_mask[orientation][plane][i])->hasChildren()) {
                clusterBoundaryFace(node->child(cell_edge_mask[orientation][plane][i]),
                                    orientation, plane, maxSurfaceIndex, vertices);
        }
    }

    for (uint i = 0; i < 4; ++i) { // 4 edges in coarse face
        // 2 child cells define 1 boundary edge
        clusterBoundaryEdge(array<DMCOctreeCell*, 2>{
                        node->child(boundary_face_edge_mask[orientation][plane][i][0]),
                        node->child(boundary_face_edge_mask[orientation][plane][i][1])},
                        face_edge_orientation[orientation][i], orientation, plane, maxSurfaceIndex, vertices);
    }


}

void DualMarchingCubes::clusterCell(const Index& cell_index, DMCOctreeCell* node) {
    if (node->hasChildren()) {
        // cluster children cells first
/*        for (uint i = 0; i < 8; ++i) {
            Index child_index = cell_index+sampler->nodeSize(node->level+1)*child_origin[i];
            clusterCell(child_index, node->child(i));
        }*/

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

        // cluster boundary faces
        for (uint i = 0; i < 3; ++i) { // x,y,z
            for (uint j = 0; j < 2; ++j) { //plane: left, right, bottom, top, back, front boundary face
                clusterBoundaryFace(node, i, j, maxSurfaceIndex, vertices);
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

        int last_index = -1;
        // the count of QEF masspoints per cluster/surface
        vector<uint> mCount;
        for (shared_ptr<VertexNode> v : vertices) {
            int index = v->surfaceIndex;
            if (index == -1) // stray vertex may be clustered in higher level
               node->strayVertices.push_back(v);
            else {
                if (index > last_index) { // new surface/cluster?
                    node->vertices.push_back(shared_ptr<VertexNode>(new VertexNode()));
                    mCount.push_back(0);
                    last_index = index;
                }
                QEF& qef = *(node->vertices.back()->qef);
                // add QEFs of connected vertices
                qef.add(*(v->qef));
                if (qef.dimension == v->qef->dimension) {
                    qef.m += v->qef->m;
                    mCount.back()++;
                } else if (qef.dimension < v->qef->dimension) {
                    qef.m = v->qef->m;
                    qef.dimension = v->qef->dimension;
                    mCount.back() = 1;
                }
                v->parent = node->vertices.back();      
                v->qef.reset(); // QEF is no longer needed, since the vertex is clustered
            }
        }


        // generate vertices
        for (uint i = 0; i < node->vertices.size(); ++i) {
            VertexNode* vNode = node->vertices[i].get();
            // average masspoint from QEFs of connected vertices with highest dimension
            vNode->qef->m /= (float) mCount[i];
            generateVertex(cell_index, node->level, *(vNode->qef), vNode->v);
            vNode->computeError();
        }

    }
}

bool DualMarchingCubes::updateCollapsableFlag(DMCOctreeCell *node, float max_error) {
    // reset vertex indices
    for (shared_ptr<VertexNode> vNode : node->vertices)
        vNode->vertexIndex = -1;

    if (node->hasChildren()) { // only update non-leaf cells
        node->collapsed = node->strayVertices.empty();
        for (shared_ptr<VertexNode> vNode : node->vertices) {
            if (vNode->error <= max_error)
                vNode->collapsable = true;
            else {
                vNode->collapsable = false;
                node->collapsed = false;
            }
        }
        for (int i = 0; i < 8; ++i)
            node->collapsed &= updateCollapsableFlag(node->child(i), max_error);
    }
    return node->collapsed;
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

void DualMarchingCubes::edgeProc(array<DMCOctreeCell* , 4> nodes, uint orientation, FaceType type,
                                 aligned_vector3f& positions, vector<uint>& indices) {

    array<VertexNode*, 4> vNodes{nullptr, nullptr, nullptr, nullptr};
    for (uint i = 0; i < 4; ++i) {

        // get vertices assigned to this edge
        vNodes[i] = nodes[i]->vertexAssignedTo(edge_of_four_cells[orientation][i]);
        // folow parent pointers up the vertex tree to the last vertex marked as collapsable
        while (!vNodes[i]->parent.expired() && vNodes[i]->parent.lock()->collapsable)
            vNodes[i] = vNodes[i]->parent.lock().get();
    }
    vector<VertexNode*> v = makeUnique(vNodes); // remove doubles (e.g vNodes have same ancestor)

    switch (type) {
        case FRONT_FACE:
        if (v.size() == 4) { // quad
            triangulate(v, true, orientation, positions, indices);
        } else if (v.size() == 3) { // triangle
            triangle(v, {0,1,2}, positions, indices);
        }
        break;
        case BACK_FACE:
        if (v.size() == 4) { // quad
            triangulate(v, false, orientation, positions, indices);
        } else if (v.size() == 3) { // triangle
            triangle(v, {0,2,1}, positions, indices);
        }
        break;
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

        uint e = edge_of_four_cells[orientation][0];
        bool s0 = nodes[0]->sign(edge_corners[e][0]);
        bool s1 = nodes[0]->sign(edge_corners[e][1]);
        if (!s0 && s1)
            edgeProc(nodes, orientation, FRONT_FACE, positions, indices);
        else if (s0 && !s1)
            edgeProc(nodes, orientation, BACK_FACE, positions, indices);
    }

}

void DualMarchingCubes::faceProc(array<DMCOctreeCell*, 2> nodes, uint orientation, aligned_vector3f& positions, vector<uint>& indices) {
    if (nodes[0]->collapsed && nodes[1]->collapsed)
        return;

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
    if (node->hasChildren() && !node->collapsed) {
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

bool DualMarchingCubes::inCell(Vector3d& pos, const Vector3d& cellOrigin, double size, double eps) const {
    return pos[0] <= size+cellOrigin[0]+eps && pos[1] <= size+cellOrigin[1]+eps && pos[2] <= size+cellOrigin[2]+eps &&
           pos[0] >= cellOrigin[0]-eps && pos[1] >= cellOrigin[1]-eps && pos[2] >= cellOrigin[2]-eps;
}

void DualMarchingCubes::generateVertex(const Index& cell_index, uint8_t level, QEF& qef, Vector3f& v) {
    Vector3d vQEF(0,0,0);
    qef.solve(truncation, vQEF); // relative to masscenter
    vQEF += qef.m.cast<double>();  // relative to cellGrid origin
    Vector3d cell_origin(cell_index.x,cell_index.y, cell_index.z);
    cell_origin /= res;
    double cell_size = cellSize(level);
    if (inCell(vQEF, cell_origin, cell_size, cell_size)) {
        v = vQEF.cast<float>();
    } else
        v = qef.m;

    //TODO better vertex placement
}

void DualMarchingCubes::addToQEF(const Index& edge, uint orientation, float d,
                                 const Vector3f& n, QEF& qef) const {
    Vector3f p = Vector3f(edge.x,edge.y,edge.z);
    p[orientation] += d;
    p /= res;
    qef.add(n, p);
    qef.m += p;
}

void DualMarchingCubes::initQEF(const int8_t frontEdges[], const int8_t backEdges[], uint frontCount,
                                uint backCount, const Index& cell_index, QEF& qef) const {
    Vector3f n;
    n.setZero(3);
    float d = 0.0;
    for (uint i = 0; i < frontCount; ++i) {
        int e = frontEdges[i];
        Index edge = cell_index + corner_delta[edge_corners[e][0]];
        uint orientation = edge_orientation[e];
        assert(sampler->frontEdgeInfo(orientation, edge, d, n));
        addToQEF(edge, orientation, d, n, qef);
    }
    for (uint i = 0; i < backCount; ++i) {
        int e = backEdges[i];
        Index edge = cell_index + corner_delta[edge_corners[e][0]];
        uint orientation = edge_orientation[e];
        assert(sampler->backEdgeInfo(orientation, edge, d, n));
        addToQEF(edge, orientation, d, n, qef);
    }
    qef.m /= frontCount + backCount;
}

void DualMarchingCubes::createVertexNodesFromEdges(DMCOctreeLeaf& leaf, const Index& leaf_index) {
/*    int edgeConfig = sampler->edgeConfig(leaf_index);
    int8_t frontEdges[8] = {-1,-1,-1,-1,-1,-1,-1,-1};
    int8_t backEdges[8] = {-1,-1,-1,-1,-1,-1,-1,-1};
    uint frontCount = 0;
    uint backCount = 0;
    bool inside = false; // the connected corners have an inside sign
    for (int i = 0; i < 16; ++i) {
        int corner = cornerTable[edgeConfig][i];
        if (corner < 0) {
            if (!inside) {
                VertexNode* vNode = new VertexNode();
                initQEF(frontEdges, backEdges, frontCount, backCount, leaf_index, vNode->qef);
                generateVertex(leaf_index, sampler->leaf_level, vNode->qef, vNode->v);
                vNode->computeError();
                vNode->collapsable = true; // a leaf node is always collapsable
                // assign vertex index to edges
                for (uint e = 0; e < frontCount; ++e)
                    leaf.frontEdgeVertices[frontEdges[e]] = leaf.vertices.size();
                for (uint e = 0; e < backCount; ++e)
                    leaf.backEdgeVertices[backEdges[e]] = leaf.vertices.size();
                leaf.vertices.push_back(shared_ptr<VertexNode>(vNode));
            }
            frontCount = 0;
            backCount = 0;
            inside = false;
            if (corner == -2)
                break;
        } else if (!inside) {
            if (leaf.sign(corner)) {// inside (only connect over outside corners)
                inside = true;
                continue;
            }
            for (int j = 0; j < 3; ++j) { // 3 adjacent edges per corner
                int e = corner_edges[corner][j];
                uint frontCorner = edge_corners[e][0]; // corner associated with front cut of edge
                uint orientation = edge_orientation[e];
                Index edge = leaf_index + corner_delta[frontCorner];
                // has asociated edge cut?
                if (corner == frontCorner && sampler->hasFrontCut(orientation,edge))
                    frontEdges[frontCount++] = e;
                else if (corner != frontCorner && sampler->hasBackCut(orientation,edge))
                    backEdges[backCount++] = e;

            }
        }
    }*/
}

void DualMarchingCubes::initQEF(const vector<int>& edges, const Index& cell_index, QEF& qef) const {
    Vector3f n;
    n.setZero(3);
    float d = 0.0;
    for (int e : edges) {
        int eIndex = e > 0? e-1 : -e-1;
        Index edge = cell_index + corner_delta[edge_corners[eIndex][0]];
        uint orientation = edge_orientation[eIndex];
        if (e > 0)
            sampler->frontEdgeInfo(orientation,edge,d,n);
        else
            sampler->backEdgeInfo(orientation,edge,d,n);
        addToQEF(edge, orientation, d, n, qef);
    }
    qef.m /= edges.size();
}

void DualMarchingCubes::createVertexNodesFromNormalGroups(DMCOctreeLeaf &leaf, const Index &leaf_index) {
/*    bool frontCuts[12] = {0};
    bool backCuts[12] = {0};
    int cutCount = 0;
    for (int e = 0; e < 12; ++e) {
        Index from = leaf_index + corner_delta[edge_corners[e][0]];
        Index to = leaf_index + corner_delta[edge_corners[e][1]];
        if (!sampler->sign(from) && sampler->sign(to))
            frontCuts[e] = sampler->hasFrontCut(edge_orientation[e], from);
        else if (!sampler->sign(to) && sampler->sign(from))
            backCuts[e] = sampler->hasBackCut(edge_orientation[e], from);
        else if (sampler->isComplex(edge_orientation[e], from)) {
            frontCuts[e] = true;
            backCuts[e] = true;
        }
        cutCount += frontCuts[e]? 1 : 0;
        cutCount += backCuts[e]? 1 : 0;
    }

    int edgeConfig = 0;
    for (int i = 0; i < 12; ++i) {
        if (frontCuts[i] || backCuts[i]) {
            edgeConfig |= 1 << i;
        }
    }

    while (edgeConfig != 0) {

        // retrieve possible surface components
        vector<vector<int>> components = vector<vector<int>>(1, vector<int>());

        for (int i = 0; i < 16; ++i) {
            int corner = cornerTable[edgeConfig][i];
            if (corner < 0) {
                if (corner == -2)
                    break;
                components.push_back(vector<int>());
            } else {
                for (int j = 0; j < 3; ++j) { // 3 adjacent edges per corner
                    int e = corner_edges[corner][j];
                    uint frontCorner = edge_corners[e][0]; // corner associated with front cut of edge
                    if (corner == frontCorner) {
                        if (frontCuts[e])
                            components.back().push_back(e+1);
                        else if (backCuts[e])
                            components.back().push_back(-e-1);
                    } else {
                        if (backCuts[e])
                            components.back().push_back(-e-1);
                        else if (frontCuts[e])
                            components.back().push_back(e+1);
                    }
                }
            }
        }

        // calculate best next component
        Vector3f n(0,0,0);
        float d; // not needed
        float bestDeviation = -2.0; // the best max normal deviation of all components
        uint best = 0; // the index of the best component
        for (uint i = 0; i < components.size(); i++) {
            float deviation = 1.0; //the max normal deviation (dot product [-1,1], higher is better) of this component
            vector<Vector3f> normals;
            for (int e : components[i]) { // iterate over edge cuts of this component
                int eIndex = e > 0? e-1 : -e-1;
                Index edge = leaf_index + corner_delta[edge_corners[eIndex][0]];
                if (e > 0) {
                    if (!sampler->frontEdgeInfo(edge_orientation[eIndex],edge,d,n))
                        std::cout << leaf_index.x << " " << leaf_index.y  <<  " "  << leaf_index.z << std::endl;
                } else {
                    if(!sampler->backEdgeInfo(edge_orientation[eIndex],edge,d,n)) {
                        std::cout << leaf_index.x << " " << leaf_index.y  <<  " "  << leaf_index.z << std::endl;
                    }
                }
                for (Vector3f normal : normals) {
                    float dev = normal.transpose()*n;
                    deviation = std::min(deviation, dev);
                }
                normals.push_back(n);

            }
            if (bestDeviation + 0.01 < deviation || (bestDeviation < deviation && components[best].size() < cutCount)
                    || (bestDeviation < deviation + 0.01 && components[i].size() == cutCount)) {
                best = i;
                bestDeviation = deviation;
            }
        }

        // create vertex node for the best surface component
        VertexNode* vNode = new VertexNode();
        initQEF(components[best], leaf_index, vNode->qef);
        generateVertex(leaf_index, sampler->leaf_level, vNode->qef, vNode->v);
        vNode->computeError();
        vNode->collapsable = true; // a leaf node is always collapsable
        // assign vertex index to edges
        for (int e : components[best]) {
            if (e > 0) {
                leaf.frontEdgeVertices[e-1] = leaf.vertices.size();
                frontCuts[e-1] = false; // remove the edge cut
                cutCount--;
            } else {
                leaf.backEdgeVertices[-e-1] = leaf.vertices.size();
                backCuts[-e-1] = false; // remove the edge cut
                cutCount--;
            }
        }
        leaf.vertices.push_back(shared_ptr<VertexNode>(vNode));

        // update edge config of cell
        edgeConfig = 0;
        for (int e = 0; e < 12; ++e) {
            if (frontCuts[e] || backCuts[e])
                edgeConfig |= 1 << e;
        }
    }

*/
}

bool DualMarchingCubes::initQEF(const int8_t edges[], uint count, const Index& cell_index, QEF& qef) const {
    //if (cell_index.x == 247 && cell_index.y == 56 && cell_index.z == 250)
    //    std::cout << "247,56,250" << std::endl;
    uint cuts = 0;
    for (uint i = 0; i < count; ++i) {
        int e = edges[i];
        Index frontCorner = cell_index + corner_delta[edge_corners[e][0]];
        Index backCorner = cell_index + corner_delta[edge_corners[e][1]];

        Vector3f n;
        float d;
        uint orientation = edge_orientation[e];
        bool s0 = signSampler->sign(frontCorner);
        bool s1 = signSampler->sign(backCorner);
        //bool fcut = sampler->hasFrontCut(orientation, frontCorner);
        //bool bcut = sampler->hasBackCut(orientation, frontCorner);

        if ((!s0 && s1 && sampler->frontEdgeInfo(orientation, frontCorner, d, n))
                || (s0 && !s1 && sampler->backEdgeInfo(orientation, frontCorner, d, n))) {
            addToQEF(frontCorner, orientation, d, n, qef);
            cuts++;
        }
    }
    if (cuts == 0) {
        //std::cout << "0 cuts " << cell_index.x << ", " << cell_index.y << ", " << cell_index.z << std::endl;
        return false;
    } else {
        qef.m /= cuts;
        return true;
    }
}

void DualMarchingCubes::createVertexNodes(DMCOctreeLeaf* leaf, const Index &leaf_index) {

    //if (sampler->hasComplexEdge(leaf_index))
        //createVertexNodesFromNormalGroups(leaf, leaf_index);
        //createVertexNodesFromEdges(leaf, leaf_index);
    //else
    uint vCount = vertexCount[leaf->signConfig];
    if (vCount == 0)
        return;
    uint count = 0;
    int8_t vEdges[8] = {-1,-1,-1,-1,-1,-1,-1,-1};
    for (uint i = 0; i < 16; ++i) {
        int edge = edgeTable[leaf->signConfig][i];
        if (edge < 0) {
            VertexNode* vNode = new VertexNode();
            if (initQEF(vEdges, count, leaf_index, *(vNode->qef))) {
                generateVertex(leaf_index, leaf_level, *(vNode->qef), vNode->v);
            } else {
                vNode->v = Vector3f(leaf_index.x+0.5,leaf_index.y+0.5, leaf_index.z+0.5);
                vNode->v /= res;
            }

            vNode->computeError();
            vNode->collapsable = true; // a leaf node is always collapsable
            // assign vertex index to edges
            for (uint e = 0; e < count; ++e) {
                leaf->edgeVertices[vEdges[e]] = leaf->vertices.size();
            }
            leaf->vertices.push_back(shared_ptr<VertexNode>(vNode));

            count = 0;
            if (edge == -2)
                break;
        } else
            vEdges[count++] = edge;
    }

}

bool DualMarchingCubes::createOctreeNodes(DMCOctreeNode* parent, uint parent_size, const Index& parent_index) {
    uint child_size = parent_size/2;
    bool homogeneous = true; // true if every child cell has an inside sign config or no edge cut
    if (parent->level == leaf_level-1) {
        // create leaves
        for (int i = 0; i < 8; ++i) {
            Index child_index = parent_index + child_size*child_origin[i];
            unique_ptr<DMCOctreeLeaf> child(new DMCOctreeLeaf(parent->level+1));
            child->signConfig = signSampler->signConfig(child_index);
            if (!child->homogeneousSigns()) {
                createVertexNodes(child.get(), child_index);
                homogeneous = false;
            }
            parent->children[i] = move(child);
        }
    } else {
        // create nodes
        for (int i = 0; i < 8; ++i) {
            Index child_index = parent_index + child_size*child_origin[i];
            unique_ptr<DMCOctreeNode> child(new DMCOctreeNode(parent->level+1));
            homogeneous &= createOctreeNodes(child.get(), child_size, child_index);
            parent->children[i] = move(child);
        }
    }
    if (homogeneous)
        parent->removeChildren();

    clusterCell(parent_index, parent);


    return homogeneous;
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
    vector<DMCOctreeNode*> dummy(leaf_level+1, nullptr);
    if (processEmptyNodes) {
        for (uint i = 0; i <= leaf_level; ++i)
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
        } else if (processEmptyNodes && node->level < leaf_level) { // homogeneous node
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
        for (uint i = 0; i <= leaf_level; ++i) {
            delete dummy[i];
            dummy[i] = nullptr;
        }
    }

}

void DualMarchingCubes::initVector(vector_4_uint& v) const {
    v.resize(leaf_level+1);
    uint size = 1;
    for (uint i = 0; i <= leaf_level; ++i) {
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

bool DualMarchingCubes::initShaderProgram(const char *vname, const char *fname, QGLShaderProgram& program) {
    setlocale(LC_NUMERIC, "C");
    // shader
    if (!program.addShaderFromSourceFile(QGLShader::Vertex, vname))
        return false;

    if (!program.addShaderFromSourceFile(QGLShader::Fragment, fname))
        return false;

    if (!program.link())
        return false;

    if (!program.bind())
        return false;

    return true;
}

void DualMarchingCubes::projectionX(const Index& node_index, QMatrix4x4& projection) {
    float subGridSize = cell_size*workRes;
    float subVoxelGridSize = subGridSize + cell_size;
    float left = cell_size*node_index.z - voxelGridRadius;
    float right = left + subVoxelGridSize;
    float bottom = cell_size*node_index.y - voxelGridRadius;
    float top = bottom + subVoxelGridSize;
    // do not shift near and far plane here (small overlap for border points)
    float n = -cell_size*node_index.x + voxelGridRadius;
    float f = n-subVoxelGridSize;
    projection.ortho(left, right, bottom, top, -n, -f);
}

void DualMarchingCubes::projectionY(const Index& node_index, QMatrix4x4& projection) {
    float subGridSize = cell_size*workRes;
    float subVoxelGridSize = subGridSize + cell_size;
    float left = cell_size*node_index.x - voxelGridRadius;
    float right = left + subVoxelGridSize;
    float bottom = cell_size*node_index.z - voxelGridRadius;
    float top = bottom + subVoxelGridSize;
    // do not shift near and far plane here (small overlap for border points)
    float n = -cell_size*node_index.y + voxelGridRadius;
    float f = n-subVoxelGridSize;
    projection.setToIdentity();
    projection.ortho(left, right, bottom, top, -n, -f);
}

void DualMarchingCubes::projectionZ(const Index& node_index, QMatrix4x4& projection) {
    float subGridSize = cell_size*workRes;
    float subVoxelGridSize = subGridSize + cell_size;
    float left = -cell_size*(node_index.x+workRes+1) + voxelGridRadius;
    float right = left + subVoxelGridSize;
    float bottom = cell_size*node_index.y - voxelGridRadius;
    float top = bottom + subVoxelGridSize;
    // do not shift near and far plane here (small overlap for border points)
    float n = -cell_size*node_index.z + voxelGridRadius;
    float f = n-subVoxelGridSize;
    projection.setToIdentity();
    projection.ortho(left, right, bottom, top, -n, -f);
}

void DualMarchingCubes::conturing(uint currentRes, DMCOctreeNode* node, const Index& node_index) {
    if (workRes < currentRes) {
        for (int i = 0; i < 8; ++i) {
            uint newRes = currentRes/2;
            Index child_index = node_index + newRes*child_origin[i];
            unique_ptr<DMCOctreeNode> child(new DMCOctreeNode(node->level+1));

            conturing(newRes, child.get(), child_index);
            node->children[i] = move(child);
        }
        clusterCell(node_index, node);
    } else {

        std::cout << "scan hermite data (" << currentRes << ")" << std::endl;
        CompressedHermiteScanner hermiteScanner(currentRes);
        hermiteScanner.program = &programHermiteScan;

        projectionX(node_index, hermiteScanner.projection);
        hermiteScanner.scan(scene, X);
        projectionY(node_index, hermiteScanner.projection);
        hermiteScanner.scan(scene, Y);
        projectionZ(node_index, hermiteScanner.projection);
        hermiteScanner.scan(scene, Z);
        sampler.reset();
        sampler = unique_ptr<HermiteDataSampler>(new CompressedHermiteSampler(hermiteScanner.data, node_index));
        std::cout << "create octree (" << currentRes << ")" << std::endl;
        createOctreeNodes(node, currentRes, node_index);
    }

}

bool DualMarchingCubes::conturing(RenderStrategy* scene, float voxelGridRadius, uint resolution, uint workResolution) {
    initializeOpenGLFunctions();
    if(!scene->initShaders(programEdgeScan, programHermiteScan))
        return false;

    this->scene = scene;
    this->res = nextPowerOf2(resolution);
    this->workRes = min(nextPowerOf2(workResolution), res);
    this->voxelGridRadius = voxelGridRadius;
    this->cell_size = 2*voxelGridRadius/(resolution+1);
    this->leaf_level = i_log2(res);

    std::cout << "scan edge intersections (" << resolution << ")" << std::endl;
    CompressedEdgeScanner edgeScanner(resolution);
    edgeScanner.program = &programEdgeScan;

    //shift near and far plane about a half voxel cell
    //because rays are casted from the pixel center
    edgeScanner.projection.ortho(-voxelGridRadius, voxelGridRadius,
                                 -voxelGridRadius, voxelGridRadius,
                                 -voxelGridRadius + 0.5*cell_size,
                                 voxelGridRadius - 0.5*cell_size);

    edgeScanner.scan(scene, X);
    edgeScanner.scan(scene, Y);
    edgeScanner.scan(scene, Z);
    std::cout << "create sign sampler (flood fill)" << std::endl;
    signSampler = unique_ptr<SignSampler>(new CompressedSignSampler(edgeScanner.data.get()));

    root = unique_ptr<DMCOctreeNode>(new DMCOctreeNode(0));
    conturing(res, root.get(), Index(0));
    signSampler.reset();
    sampler.reset();
/*
    CompressedHermiteScanner hermiteScanner(workResolution);
    hermiteScanner.program = &programHermiteScan;
    Index node_index(0,0,0);
    projectionX(node_index, hermiteScanner.projection);
    hermiteScanner.scan(scene, X);
    projectionY(node_index, hermiteScanner.projection);
    hermiteScanner.scan(scene, Y);
    projectionZ(node_index, hermiteScanner.projection);
    hermiteScanner.scan(scene, Z);
    sampler = unique_ptr<HermiteDataSampler>(new CompressedHermiteSampler(hermiteScanner.data));
*/
/*
    uint size = resolution +1;
    float o = -voxelGridRadius+voxelGridRadius/size;
    Vector3f origin(o,o,o);
    float cellSize = 2*voxelGridRadius/size;


    positions.clear();

    for (uint x = 0; x < size; ++x) {
         for (uint y = 0; y < size; ++y) {
             for (uint z = 0; z < resolution; ++z) {


                 positions.push_back(origin+cellSize*Vector3f(x,y,z));
                 positions.push_back(origin+cellSize*Vector3f(x+1,y,z));

                 positions.push_back(origin+cellSize*Vector3f(x,y,z));
                 positions.push_back(origin+cellSize*Vector3f(x,y+1,z));

                 positions.push_back(origin+cellSize*Vector3f(x,y,z));
                 positions.push_back(origin+cellSize*Vector3f(x,y,z+1));

                 if (edgeScanner.data->cut(0, x,y,z)) {
                     colors.push_back(Color::GREEN);
                     colors.push_back(Color::GREEN);
                 } else {
                     colors.push_back(Color::WHITE);
                     colors.push_back(Color::WHITE);
                 }
                 if (edgeScanner.data->cut(1, x,y,z)) {
                     colors.push_back(Color::GREEN);
                     colors.push_back(Color::GREEN);
                 } else {
                     colors.push_back(Color::WHITE);
                     colors.push_back(Color::WHITE);
                 }
                 if (edgeScanner.data->cut(2, x,y,z)) {
                     colors.push_back(Color::GREEN);
                     colors.push_back(Color::GREEN);
                 } else {
                     colors.push_back(Color::WHITE);
                     colors.push_back(Color::WHITE);
                 }
             }
         }
     }*/
}


