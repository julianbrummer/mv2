#include "dmc.h"
#include <iostream>
#include <queue>
#include <stack>
#include <algorithm>

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

void DMCOctreeNode::setChild(uint i, DMCOctreeCell* child) {
    children[i] = unique_ptr<DMCOctreeCell>(child);
}

void DMCOctreeNode::removeChildren() {
    for (int i = 0; i < 8; ++i) {
        children[i] = nullptr;
    }
}

DMCOctreeLeaf::DMCOctreeLeaf(uint8_t level) : DMCOctreeCell(level), signConfig(0) {
    for (uint i = 0; i < 12; ++i) {
        frontEdgeVertices[i] = -1;
        backEdgeVertices[i] = -1;
    }
    collapsed = true;
}

VertexNode* DMCOctreeLeaf::vertexAssignedTo(uint edgeIndex, Orientation type) const {
    int8_t vIndex = -1;
    switch (type) {
        case FRONT:
            vIndex = frontEdgeVertices[edgeIndex]; break;
        case BACK:
            vIndex = backEdgeVertices[edgeIndex]; break;
    }
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

void DMCOctreeLeaf::flipSign(uint i) {
    signConfig ^= (1 << i);
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

void DualMarchingCubes::clusterIndices(array<DMCOctreeCell* , 4> nodes, Direction dir, Orientation orientation, uint& maxSurfaceIndex,
                                    const vector<shared_ptr<VertexNode>>& vertices) {
    array<VertexNode*, 4> v{nullptr, nullptr, nullptr, nullptr};
    for (uint i = 0; i < 4; ++i) {
        uint e = edge_of_four_cells[dir][i];
        v[i] = nodes[i]->vertexAssignedTo(e, orientation);
        if (!v[i]) // this may happen since the method is also called for O----O edges
            return;
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

void DualMarchingCubes::clusterBoundaryIndices(array<DMCOctreeCell* , 2> nodes, Direction dir, Orientation orientation,
                                               uint face_orientation, uint plane, uint& maxSurfaceIndex,
                                               const vector<shared_ptr<VertexNode>>& vertices) {
    array<VertexNode*, 2> v{nullptr, nullptr};
    for (uint i = 0; i < 2; ++i) {
        uint e = edge_of_two_cells[face_orientation][plane][dir][i];
        v[i] = nodes[i]->vertexAssignedTo(e, orientation);
        if (!v[i]) // this may happen since the method is also called for O----O edges
            return;
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

void DualMarchingCubes::clusterEdge(array<DMCOctreeCell* , 4> nodes, Direction dir,
                                    uint& maxSurfaceIndex, const vector<shared_ptr<VertexNode>>& vertices) {

    if (nodes[0]->hasChildren() && nodes[1]->hasChildren() && nodes[2]->hasChildren() && nodes[3]->hasChildren()) {
        for (uint i = 0; i < 2; ++i) { // subdivided edge
            clusterEdge(array<DMCOctreeCell*, 4>{
                            nodes[0]->child(edge_edge_mask[dir][i][0]),
                            nodes[1]->child(edge_edge_mask[dir][i][1]),
                            nodes[2]->child(edge_edge_mask[dir][i][2]),
                            nodes[3]->child(edge_edge_mask[dir][i][3])},
                        dir, maxSurfaceIndex, vertices);
        }

    } else if (!nodes[0]->hasChildren() && !nodes[1]->hasChildren() && !nodes[2]->hasChildren() && !nodes[3]->hasChildren()) {

        uint e = edge_of_four_cells[dir][0];
        bool s0 = nodes[0]->sign(edge_corners[e][0]);
        bool s1 = nodes[0]->sign(edge_corners[e][1]);
        if (!s0 && s1)
            clusterIndices(nodes, dir, FRONT, maxSurfaceIndex, vertices);
        else if (s0 && !s1)
            clusterIndices(nodes, dir, BACK, maxSurfaceIndex, vertices);
        else if (!s0 && !s1) {
            clusterIndices(nodes, dir, FRONT, maxSurfaceIndex, vertices);
            clusterIndices(nodes, dir, BACK, maxSurfaceIndex, vertices);
        }
    }

}

void DualMarchingCubes::clusterBoundaryEdge(array<DMCOctreeCell* , 2> nodes, Direction dir, uint face_orientation, uint plane,
                                    uint& maxSurfaceIndex, const vector<shared_ptr<VertexNode>>& vertices) {

    if (nodes[0]->hasChildren() && nodes[1]->hasChildren()) {
        for (uint i = 0; i < 2; ++i) { // subdivided edge
            clusterBoundaryEdge(array<DMCOctreeCell*, 2>{
                            nodes[0]->child(boundary_edge_edge_mask[face_orientation][plane][dir][i][0]),
                            nodes[1]->child(boundary_edge_edge_mask[face_orientation][plane][dir][i][1])},
                            dir, face_orientation, plane, maxSurfaceIndex, vertices);
        }

    } else if (!nodes[0]->hasChildren() && !nodes[1]->hasChildren()) {

        uint e = edge_of_two_cells[face_orientation][plane][dir][0];
        bool s0 = nodes[0]->sign(edge_corners[e][0]);
        bool s1 = nodes[0]->sign(edge_corners[e][1]);
        if (!s0 && s1)
            clusterBoundaryIndices(nodes, dir, FRONT, face_orientation, plane, maxSurfaceIndex, vertices);
        else if (s0 && !s1)
            clusterBoundaryIndices(nodes, dir, BACK, face_orientation, plane, maxSurfaceIndex, vertices);
        else if (!s0 && !s1) {
            clusterBoundaryIndices(nodes, dir, FRONT, face_orientation, plane, maxSurfaceIndex, vertices);
            clusterBoundaryIndices(nodes, dir, BACK, face_orientation, plane, maxSurfaceIndex, vertices);
        }
    }

}

void DualMarchingCubes::clusterFace(array<DMCOctreeCell*, 2> nodes, Direction dir,
                                    uint& maxSurfaceIndex, const vector<shared_ptr<VertexNode>>& vertices) {

    for (int i = 0; i < 4; ++i) { // 4 faces tiling coarse face
        // 2 child cells define 1 face
        if (nodes[0]->child(face_face_mask[dir][i][0])->hasChildren()
                    && nodes[1]->child(face_face_mask[dir][i][1])->hasChildren()) {
                clusterFace(array<DMCOctreeCell*, 2>{
                                nodes[0]->child(face_face_mask[dir][i][0]),
                                nodes[1]->child(face_face_mask[dir][i][1])},
                            dir, maxSurfaceIndex, vertices);
        }
    }


    for (uint i = 0; i < 4; ++i) { // 4 edges in coarse face
        // 4 child cells define 1 edge
        clusterEdge(array<DMCOctreeCell*, 4>{
                        nodes[face_edge_mask[dir][i][0][0]]->child(face_edge_mask[dir][i][0][1]),
                        nodes[face_edge_mask[dir][i][1][0]]->child(face_edge_mask[dir][i][1][1]),
                        nodes[face_edge_mask[dir][i][2][0]]->child(face_edge_mask[dir][i][2][1]),
                        nodes[face_edge_mask[dir][i][3][0]]->child(face_edge_mask[dir][i][3][1])},
                    face_edge_dir[dir][i], maxSurfaceIndex, vertices);
    }
}

void DualMarchingCubes::clusterBoundaryFace(DMCOctreeCell* node, Direction dir, uint plane,
                                            uint& maxSurfaceIndex, const vector<shared_ptr<VertexNode>>& vertices) {

    for (int i = 0; i < 4; ++i) { // 4 faces tiling coarse face
        // cell_edge_mask is equal to boundary_face_face_mask
        if (node->child(cell_edge_mask[dir][plane][i])->hasChildren()) {
                clusterBoundaryFace(node->child(cell_edge_mask[dir][plane][i]),
                                    dir, plane, maxSurfaceIndex, vertices);
        }
    }

    for (uint i = 0; i < 4; ++i) { // 4 edges in coarse face
        // 2 child cells define 1 boundary edge
        clusterBoundaryEdge(array<DMCOctreeCell*, 2>{
                        node->child(boundary_face_edge_mask[dir][plane][i][0]),
                        node->child(boundary_face_edge_mask[dir][plane][i][1])},
                        face_edge_dir[dir][i], dir, plane, maxSurfaceIndex, vertices);
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
                                Direction(i), maxSurfaceIndex, vertices);
                }
            }
        }

        // cluster boundary faces
        for (uint i = 0; i < 3; ++i) { // x,y,z
            for (uint j = 0; j < 2; ++j) { //plane: left, right, bottom, top, back, front boundary face
                clusterBoundaryFace(node, Direction(i), j, maxSurfaceIndex, vertices);
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
                            Direction(i), maxSurfaceIndex, vertices);
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
            componentStrategy->vertexStrategy->generateVertex(cell_index, node->level, res, *(vNode->qef), vNode->v);
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

void DualMarchingCubes::triangulate(const vector<VertexNode*> v, bool front_face, const Direction dir, aligned_vector3f& positions, vector<uint>& indices) {
    Vector3f vDir(0,0,0);
    vDir[dir] = front_face? 1 : -1;
    float d0_2 = (v[0]->v - v[2]->v).squaredNorm();
    float d1_3 = (v[1]->v - v[3]->v).squaredNorm();
    if (front_face) {
        if ((d0_2 < d1_3 && checkNormal(v, {1,0,2,0, 2,0,3,0}, vDir)) ||
            (d0_2 >= d1_3 && !checkNormal(v, {1,0,3,0, 2,1,3,1}, vDir))) {
            triangle(v, {0,1,2}, positions, indices);
            triangle(v, {0,2,3}, positions, indices);
        } else {
            triangle(v, {0,1,3}, positions, indices);
            triangle(v, {1,2,3}, positions, indices);
        }
    } else {
        if ((d0_2 < d1_3 && checkNormal(v, {2,0,1,0, 3,0,2,0}, vDir)) ||
            (d0_2 >= d1_3 && !checkNormal(v, {3,0,1,0, 3,1,2,1}, vDir))) {
            triangle(v, {0,2,1}, positions, indices);
            triangle(v, {0,3,2}, positions, indices);
        } else {
            triangle(v, {0,3,1}, positions, indices);
            triangle(v, {1,3,2}, positions, indices);
        }
    }
}

void DualMarchingCubes::edgeProc(array<DMCOctreeCell* , 4> nodes, Direction dir, Orientation orientation,
                                 aligned_vector3f& positions, vector<uint>& indices) {

    array<VertexNode*, 4> vNodes{nullptr, nullptr, nullptr, nullptr};
    for (uint i = 0; i < 4; ++i) {
        // get vertices assigned to this edge
        vNodes[i] = nodes[i]->vertexAssignedTo(edge_of_four_cells[dir][i], orientation);
        if (!vNodes[i]) // this may happen since the method is also called for O----O edges
            return;
        // folow parent pointers up the vertex tree to the last vertex marked as collapsable
        while (!vNodes[i]->parent.expired() && vNodes[i]->parent.lock()->collapsable)
            vNodes[i] = vNodes[i]->parent.lock().get();
    }
    vector<VertexNode*> v = makeUnique(vNodes); // remove doubles (e.g vNodes have same ancestor)

    switch (orientation) {
        case FRONT:
        if (v.size() == 4) { // quad
            triangulate(v, true, dir, positions, indices);
        } else if (v.size() == 3) { // triangle
            triangle(v, {0,1,2}, positions, indices);
        }
        break;
        case BACK:
        if (v.size() == 4) { // quad
            triangulate(v, false, dir, positions, indices);
        } else if (v.size() == 3) { // triangle
            triangle(v, {0,2,1}, positions, indices);
        }
        break;
    }
}

void DualMarchingCubes::edgeProc(array<DMCOctreeCell* , 4> nodes, Direction dir, aligned_vector3f& positions, vector<uint>& indices) {

    if (nodes[0]->hasChildren() && nodes[1]->hasChildren() && nodes[2]->hasChildren() && nodes[3]->hasChildren()) {
        for (uint i = 0; i < 2; ++i) { // subdivided edge
            edgeProc(array<DMCOctreeCell*, 4>{
                            nodes[0]->child(edge_edge_mask[dir][i][0]),
                            nodes[1]->child(edge_edge_mask[dir][i][1]),
                            nodes[2]->child(edge_edge_mask[dir][i][2]),
                            nodes[3]->child(edge_edge_mask[dir][i][3])},
                        dir, positions, indices);
        }

    } else if (!nodes[0]->hasChildren() && !nodes[1]->hasChildren() && !nodes[2]->hasChildren() && !nodes[3]->hasChildren()) {

        uint e = edge_of_four_cells[dir][0];
        bool s0 = nodes[0]->sign(edge_corners[e][0]);
        bool s1 = nodes[0]->sign(edge_corners[e][1]);

        if (!s0 && s1)
            edgeProc(nodes, dir, FRONT, positions, indices);
        else if (s0 && !s1)
            edgeProc(nodes, dir, BACK, positions, indices);
        else if (!s0 && !s1) {
            edgeProc(nodes, dir, FRONT, positions, indices);
            edgeProc(nodes, dir, BACK, positions, indices);
        }
    }

}

void DualMarchingCubes::faceProc(array<DMCOctreeCell*, 2> nodes, Direction dir, aligned_vector3f& positions, vector<uint>& indices) {
    if (nodes[0]->collapsed && nodes[1]->collapsed)
        return;

    for (uint i = 0; i < 4; ++i) { // 4 faces tiling coarse face
        // 2 child cells define 1 face
        if (nodes[0]->child(face_face_mask[dir][i][0])->hasChildren()
                    && nodes[1]->child(face_face_mask[dir][i][1])->hasChildren()) {
                faceProc(array<DMCOctreeCell*, 2>{
                                nodes[0]->child(face_face_mask[dir][i][0]),
                                nodes[1]->child(face_face_mask[dir][i][1])},
                            dir, positions, indices);
        }
    }


    for (uint i = 0; i < 4; ++i) { // 4 edges in coarse face
        // 4 child cells define 1 edge
        edgeProc(array<DMCOctreeCell*, 4>{
                        nodes[face_edge_mask[dir][i][0][0]]->child(face_edge_mask[dir][i][0][1]),
                        nodes[face_edge_mask[dir][i][1][0]]->child(face_edge_mask[dir][i][1][1]),
                        nodes[face_edge_mask[dir][i][2][0]]->child(face_edge_mask[dir][i][2][1]),
                        nodes[face_edge_mask[dir][i][3][0]]->child(face_edge_mask[dir][i][3][1])},
                    face_edge_dir[dir][i], positions, indices);
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
                                Direction(i), positions, indices);
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
                            Direction(i), positions, indices);
            }
        }
    }

}

bool MapToQEFMinimum::inCell(Vector3d& pos, const Vector3d& cellOrigin, double size, double eps) const {
    return pos[0] <= size+cellOrigin[0]+eps && pos[1] <= size+cellOrigin[1]+eps && pos[2] <= size+cellOrigin[2]+eps &&
           pos[0] >= cellOrigin[0]-eps && pos[1] >= cellOrigin[1]-eps && pos[2] >= cellOrigin[2]-eps;
}

void MapToQEFMinimum::generateVertex(const Index& cell_index, uint8_t level, uint res, QEF& qef, Vector3f& v) {
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

void SurfaceComponentStrategy::addToQEF(const Index& edge, Direction dir, float d,
                                 const Vector3f& n, QEF& qef) const {
    Vector3f p = Vector3f(edge.x,edge.y,edge.z);
    p[dir] += d;
    p /= res;
    qef.add(n, p);
    qef.m += p;
}

void MCStrategy::initQEF(const int8_t edges[], uint count, DMCOctreeLeaf* leaf, const Index& cell_index, QEF& qef) const {
    uint cuts = 0;
    for (uint i = 0; i < count; ++i) {
        int e = edges[i];
        const Index frontCorner = cell_index + corner_delta[edge_corners[e][0]];

        Vector3f n;
        float d;
        Direction dir = edge_dir[e];
        bool s0 = leaf->sign(edge_corners[e][0]);
        bool s1 = leaf->sign(edge_corners[e][1]);
        if ((!s0 && s1 && sampler->edgeInfo(dir, FRONT, frontCorner, d, n))
                || (s0 && !s1 && sampler->edgeInfo(dir, BACK, frontCorner, d, n))) {
            addToQEF(frontCorner, dir, d, n, qef);
            cuts++;
        }
    }
    if (cuts == 0)
        std::cout << "0 cuts" << std::endl;
    qef.m /= cuts;
}

void MCStrategy::createVertexNodes(DMCOctreeLeaf* leaf, const Index &leaf_index) const {

    uint vCount = vertexCount[leaf->signConfig];
    if (vCount == 0)
        return;
    uint count = 0;
    int8_t vEdges[8] = {-1,-1,-1,-1,-1,-1,-1,-1};
    for (uint i = 0; i < 16; ++i) {
        int edge = edgeTable[leaf->signConfig][i];
        if (edge < 0) {
            VertexNode* vNode = new VertexNode();
            initQEF(vEdges, count, leaf, leaf_index, *(vNode->qef));
            vertexStrategy->generateVertex(leaf_index, leaf->level, res, *(vNode->qef), vNode->v);
            vNode->computeError();
            vNode->collapsable = true; // a leaf node is always collapsable
            // assign vertex index to edges
            for (uint e = 0; e < count; ++e) {
                leaf->frontEdgeVertices[vEdges[e]] = leaf->vertices.size();
                leaf->backEdgeVertices[vEdges[e]] = leaf->vertices.size();
            }
            leaf->vertices.push_back(shared_ptr<VertexNode>(vNode));

            count = 0;
            if (edge == -2)
                break;
        } else
            vEdges[count++] = edge;
    }
}

void ThinShelledStrategy::initQEF(int comp[10], int size, HermiteData* frontCuts[12], HermiteData* backCuts[12], const Index &leaf_index,QEF& qef) const {
    int edge = 0;
    const HermiteData* data = nullptr;
    for (int i = 0; i < size; i++) { // iterate over edge cuts of this component
        edge = comp[i];
        if (edge > 0) {
            edge -= 1;
            if (frontCuts[edge])
                data = frontCuts[edge];
            else {
                data = backCuts[edge];
                comp[i] *= -1;
            }
        } else {
            edge = -edge-1;
            if (backCuts[edge])
                data = backCuts[edge];
            else {
                data = frontCuts[edge];
                comp[i] *= -1;
            }
        }
        Index e = leaf_index + corner_delta[edge_corners[edge][0]];
        Vector3f p = Vector3f(e.x,e.y,e.z);
        p[edge_dir[edge]] += data->d;
        p /= res;
        qef.add(data->n, p);
        qef.m += p;
    }

    qef.m /= size;
}


float ThinShelledStrategy::compRank(int comp[10], int size, HermiteData* frontCuts[12], HermiteData* backCuts[12]) const {
    float deviation = 1.0; //the max normal deviation (dot product [-1,1], higher is better) of this component
    vector<Vector3f*> normals;
    int edge = 0;
    HermiteData* data = nullptr;
    for (int i = 0; i < size; ++i) { // iterate over edge cuts of this component
        edge = comp[i];
        if (edge > 0) {
            edge -= 1;
            if (frontCuts[edge])
                data = frontCuts[edge];
            else
                data = backCuts[edge];
        } else {
            edge = -edge-1;
            if (backCuts[edge])
                data = backCuts[edge];
            else
                data = frontCuts[edge];
        }
        for (Vector3f* normal : normals)
            deviation = std::min(deviation, (float) (normal->transpose()*data->n));

        normals.push_back(&data->n);
    }
    return deviation;
}

void ThinShelledStrategy::createVertexNodes(DMCOctreeLeaf *leaf, const Index &leaf_index) const{
    // gather edge data of this cell
    HermiteData* frontCuts[12] = {nullptr};
    HermiteData* backCuts[12] = {nullptr};
    int cutCount = 0;
    for (int e = 0; e < 12; ++e) {
        Index edge = leaf_index + corner_delta[edge_corners[e][0]];
        bool s0 = leaf->sign(edge_corners[e][0]);
        bool s1 = leaf->sign(edge_corners[e][1]);
        if (!s0 && s1) {
            frontCuts[e] = sampler->edgeInfo(edge_dir[e], FRONT, edge);
            cutCount++;
        } else if (s0 && !s1) {
            backCuts[e] = sampler->edgeInfo(edge_dir[e], BACK, edge);
            cutCount++;
        } else if (!s0 && !s1 && sampler->hasCut(edge_dir[e], FRONT, edge) && sampler->hasCut(edge_dir[e], BACK, edge)) {
            frontCuts[e] = sampler->edgeInfo(edge_dir[e], FRONT, edge);
            backCuts[e] = sampler->edgeInfo(edge_dir[e], BACK, edge);
            cutCount += 2;
        }
    }
    int bestComp[10] = {0};
    int comp[10] = {0};
    int c = 0;
    int bestSize = 0;
    float bestRank = -2;
    while (cutCount > 0) {
        bestRank = -2;
        // compute edge configuration
        int edgeConfig = 0;
        for (int i = 0; i < 12; ++i) {
            if (frontCuts[i] || backCuts[i]) {
                edgeConfig |= 1 << i;
            }
        }

        // iterate over all possible surface components for this edgeconfig
        for (int i = 0; i < 32; ++i) {
            int edge = compEdgeTable[edgeConfig][i];
            if (edge == 0) { // no more components
                c = 0;
                break;
            }
            if (edge == -13) { // end of component
                float rank = compRank(comp, c, frontCuts, backCuts);
                // compare rank (normal deviation) of the component
                if (rank > bestRank || (c == cutCount && bestRank-rank < eps_normal_dev)) {
                    copy(comp, comp+c, bestComp);
                    bestSize = c;
                    bestRank = rank;
                }
                c = 0;
            } else { // add to current component
                comp[c++] = edge;
            }
        }

        // create vertex node for the best surface component
        VertexNode* vNode = new VertexNode();
        initQEF(bestComp, bestSize, frontCuts, backCuts, leaf_index, *(vNode->qef));
        vertexStrategy->generateVertex(leaf_index, sampler->leaf_level, res, *(vNode->qef), vNode->v);
        vNode->computeError();
        vNode->collapsable = true; // a leaf node is always collapsable

        // assign vertex index to edges
        int edge = 0;
        for (int i = 0; i < bestSize; ++i) { // iterate over edge cuts of this component
            edge = bestComp[i];
            if (edge > 0) {
                edge -= 1;
                leaf->frontEdgeVertices[edge] = leaf->vertices.size();
                frontCuts[edge] = nullptr; // remove edge cut
            } else {
                edge = -edge-1;
                leaf->backEdgeVertices[edge] = leaf->vertices.size();
                backCuts[edge] = nullptr; // remove edge cut
            }
            cutCount--;
        }
        leaf->vertices.push_back(shared_ptr<VertexNode>(vNode));
    }
}



DMCOctreeCell* DualMarchingCubes::createOctreeNodes(uint size, const Index& index, uint8_t level) {
    uint child_size = size/2;
    if (level == leaf_level) {
       uint8_t signConfig = signSampler->signConfig(index);
       if (signConfig == 255 || !signSampler->contributes(index)) // inside sign config or no conributing cut
           return nullptr;

       DMCOctreeLeaf* node = new DMCOctreeLeaf(level);
       node->signConfig = signConfig;
       componentStrategy->createVertexNodes(node, index);
       return node;
    }


    bool homogeneous = true; // true if every child cell is homogeneous
    DMCOctreeCell* children[8];
    for (int i = 0; i < 8; ++i) {
        children[i] = nullptr;
        Index child_index = index + child_size*child_origin[i];
        children[i] = createOctreeNodes(child_size, child_index, level+1);        
        homogeneous &= !children[i];
        if (level == 0)
            cout << endl;
        if (level == 1)
            cout << i << "..";
    }

    if (!homogeneous) {
        DMCOctreeCell* node = new DMCOctreeNode(level);
        // create nodes
        for (int i = 0; i < 8; ++i) {
            if (children[i])
                node->setChild(i, children[i]);
            else {
                Index child_index = index + child_size*child_origin[i];
                node->setChild(i, new DMCHomogeneousCell(level+1, signSampler->sign(child_index)));
            }
        }
        clusterCell(index, node);
        return node;
    }

    return nullptr;


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


bool DualMarchingCubes::conturing(RenderStrategy* scene, float voxelGridRadius, uint resolution) {
    programEdgeScan.removeAllShaders();
    programHermiteScan.removeAllShaders();

    if(!scene->initShaders(programEdgeScan, programHermiteScan))
        return false;
    this->scene = scene;
    this->res = nextPowerOf2(resolution);
    this->voxelGridRadius = voxelGridRadius;
    this->cell_size = 2*voxelGridRadius/(resolution+1);
    this->leaf_level = i_log2(res);

    std::cout << "scan edge intersections (" << resolution << ")" << std::endl;
    CompressedEdgeScanner edgeScanner(resolution);
    edgeScanner.program = &programEdgeScan;

    //shift near and far plane about a half voxel cell
    //because rays are casted from the pixel center
    QMatrix4x4 projection;
    projection.ortho(-voxelGridRadius, voxelGridRadius,
                     -voxelGridRadius, voxelGridRadius,
                     -voxelGridRadius + 0.5*cell_size,
                     voxelGridRadius - 0.5*cell_size);
    edgeScanner.projection = projection;
    edgeScanner.scan(scene);

    std::cout << "create sign sampler (flood fill)" << std::endl;
    signSampler = unique_ptr<SignSampler>(new CompressedSignSampler(edgeScanner.data));

    std::cout << "scan hermite data (" << resolution << ")" << std::endl;
    HermiteScanner hermiteScanner(res, signSampler->contributingIntersections);
    hermiteScanner.program = &programHermiteScan;
    hermiteScanner.projection = projection;
    hermiteScanner.scan(scene);

    sampler = unique_ptr<HermiteDataSampler>(new SparseHermiteSampler(hermiteScanner.data));
    hermiteScanner.data = nullptr;
    componentStrategy->sampler = sampler.get();
    componentStrategy->res = res;

    std::cout << "create octree (" << res << ")" << std::endl;
    DMCOctreeCell* node = createOctreeNodes(res, Index(0), 0);
    if (!node)
        node = new DMCHomogeneousCell(0, signSampler->sign(Index(0)));
    root = unique_ptr<DMCOctreeCell>(node);

    signSampler.reset();
    sampler.reset();

    return true;
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


