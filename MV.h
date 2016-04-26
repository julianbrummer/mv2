#ifndef MV_H
#define MV_H

#include <QMainWindow>
#include <QGridLayout>
#include <QFrame>
#include <QtCore>
#include <QLocale>
#include <QInputDialog>

#include <QtOpenGL/QGLWidget>
#include <QtOpenGL/QGLFunctions>
#include <QtOpenGL/QGLShaderProgram>
#include <QOpenGLFunctions_2_0>

#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include "scene.h"
#include "dmc.h"

using namespace Eigen;
using namespace std;

class MyGLWidget;

class CGMainWindow : public QMainWindow {
    Q_OBJECT

public:

    CGMainWindow (QWidget* parent = 0);
    ~CGMainWindow ();
    MyGLWidget *ogl;

public slots:
    void loadModel();
    //void loadTrack();
};


class MyGLWidget : public QGLWidget, public QOpenGLFunctions_2_0 {
    Q_OBJECT

public slots:
    void toggleViewModel() {showModel = !showModel; updateGL();}
    void toggleViewEdgeIntersections() {showEdgeIntesections = !showEdgeIntesections; updateGL();}
    void toggleViewInGridPoints() {showInGridPoints = !showInGridPoints; updateGL();}
    void toggleViewOutGridPoints() {showOutGridPoints = !showOutGridPoints; updateGL();}
    void toggleViewDMCVertices() {showDMCVertices = !showDMCVertices; updateGL();}
    void toggleViewCells() {showCells = !showCells; updateGL();}
    void toggleViewCellLevel() {showSingleCell = !showSingleCell; updateGL();}
    void increaseSelectedLevel() {
        if (selectedLevel < levels-1) {
            selectedLevel = selectedLevel+1;
            selectedCell = selectedCell*2;
        }
        updateGL();
    }
    void decreaseSelectedLevel() {
        if (selectedLevel > 0) {
            selectedLevel = selectedLevel-1;
            selectedCell = selectedCell/2;
        }
        updateGL();
    }
    void centerCamera() {
        if (showSingleCell) {
            Vector3f cell_center(selectedCell.x+0.5, selectedCell.y+0.5, selectedCell.z+0.5);
            camera.center = origin+cell_center*2*cellGridRadius/pow2(selectedLevel);
            camera.position = camera.center - camera.forwd()*zoom;
        } else {
            camera.center.setZero(3);
            camera.position = -camera.forwd()*zoom;
        }
        updateGL();
    }

    void dmc();
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MyGLWidget(CGMainWindow*,QWidget*);
    bool initShaderProgram(const char *vname, const char *fname, QGLShaderProgram& program);
    void initializeGL();

    void trackballCoord(int x, int y, Vector3f& v);
    Quaternionf trackball(const Vector3f& u, const Vector3f& v);

/*
    void snapshotDualConturing(bool front_faces);
    void snapshot(QGLShaderProgram &program);

    void createAll();
    void copyEdgeData(std::vector<std::vector<GLuint> > &edgeData);
    void copyGridData(std::vector<bool>& gridData);
*/
    static const int CELL_EDGES_VERTEX_COUNT = 24;
    static const float CAMERA_MOVEMENT_SPEED;
    static const float CAMERA_SCROLL_FACTOR;
    unique_ptr<Model> model, edgeIntersections, inGridPoints, outGridPoints, dmcVertices, cells;
    Camera camera;

    Vector3f origin;

    //bool wireframe;
    bool showModel, showEdgeIntesections, showInGridPoints, showOutGridPoints, showDMCVertices, showCells, showSingleCell;

    //bool showEdgeCuts, showCells, showGrid, showDCVertices, showDCMesh, showAxis;
    uint res;
    int levels;

    //qreal timestep;

    //std::vector<QMatrix4x4> trafo;
    //int trafo_cnt;

protected:

    void paintGL();
    void resizeGL(int width, int height);

    void mouseMoveEvent(QMouseEvent*);
    void mousePressEvent(QMouseEvent*);
    void mouseReleaseEvent(QMouseEvent*);
    void wheelEvent(QWheelEvent*);
    void keyPressEvent(QKeyEvent*);

    CGMainWindow *main;
    int oldX,oldY,button;

private:
    void bindDebugMesh(Model* model, const Matrix4f &V, bool useVertexColor = false, QVector4D color = QVector4D(1,1,1,1));
    void renderDebugMesh(Model* model, const Matrix4f &V, bool useVertexColor = false, QVector4D color = QVector4D(1,1,1,1));

    QGLShaderProgram program, programColor;
    QGLShaderProgram programScan;
    int w, h;
    float rotX, rotY, zoom;

    Index selectedCell;
    int selectedLevel;
    // the offset in the VBO for each cell on each level
    // [level][x][y][z]
    vector_4_uint v_offset, cell_offset;
    // the count of dmc vertices in the VBO for each cell on each level
    // [level][x][y][z]
    vector_4_uint v_count;
    // the count of vertices in the VBO for each level
    vector<uint> v_level_count, cell_level_count;
    float voxelGridRadius, cellGridRadius;


};

#endif
