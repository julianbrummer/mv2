#ifndef MV_H
#define MV_H

#include <QMainWindow>
#include <QGridLayout>
#include <QFrame>
#include <QtCore>
#include <QLocale>
#include <QInputDialog>
#include <QLabel>

#include <QtOpenGL/QGLWidget>
#include <QtOpenGL/QGLFunctions>
#include <QtOpenGL/QGLShaderProgram>
#include <QOpenGLFunctions_2_0>
#include <QOpenGLFunctions_4_3_Core>
#include <QOpenGLBuffer>

#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include "scene.h"
#include "dmc.h"

using namespace Eigen;
using namespace std;

class ConturingWidget;

class CGMainWindow : public QMainWindow {
    Q_OBJECT

public:

    static const int SLIDER_GRANULARITY = 1000;

    CGMainWindow (QWidget* parent = 0);
    ~CGMainWindow ();
    ConturingWidget *ogl;
    QLabel* t_error_label;

};

class DefaultRenderStrategy : public RenderStrategy, protected QOpenGLFunctions_4_3_Core {
protected:
    virtual GLuint subroutineSelection();
    virtual void doRender(QGLShaderProgram& program, const QMatrix4x4& projection, const QMatrix4x4& view) = 0;
public:
    bool initShaders(QGLShaderProgram &programEdgeScan, QGLShaderProgram &programHermiteScan) const override;
    void render(QGLShaderProgram& program, const QMatrix4x4& projection, const QMatrix4x4& view) override;
};

class RenderSingleModel : public DefaultRenderStrategy {
private:
    const Model* model;
public:
    RenderSingleModel(const Model *model);
protected:
    void doRender(QGLShaderProgram& program, const QMatrix4x4& projection, const QMatrix4x4& view) override;
};

class RenderTrafoModel : public DefaultRenderStrategy {
private:
    const Model* model;
    const Trafo* trafo;
    int progress_update_instances;
public:
    RenderTrafoModel(const Model *model, const Trafo *trafo, int progress_update_instances);
protected:
    void doRender(QGLShaderProgram& program, const QMatrix4x4& projection, const QMatrix4x4& view) override;
};

class RenderTrafoModelInstanced : public DefaultRenderStrategy {
private:
    const Model* model;
    const Trafo* trafo;
    int max_instances;
public:
    RenderTrafoModelInstanced(const Model *model, const Trafo *trafo, int max_instances);
protected:
    virtual GLuint subroutineSelection();
    void doRender(QGLShaderProgram& program, const QMatrix4x4& projection, const QMatrix4x4& view) override;
};

class RenderSparseTrafoModel : public RenderTrafoModelInstanced {
public:
    RenderSparseTrafoModel(const Model *model, const Trafo *trafo, int max_instances)
        : RenderTrafoModelInstanced(model, trafo, max_instances) {}
    bool initShaders(QGLShaderProgram &programEdgeScan, QGLShaderProgram &programHermiteScan) const override;
};

class ConturingWidget : public QGLWidget, public QOpenGLFunctions_2_0 {
    Q_OBJECT

public slots:
    void storeModel();
    void loadModel();
    void loadTrack();
    void removeTrack() {
        trafo = nullptr;
        trafo_buffer = nullptr;
        trafo_now = 0;
        if (model) {
            model->init(1.9f);
        }
    }

    void sliderValueChanged(int value);

    void toggleViewModel() {showModel = !showModel; updateGL();}
    void toggleViewEdgeIntersections() {showEdgeIntesections = !showEdgeIntesections; updateGL();}
    void toggleViewInGridPoints() {showInGridPoints = !showInGridPoints; updateGL();}
    void toggleViewOutGridPoints() {showOutGridPoints = !showOutGridPoints; updateGL();}
    void toggleViewDMCVertices() {showDMCVertices = !showDMCVertices; updateGL();}
    void toggleViewOutModel() {showOutModel = !showOutModel; updateGL();}
    void toggleWireframe() {wireframe = !wireframe; updateGL();}
    void toggleViewCells() {
        showCells = !showCells;
        updateGL();
    }
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

        if (showCells) {
            Vector3f cell_center(selectedCell.x()+0.5, selectedCell.y()+0.5, selectedCell.z()+0.5);
            camera.center = origin+cell_center*2*voxelGridRadius/pow2(selectedLevel);
        } else {
            camera.center.setZero(3);
        }

        updateGL();
    }
    void thinShelled() {
        DMC.componentStrategy = unique_ptr<SurfaceComponentStrategy>(new ThinShelledStrategy(DEFAULT_TRUNCATION));
    }
    void dualContouring() {
        DMC.componentStrategy = unique_ptr<SurfaceComponentStrategy>(new DCStrategy(DEFAULT_TRUNCATION));
    }
    void dmc() {
        DMC.componentStrategy = unique_ptr<SurfaceComponentStrategy>(new MCStrategy(DEFAULT_TRUNCATION));
    }
    void toggleSparseTrajectory() {
        sparseTrajectory = !sparseTrajectory;
    }
    void toggleDebug() {
        debug = !debug;
    }
    void trackForwardSlow() {
        shiftTrack(1);
    }
    void trackForwardNormal() {
        shiftTrack(50);
    }
    void trackForwardFast() {
        shiftTrack(1000);
    }
    void trackBackwardSlow() {
        shiftTrack(-1);
    }
    void trackBackwardNormal() {
        shiftTrack(-50);
    }
    void trackBackwardFast() {
        shiftTrack(-1000);
    }

    void updateDMCMesh();
    void compute();


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const int CELL_EDGES_VERTEX_COUNT = 24;
    static const float CAMERA_MOVEMENT_SPEED;
    static const float CAMERA_SCROLL_FACTOR;
    static const float MIN_ERROR_THRESHOLD;
    static const float MAX_ERROR_THRESHOLD;
    static const float DEFAULT_TRUNCATION;
    static const int MIN_OCTREE_DEPTH = 5;
    static const int DEBUG_MAX_OCTREE_DEPTH = 7;
    static const int MAX_INSTANCES = 100;


    ConturingWidget(CGMainWindow*,QWidget*);
    void initializeGL();

    void trackballCoord(int x, int y, Vector3f& v);
    Quaternionf trackball(const Vector3f& u, const Vector3f& v);

    void updateThreshold(float s);


    unique_ptr<Model> model, edgeIntersections, inGridPoints, outGridPoints, dmcVertices, dmcModel, cells;
    unique_ptr<RenderStrategy> scene;
    Camera camera;

    Vector3f origin;

    //bool wireframe;
    bool showModel, showEdgeIntesections, showInGridPoints, showOutGridPoints,
         showOutModel, showDMCVertices, showCells, wireframe, sparseTrajectory, debug;

    GLint max_res;
    uint res;
    float errorThreshold;
    int levels, max_depth;

    qreal timestep;
    unique_ptr<Trafo> trafo;
    int trafo_now;
    unique_ptr<SSBO> trafo_buffer;

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
    void shiftTrack(int shift);
    void updateRenderStrategy();
    void fillTrafoBuffers();

    void bindModel(const Matrix4f &VM, QVector4D color);
    void renderModel(Model* model, const Matrix4f &VM, QVector4D color);
    void bindDebugMesh(Model* model, const Matrix4f &V, bool useVertexColor = false, QVector4D color = QVector4D(1,1,1,1));
    void renderDebugMesh(Model* model, const Matrix4f &V, bool useVertexColor = false, QVector4D color = QVector4D(1,1,1,1));

    void createDMCMesh();
    void createCellMesh();

    QGLShaderProgram program, programDebug;
    int w, h;

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
    float voxelGridRadius;

    DualMarchingCubes DMC;

    // keep to write to file
    aligned_vector3f positions;
    vector<uint> indices;


};

#endif
