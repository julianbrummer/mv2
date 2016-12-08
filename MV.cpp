#include <QApplication>
#include <QMenu>
#include <QMenuBar>
#include <QStatusBar>
#include <QFileDialog>
#include <QKeyEvent>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <iostream>
#include "MV.h"


#include "reader.h"
#include "scan.h"
#include "util.h"


CGMainWindow *w;

CGMainWindow::CGMainWindow (QWidget* parent) 
	: QMainWindow (parent) {
    resize (800,800);

    // Create a nice frame to put around the OpenGL widget
    QFrame* f = new QFrame (this);
    f->setFrameStyle(QFrame::Sunken | QFrame::Panel);
    f->setLineWidth(2);

    // Create our OpenGL widget
    ogl = new ConturingWidget (this,f);
    ogl->setFocusPolicy(Qt::StrongFocus); // for keyboard events
    ogl->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

    // Create a menu
    QMenu *file = new QMenu("&File",this);
    file->addAction ("Load model", ogl, SLOT(loadModel()), Qt::CTRL+Qt::Key_L);
    file->addAction ("Load track", ogl, SLOT(loadTrack()), Qt::CTRL+Qt::Key_T);
    file->addAction ("Remove track", ogl, SLOT(removeTrack()), Qt::CTRL+Qt::Key_R);
    file->addAction ("Save model", ogl, SLOT(storeModel()), Qt::CTRL+Qt::Key_S);

    file->addAction ("Quit", qApp, SLOT(quit()), Qt::CTRL+Qt::Key_Q);

    menuBar()->addMenu(file);               

    QMenu *view = new QMenu("&View",this);
    view->addAction ("Model", ogl, SLOT(toggleViewModel()), Qt::Key_M)->setCheckable(true);
    view->addAction ("Edge Intersections", ogl, SLOT(toggleViewEdgeIntersections()), Qt::Key_E)->setCheckable(true);
    view->addAction ("Inner Grid", ogl, SLOT(toggleViewInGridPoints()), Qt::Key_I)->setCheckable(true);
    view->addAction ("Vertices", ogl, SLOT(toggleViewDMCVertices()), Qt::Key_V)->setCheckable(true);
    view->addAction ("Swept Volume", ogl, SLOT(toggleViewOutModel()), Qt::Key_S)->setCheckable(true);
    view->addAction ("Wireframe", ogl, SLOT(toggleWireframe()), Qt::Key_W)->setCheckable(true);
    view->addAction ("Cells", ogl, SLOT(toggleViewCells()), Qt::Key_C)->setCheckable(true);
    view->addAction ("Higher Level", ogl, SLOT(decreaseSelectedLevel()), Qt::Key_Plus);
    view->addAction ("Lower Level", ogl, SLOT(increaseSelectedLevel()), Qt::Key_Minus);
    view->addAction ("Center Camera", ogl, SLOT(centerCamera()), Qt::Key_Comma);

    menuBar()->addMenu(view);


    QMenu *track = new QMenu("&Track",this);
    track->addAction (">", ogl, SLOT(trackForwardSlow()), Qt::Key_Right);
    track->addAction (">>", ogl, SLOT(trackForwardNormal()), Qt::Key_Up);
    track->addAction (">>>", ogl, SLOT(trackForwardFast()), Qt::Key_PageUp);
    track->addAction ("<", ogl, SLOT(trackBackwardSlow()), Qt::Key_Left);
    track->addAction ("<<", ogl, SLOT(trackBackwardNormal()), Qt::Key_Down);
    track->addAction ("<<<", ogl, SLOT(trackBackwardFast()), Qt::Key_PageDown);

    menuBar()->addMenu(track);

    QMenu *sv = new QMenu("&SweptVolume",this);
    sv->addAction ("Sparse Trajectory", ogl, SLOT(toggleSparseTrajectory()), Qt::CTRL+Qt::ALT+Qt::Key_S)->setCheckable(true);
    sv->addAction ("Thin Shelled", ogl, SLOT(toggleThinShelled()), Qt::CTRL+Qt::ALT+Qt::Key_T)->setCheckable(true);
    sv->addAction ("Compute", ogl, SLOT(compute()), Qt::CTRL+Qt::Key_D);
    menuBar()->addMenu(sv);

    QSlider *threshold_slider = new QSlider(Qt::Horizontal);
    threshold_slider->setMinimum(0);
    threshold_slider->setMaximum(SLIDER_GRANULARITY);
    threshold_slider->setSliderPosition(0);
    QLabel *threshold_label = new QLabel("Error Threshold");
    t_error_label = new QLabel(to_string(ogl->errorThreshold).c_str());
    QPushButton *simplify_button = new QPushButton("Simplify");

    threshold_label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    t_error_label->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Preferred);
    threshold_slider->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    simplify_button->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);

    connect(threshold_slider,SIGNAL(valueChanged(int)),ogl,SLOT(sliderValueChanged(int)));
    connect(simplify_button,SIGNAL(clicked(bool)),ogl,SLOT(updateDMCMesh()));

    // Put the widgets inside the frame
    QVBoxLayout* layout = new QVBoxLayout();
    QHBoxLayout* topbar = new QHBoxLayout();
    topbar->addWidget(threshold_label);
    topbar->addWidget(threshold_slider);
    topbar->addWidget(t_error_label);
    topbar->addWidget(simplify_button);
    layout->addLayout(topbar);
    layout->addWidget(ogl);
    layout->setMargin(0);
    f->setLayout(layout);
    setCentralWidget(f);

    statusBar()->showMessage("Ready",1000);
}

CGMainWindow::~CGMainWindow () {
    delete ogl;
    ogl = nullptr;
}

/*

void writeToStlFile(std::vector<QVector3D>& T, const char *filename) {
	char buffer[80];
	for(int i=0;i<80;i++)
		buffer[i] = ' ';
	buffer[0] = 'V';
	buffer[1] = 'C';
	buffer[2] = 'G';

        std::ofstream outstream(filename,std::ofstream::binary);
	outstream.write(buffer,80);
	unsigned int n = T.size()/3;
	outstream.write((char*) &n,4);

	float f;
	unsigned short s = 0;

	for(size_t j=0;j<n;j++) {
		const QVector3D& a = T[3*j+0];
		const QVector3D& b = T[3*j+1];
		const QVector3D& c = T[3*j+2];
		QVector3D n = QVector3D::normal(a,b,c);
		f = n.x(); outstream.write((char*) &f,4);
		f = n.y(); outstream.write((char*) &f,4);
		f = n.z(); outstream.write((char*) &f,4);

		f = a.x(); outstream.write((char*) &f,4);
		f = a.y(); outstream.write((char*) &f,4);
		f = a.z(); outstream.write((char*) &f,4);

		f = b.x(); outstream.write((char*) &f,4);
		f = b.y(); outstream.write((char*) &f,4);
		f = b.z(); outstream.write((char*) &f,4);

		f = c.x(); outstream.write((char*) &f,4);
		f = c.y(); outstream.write((char*) &f,4);
		f = c.z(); outstream.write((char*) &f,4);

		outstream.write((char*) &s,2);
	}

	outstream.close();
}

void writeToStlFile(const std::vector<Vertex>& T, const char *filename) {
    char buffer[80];
    for(int i=0;i<80;i++)
        buffer[i] = ' ';
    buffer[0] = 'V';
    buffer[1] = 'C';
    buffer[2] = 'G';

        std::ofstream outstream(filename,std::ofstream::binary);
    outstream.write(buffer,80);
    unsigned int n = T.size()/3;
    outstream.write((char*) &n,4);

    float f;
    unsigned short s = 0;

    for(size_t j=0;j<n;j++) {
        const Vector3f& n = T[3*j+0].normal;
        const Vector3f& a = T[3*j+0].position;
        const Vector3f& b = T[3*j+1].position;
        const Vector3f& c = T[3*j+2].position;
        f = n.x(); outstream.write((char*) &f,4);
        f = n.y(); outstream.write((char*) &f,4);
        f = n.z(); outstream.write((char*) &f,4);

        f = a.x(); outstream.write((char*) &f,4);
        f = a.y(); outstream.write((char*) &f,4);
        f = a.z(); outstream.write((char*) &f,4);

        f = b.x(); outstream.write((char*) &f,4);
        f = b.y(); outstream.write((char*) &f,4);
        f = b.z(); outstream.write((char*) &f,4);

        f = c.x(); outstream.write((char*) &f,4);
        f = c.y(); outstream.write((char*) &f,4);
        f = c.z(); outstream.write((char*) &f,4);

        outstream.write((char*) &s,2);
    }

    outstream.close();
}
*/

bool DefaultRenderStrategy::initShaders(QGLShaderProgram &programEdgeScan, QGLShaderProgram &programHermiteScan) const {
    if (!initShaderProgram(programEdgeScan, ":/shaders/vEdgeScan.glsl", ":/shaders/fEdgeScan.glsl"))
        return false;
    if (!initShaderProgram(programHermiteScan, ":/shaders/vHermiteScan.glsl", ":/shaders/fHermiteScan.glsl"))
        return false;
    return true;
}

GLuint DefaultRenderStrategy::subroutineSelection() {
    return 0;
}

void DefaultRenderStrategy::render(QGLShaderProgram &program, const QMatrix4x4 &projection, const QMatrix4x4 &view) {
    GLuint index = subroutineSelection();
    glUniformSubroutinesuiv(GL_VERTEX_SHADER, 1, &index);
    doRender(program, projection, view);
}

RenderSingleModel::RenderSingleModel(const Model *model) : model(model) {
    initializeOpenGLFunctions();
}


void RenderSingleModel::doRender(QGLShaderProgram &program, const QMatrix4x4 &projection, const QMatrix4x4 &view) {
    QMatrix4x4 PV = projection * view;
    Matrix4f M = model->getModelMatrix();
    program.setUniformValue("uVPMat", PV);
    program.setUniformValue("uMMat", qMat(M));
    model->render(program);
}


RenderTrafoModel::RenderTrafoModel(const Model* model, const Trafo* trafo, int progress_update_instances)
        : model(model), trafo(trafo), progress_update_instances(progress_update_instances) {
    initializeOpenGLFunctions();
}

void RenderTrafoModel::doRender(QGLShaderProgram &program, const QMatrix4x4 &projection, const QMatrix4x4 &view) {
    QMatrix4x4 PV = projection * view;
    program.setUniformValue("uVPMat", PV);
    for (uint i = 0; i < trafo->size(); ++i) {
        Matrix4f M_trafo = model->getModelMatrix() * (*trafo)[i];
        program.setUniformValue("uMMat", qMat(M_trafo));
        model->render(program);
        glFinish();
        glFlush();
        if (i % progress_update_instances == 0)
            std::cout << "  rendering " << i << "/" << trafo->size() << "\r" << std::flush;
    }
    std::cout << "  rendering done. " << trafo->size() << "/" << trafo->size() << std::endl;
}

RenderTrafoModelInstanced::RenderTrafoModelInstanced(const Model* model, const Trafo* trafo, int max_instances)
        : model(model), trafo(trafo), max_instances(max_instances) {
    initializeOpenGLFunctions();
}


GLuint RenderTrafoModelInstanced::subroutineSelection() {
    return 1;
}

void RenderTrafoModelInstanced::doRender(QGLShaderProgram &program, const QMatrix4x4 &projection, const QMatrix4x4 &view) {
    QMatrix4x4 PV = projection * view;
    program.setUniformValue("uVPMat", PV);
    // render in instanced groups of size max_instance
    //(too many instance at once slows down performance or even crash the program)
    int rendered_instances = 0;
    int max = trafo->size() - max_instances;
    while (rendered_instances < max) {
        model->renderInstanced(program, max_instances, rendered_instances);
        glFinish();
        glFlush();
        rendered_instances += max_instances;
        std::cout << "  rendering " << rendered_instances << "/" << trafo->size() << "\r" << std::flush;
    }
    model->renderInstanced(program, trafo->size() - rendered_instances);
    glFinish();
    glFlush();
    std::cout << "  rendering done. " << trafo->size() << "/" << trafo->size() << std::endl;
}

bool RenderSparseTrafoModel::initShaders(QGLShaderProgram &programEdgeScan, QGLShaderProgram &programHermiteScan) const {
    if (!initShaderProgram(programEdgeScan, ":/shaders/vPatchScan.glsl", ":/shaders/fEdgeScan.glsl", ":/shaders/gEdgePatchScan.glsl"))
        return false;
    if (!initShaderProgram(programHermiteScan, ":/shaders/vPatchScan.glsl", ":/shaders/fHermiteScan.glsl", ":/shaders/gHermitePatchScan.glsl"))
        return false;
    return true;
}

const float ConturingWidget::CAMERA_MOVEMENT_SPEED = 0.0025f;
const float ConturingWidget::CAMERA_SCROLL_FACTOR = 1.2f;
const float ConturingWidget::MIN_ERROR_THRESHOLD = 0.0f;
const float ConturingWidget::MAX_ERROR_THRESHOLD = 0.0001f;
const float ConturingWidget::DEFAULT_TRUNCATION = 0.1f;

ConturingWidget::ConturingWidget (CGMainWindow *mainwindow,QWidget* parent ) : QGLWidget (parent) {
    main = mainwindow;
    errorThreshold = MIN_ERROR_THRESHOLD;
}

void ConturingWidget::storeModel() {
    if (!dmcModel) {
        main->statusBar()->showMessage ("No output model!",3000);
        return;
    }
    QString filename = QFileDialog::getSaveFileName(this, "Save output model ...", QString(), "(*.off)" );
    writeToOffFile(positions, indices, filename.toLatin1());

    main->statusBar()->showMessage ("Saving model done.",3000);
}

void ConturingWidget::loadTrack() {
    QString filename = QFileDialog::getOpenFileName(this, "Load track ...", QString(), "(*.vda)" );

    if (filename.isEmpty()) return;
    bool ok;
    double d = QInputDialog::getDouble(this, "Scale Factor", "How do you want to scale the track?", 1.0, 0.0001,numeric_limits<double>::max(),1,&ok);
    if (!ok) return;

    removeTrack();
    main->statusBar()->showMessage ("Loading track ...");

    trafo = unique_ptr<Trafo>(new Trafo());
    trafo->scale = d;
    LoadVdaFile(trafo->M,filename.toStdString().c_str(),trafo->scale, timestep);

    if (model) {
        model->init(1.9f, *trafo);
        fillTrafoBuffers();
    }
    main->statusBar()->showMessage ("Loading track done.",3000);
}

void ConturingWidget::loadModel() {

    QString filename = QFileDialog::getOpenFileName(this, "Load model ...", QString(), "(*.stl *.off)" );

    if (filename.isEmpty()) return;
        main->statusBar()->showMessage ("Loading model ...");

    aligned_vector3f positions, normals;
    if (filename.endsWith(".stl"))
        LoadStlFile(filename.toLatin1(), positions, normals);

    if (filename.endsWith(".off"))
        LoadOffFile(filename.toLatin1(), positions, normals);


    model = unique_ptr<Model>(new Model(positions, normals, GL_TRIANGLES, true, 1.9f));
    if (trafo) {
        model->init(1.9f, *trafo);
        fillTrafoBuffers();
    }
    camera.position = camera.rotation._transformVector(Z_AXIS) + camera.center;

    main->statusBar()->showMessage ("Loading model done.",3000);
}

void ConturingWidget::shiftTrack(int shift) {
    if (trafo) {
        trafo_now += shift;
        trafo_now = max(min(trafo_now, (int) trafo->size()-1), 0);

        updateGL();
    }
}

void ConturingWidget::updateRenderStrategy() {
    if (model) {
        if (trafo) {
            if (sparseTrajectory)
                scene = unique_ptr<RenderStrategy>(new RenderSparseTrafoModel(model.get(), trafo.get(), MAX_INSTANCES));
            else
                scene = unique_ptr<RenderStrategy>(new RenderTrafoModelInstanced(model.get(), trafo.get(), MAX_INSTANCES));
        } else
            scene = unique_ptr<RenderStrategy>(new RenderSingleModel(model.get()));
    }
}

void ConturingWidget::fillTrafoBuffers() {
    std::cout << "  fill SSBO with trafo matrices" << std::endl;
    trafo_buffer = unique_ptr<SSBO>(new SSBO(1, trafo->size()*64, GL_STATIC_DRAW));
    trafo_buffer->bind();
    float values[16];
    for (uint i = 0; i < trafo->size(); ++i) {
        Matrix4f M_trafo = model->getModelMatrix() * (*trafo)[i];
        Map<Matrix4f>(&values[0], 4, 4) = M_trafo;
        trafo_buffer->bufferSubData(i*64, 64, values);
    }
    trafo_buffer->unBind();
}

void ConturingWidget::sliderValueChanged(int value) {
    updateThreshold((float)value/CGMainWindow::SLIDER_GRANULARITY);
    main->t_error_label->setText(to_string(errorThreshold).c_str());
}

void ConturingWidget::initializeGL() {
    initializeOpenGLFunctions();
    initShaderProgram(program, ":/shaders/vshader.glsl", ":/shaders/fshader.glsl");
    initShaderProgram(programDebug, ":/shaders/vpoints.glsl", ":/shaders/fpoints.glsl");

    camera.zoom = 1.0f;
    voxelGridRadius = 1.0f;

    trafo_now = 0;

    showModel = false;
    showOutModel = false;
    showEdgeIntesections = false;
    showInGridPoints = false;
    showOutGridPoints = false;
    showDMCVertices = false;
    showCells = false;
    wireframe = false;
    thinShelled = false;
    sparseTrajectory = false;
    selectedLevel = 0;

    qglClearColor(Qt::white);
    glPointSize(4.0);
    glEnable(GL_DEPTH_TEST);
    glGetIntegerv(GL_MAX_3D_TEXTURE_SIZE, &max_res);
    max_depth = i_log2(max_res);
}


void ConturingWidget::bindModel(const Matrix4f& VM, QVector4D color) {
    program.bind();
    Matrix4f PVM = camera.projection * VM;
    program.setUniformValue("uMVPMat",qMat(PVM));
    program.setUniformValue("uNMat", qMat(VM).normalMatrix());
    program.setUniformValue("uColor", color);
}

void ConturingWidget::renderModel(Model* model, const Matrix4f &VM, QVector4D color) {
    bindModel(VM, color);
    model->render(program);
}

void ConturingWidget::bindDebugMesh(Model* model, const Matrix4f& V, bool useVertexColor, QVector4D color) {
    programDebug.bind();
    Matrix4f VM = V * model->getModelMatrix();
    Matrix4f PVM = camera.projection * VM;
    programDebug.setUniformValue("uMVPMat",qMat(PVM));
    programDebug.setUniformValue("useVertexColor", useVertexColor);
    programDebug.setUniformValue("uColor", color);
}

void ConturingWidget::renderDebugMesh(Model* model, const Matrix4f &V, bool useVertexColor, QVector4D color) {
    bindDebugMesh(model, V, useVertexColor, color);
    model->render(programDebug);
}

void ConturingWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    Matrix4f V = camera.getViewMatrix();
    if (model && showModel) {
        Matrix4f VM = V * model->getModelMatrix();
        if (trafo)
            VM *= (*trafo)[trafo_now];
        renderModel(model.get(), VM, QVector4D(0.75,0.75,0.75,1.0));
    }
    if (dmcModel && showOutModel) {
        if (wireframe)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        Matrix4f VM = V * dmcModel->getModelMatrix();
        renderModel(dmcModel.get(), VM, QVector4D(1,0.75,0.0,1.0));
    }
    if (edgeIntersections && showEdgeIntesections) {
        renderDebugMesh(edgeIntersections.get(), V, true);
    }
    if (inGridPoints && showInGridPoints) {
        renderDebugMesh(inGridPoints.get(), V, false, QVector4D(0,1,0,1));
    }
    if (outGridPoints && showOutGridPoints) {
        renderDebugMesh(outGridPoints.get(), V, false, QVector4D(1,0,0,1));
    }
    if (dmcVertices && showDMCVertices) {
        bindDebugMesh(dmcVertices.get(), V, true);
        if (showCells)
            dmcVertices->render(programDebug, v_offset[selectedLevel][selectedCell.x()][selectedCell.y()][selectedCell.z()],
                                v_count[selectedLevel][selectedCell.x()][selectedCell.y()][selectedCell.z()]);
        else {
            dmcVertices->render(programDebug, v_offset[selectedLevel][0][0][0],
                                v_level_count[selectedLevel]);
        }
    }
    if (cells && showCells) {
        uint levelSize = pow2(selectedLevel);
        Vector3f position((float)selectedCell.x()/levelSize, (float)selectedCell.y()/levelSize, (float)selectedCell.z()/levelSize);
        position *= 2*voxelGridRadius;
        cells->setPosition(origin+position);
        bindDebugMesh(cells.get(), V, false);
        cells->render(programDebug, selectedLevel*CELL_EDGES_VERTEX_COUNT, CELL_EDGES_VERTEX_COUNT);
    }
}

void ConturingWidget::resizeGL(int width, int height) {
    w = width;
    h = height;
    glViewport(0,0,w,h);
    if (w > h) {
        float ratio = w/(float) h;
        camera.perspective(45.0f,ratio,0.1f,100.0f);
    } else {
        float ratio = h/(float) w;
        camera.perspective(45.0f,ratio,0.1f,100.0f);
    }
    updateGL();
}

void ConturingWidget::wheelEvent(QWheelEvent* event) {
    int delta = event->delta();
    camera.zoom *= (delta < 0)? CAMERA_SCROLL_FACTOR : 1/CAMERA_SCROLL_FACTOR;
    updateGL();
}

void ConturingWidget::trackballCoord(int x, int y, Vector3f& v) {
#ifdef MAC_QT5_BUG
    x*=2;
    y*=2;
#endif
    if (w > h) {
         v[0] = (2.0f*x-w)/h;
         v[1] = 1.0f-y*2.0f/h;
    } else {
         v[0] = (2.0f*x-w)/w;
         v[1] = (h-2.0f*y)/w;
    }
    double d = v[0]*v[0]+v[1]*v[1];
    if (d > 1.0) {
         v[2] = 0.0;
         v /= sqrt(d);
    } else v[2] = sqrt(1.0-d*d);
}

Quaternionf ConturingWidget::trackball(const Vector3f& u, const Vector3f& v) {
    Vector3f uxv = u.cross(v);
    float uTv = u.transpose() * v;
    Quaternionf ret(1+uTv,uxv.x(), uxv.y(), uxv.z());
    ret.normalize();
    return ret;
}

void ConturingWidget::mousePressEvent(QMouseEvent *event) {
    button = event->button();
    oldX = event->x();
    oldY = event->y();

    updateGL();
}

void ConturingWidget::mouseReleaseEvent(QMouseEvent*) {}

void ConturingWidget::mouseMoveEvent(QMouseEvent* event) {
    int x = event->x();
    int y = event->y();
    if (button == Qt::LeftButton) {
        Vector3f p1,p2;
        trackballCoord(oldX,oldY,p1);
        trackballCoord(x,y,p2);

        Quaternionf q = trackball(p1,p2);
        camera.rotate(q);
    }

    if (button == Qt::RightButton) {
        camera.center += camera.sidewd()*CAMERA_MOVEMENT_SPEED*(oldX-x);
        camera.center += camera.upwd()*CAMERA_MOVEMENT_SPEED*(y-oldY);
    }

    oldX = x;
    oldY = y;

    updateGL();
}

void ConturingWidget::keyPressEvent(QKeyEvent* event) {
    switch (event->key()) {
    case Qt::Key_8: if (selectedCell.z() > 0) selectedCell = selectedCell.shiftZ(-1); break;
    case Qt::Key_2: if (selectedCell.z() < pow2(selectedLevel)-1) selectedCell = selectedCell.shiftZ(1); break;
    case Qt::Key_4: if (selectedCell.x() > 0) selectedCell = selectedCell.shiftX(-1); break;
    case Qt::Key_6: if (selectedCell.x() < pow2(selectedLevel)-1) selectedCell = selectedCell.shiftX(1); break;
    case Qt::Key_7: if (selectedCell.y() > 0) selectedCell = selectedCell.shiftY(-1); break;
    case Qt::Key_9: if (selectedCell.y() < pow2(selectedLevel)-1) selectedCell = selectedCell.shiftY(1); break;
    }
    cout << selectedCell.x() << " " << selectedCell.y() <<  " " << selectedCell.z() << endl;

    updateGL();
}

void ConturingWidget::updateThreshold(float s) {
    errorThreshold = MIN_ERROR_THRESHOLD + (MAX_ERROR_THRESHOLD-MIN_ERROR_THRESHOLD)*s;
}

void ConturingWidget::createCellMesh() {
    aligned_vector3f positions;
    for (int l = 0; l < levels; ++l) {
        // calculate cell corners
        array<Vector3f, 8> corners;
        for (int i = 0; i < 8; ++i) {
            Index corner = corner_delta[i];
            corners[i] = (1.0/pow2(l))*Vector3f(corner.x(), corner.y(), corner.z());
        }
        // push cell edges
        for (int i = 0; i < 12; ++i) {
            positions.push_back(corners[edge_corners[i][0]]);
            positions.push_back(corners[edge_corners[i][1]]);
        }
    }
    cells = unique_ptr<Model>(new Model(positions, GL_LINES, false));
    cells->setPosition(origin);
    cells->scale = 2*voxelGridRadius;
}

void ConturingWidget::updateDMCMesh() {
    if (!dmcModel) {
        main->statusBar()->showMessage("No model to simplify!");
        cout << "No model to simplify!" << endl;
    }
    main->statusBar()->showMessage(("simplify (t = " + to_string(errorThreshold) + ")...").c_str());
    cout << ("simplify (t = " + to_string(errorThreshold) + ")...") << endl;
    DMC.collapse(errorThreshold);
/*
    main->statusBar()->showMessage("generate vertex model...");
    aligned_vector3f positions, colors;
    v_offset.clear();
    v_count.clear();
    DMC->vertices(positions, colors, v_offset, v_count);
    dmcVertices = unique_ptr<Model>(new Model(positions, colors, false, GL_POINTS));
    dmcVertices->setPosition(origin);
    dmcVertices->scale = 2*cellGridRadius;
*/
    createDMCMesh();
    updateGL();

}

void ConturingWidget::createDMCMesh() {
    positions.clear();
    indices.clear();
    main->statusBar()->showMessage("create DMC Mesh...");
    cout << "create DMC Mesh..." << endl;
    DMC.createMesh(positions, indices);
    dmcModel = unique_ptr<Model>(new Model(positions, indices, false));
    dmcModel->setPosition(origin);
    dmcModel->scale = 2*voxelGridRadius;
    main->statusBar()->showMessage(("Done. "
                                   + to_string(positions.size()) + " Vertices, "
                                   + to_string(indices.size()/3) + " Triangles").c_str());
    cout << ("Done. "
                  + to_string(positions.size()) + " Vertices, "
                  + to_string(indices.size()/3) + " Triangles").c_str() << endl;
}

void ConturingWidget::compute() {
    if (!model) {
        main->statusBar()->showMessage("No input model!");
        cout << "No input model!" << endl;
        return;
    }
    updateRenderStrategy();
    bool ok = false;
    int depth = QInputDialog::getInt(this, "Resolution", "Input the octree depth", MIN_OCTREE_DEPTH, MIN_OCTREE_DEPTH, max_depth, 1, &ok);
    if (!ok)
        return;
    res = pow2(depth);
    levels = depth+1;

    //cellGridRadius = voxelGridRadius-voxelGridRadius/(res+1);
    origin = Vector3f(-voxelGridRadius,-voxelGridRadius,-voxelGridRadius);
    v_level_count = vector<uint>(levels, 0);
    cell_level_count = vector<uint>(levels, 0);
    createCellMesh();


    DMC.conturing(scene.get(), voxelGridRadius, res);
    createDMCMesh();
/*
    DMC.collapse(0.0f);

    aligned_vector3f positions, colors;

    DMC.signSampler->inside(voxelGridRadius, positions);
    inGridPoints = unique_ptr<Model>(new Model(positions, GL_POINTS, false));
    positions.clear();
    colors.clear();

    std::cout << " get vertices " << std::endl;
    DMC.vertices(positions, colors, v_offset, v_count);
    dmcVertices = unique_ptr<Model>(new Model(positions, colors, false, GL_POINTS));
    dmcVertices->setPosition(origin);
    dmcVertices->scale = 2*voxelGridRadius;
    for (int i = 0; i < levels; ++i) {
        uint levelSize = pow2(i);
        for (uint x = 0; x < levelSize; ++x)
            for (uint y = 0; y < levelSize; ++y)
                for (uint z = 0; z < levelSize; ++z)
                    v_level_count[i] += v_count[i][x][y][z];
    }


    positions.clear();
    colors.clear();
    DMC.sampler->edgeIntersections(voxelGridRadius, positions, colors);
    edgeIntersections = unique_ptr<Model>(new Model(positions, colors, false, GL_POINTS));


    main->statusBar()->showMessage("generate vertex model...");
    std::cout << "generate vertex model..." << std::endl;
    positions.clear();
    colors.clear();
    DMC.vertices(positions, colors, v_offset, v_count);
    dmcVertices = unique_ptr<Model>(new Model(positions, colors, false, GL_POINTS));
    dmcVertices->setPosition(origin);
    dmcVertices->scale = 2*voxelGridRadius;
    */
    resizeGL(w,h);
}

int main (int argc, char **argv) {
    QApplication app(argc, argv);

    if (!QGLFormat::hasOpenGL()) {
        qWarning ("This system has no OpenGL support. Exiting.");
        return 1;
    }

    w = new CGMainWindow(NULL);

    w->show();

    int status = app.exec();

    delete w;

    return status;

}

