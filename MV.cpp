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
    //file->addAction ("Load track", ogl, SLOT(loadTrack()), Qt::CTRL+Qt::Key_T);
    file->addAction ("Save model", ogl, SLOT(storeModel()), Qt::CTRL+Qt::Key_S);
    file->addAction ("DMC", ogl, SLOT(dmc()), Qt::CTRL+Qt::Key_D);
    file->addAction ("Quit", qApp, SLOT(quit()), Qt::CTRL+Qt::Key_Q);

    menuBar()->addMenu(file);               

    QMenu *view = new QMenu("&View",this);
    view->addAction ("Model", ogl, SLOT(toggleViewModel()), Qt::Key_M)->setCheckable(true);
    view->addAction ("Edge Intersections", ogl, SLOT(toggleViewEdgeIntersections()), Qt::Key_E)->setCheckable(true);
    view->addAction ("Inner Grid", ogl, SLOT(toggleViewInGridPoints()), Qt::Key_I)->setCheckable(true);
    view->addAction ("Vertices", ogl, SLOT(toggleViewDMCVertices()), Qt::Key_V)->setCheckable(true);
    view->addAction ("DMC Mesh", ogl, SLOT(toggleViewDMCModel()), Qt::Key_D)->setCheckable(true);
    view->addAction ("Wireframe", ogl, SLOT(toggleWireframe()), Qt::Key_W)->setCheckable(true);
    view->addAction ("Cells", ogl, SLOT(toggleViewCells()), Qt::Key_C)->setCheckable(true);
    view->addAction ("Higher Level", ogl, SLOT(decreaseSelectedLevel()), Qt::Key_Plus);
    view->addAction ("Lower Level", ogl, SLOT(increaseSelectedLevel()), Qt::Key_Minus);
    view->addAction ("Center Camera", ogl, SLOT(centerCamera()), Qt::Key_Comma);

    menuBar()->addMenu(view);


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
void getFromStlFile(std::vector<QVector3D>& triangles, const char *filename) {
    std::ifstream instream(filename,std::ios::binary);
    if (!instream) {
        std::cerr << "file does not exist!" << std::endl;
        return;
    }

    instream.seekg( 80, std::ios_base::beg ); // skip ascii header
    int trinum = 0;
    instream.read((char*) &trinum, 4 ); // number of triangles
    float tmp;
    for(int k = 0; k < trinum; k++) {
        for(int i=0;i < 3 ; i++ )
            instream.read( (char*) &tmp, 4 );
        for(int i = 0; i < 3; i++ ) {
		qreal v[3];
            for(int j = 0 ; j < 3 ; j++) {
                instream.read( (char*) &tmp, 4 );
                v[j] = tmp;
            }
            triangles.push_back(1000.0*QVector3D(v[0],v[1],v[2]));
        }
        instream.read( (char*) &tmp, 2);
    }

    instream.close();
}

void getFromVdaFile( std::vector<QMatrix4x4>& trafo, const char* filename, qreal& timestep ) {
    std::setlocale(LC_NUMERIC,"C");

    std::ifstream vdafile( filename );
    if (!vdafile) {
        std::cerr << "getFromVdaFile: Cannot open vda-file" << std::endl;
        return;
    }

    trafo.clear();

    std::string s,t;
    size_t pos1,pos2,pos3;

    getline(vdafile,s);
    getline(vdafile,s);
    getline(vdafile,s);

    if (s.find("DT")) {
            pos1 = s.find(".");
            pos1 = s.find(".",pos1+1);
            t = s.substr(pos1,pos1+5);
            timestep = atof(t.data());
    }

    getline(vdafile,s);

    QMatrix4x4 M;
    M.setToIdentity();

    int n = 0;
    while (!vdafile.eof()) {
            getline(vdafile,s);

            if (s.find("TMAT") == std::string::npos) {
                    // std::cout << "end of file" << std::endl;
                    break;
            }

            for(int i=0;i<3;i++) {
                    getline(vdafile,s);
                    pos1 = s.find(",");
                    t = s.substr(0,pos1);
                    M(i,0) = atof(t.data());
                    pos2 = s.find(",",pos1+1);
                    t = s.substr(pos1+1,pos2-pos1-1);
                    M(i,1) = atof(t.data());
                    pos3 = s.find(",",pos2+1);
                    t = s.substr(pos2+1,pos3-pos2-1);
                    M(i,2) = atof(t.data());
            }

            getline(vdafile,s);
            pos1 = s.find(",");
            t = s.substr(0,pos1);
            M(0,3) = atof(t.data());
            pos2 = s.find(",",pos1+1);
            t = s.substr(pos1+1,pos2-pos1-1);
            M(1,3) = atof(t.data());
            pos3 = s.find(" ",pos2+1);
            t = s.substr(pos2+1,pos3-pos2-1);
            M(2,3) = atof(t.data());

            trafo.push_back(M);
            n++;
    }

    vdafile.close();

    std::cout << "Loaded " << filename << " with " << trafo.size() << " transformations" << std::endl;
}

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
    if (!initShaderProgram(":/shaders/vEdgeScan.glsl", ":/shaders/fEdgeScan.glsl", programEdgeScan))
        return false;
    if (!initShaderProgram(":/shaders/vHermiteScan.glsl", ":/shaders/fHermiteScan.glsl", programHermiteScan))
        return false;
    return true;
}

void DefaultRenderStrategy::subroutineSelection(GLuint index[2]) {
    index[0] = 0;
    index[1] = 2;
}

void DefaultRenderStrategy::render(QGLShaderProgram &program, const QMatrix4x4 &projection, const QMatrix4x4 &view) {
    GLuint index[2];
    subroutineSelection(index);
    GLint subroutine_count = 0;
    glGetProgramStageiv(program.programId(), GL_VERTEX_SHADER, GL_ACTIVE_SUBROUTINE_UNIFORM_LOCATIONS, &subroutine_count);
    glUniformSubroutinesuiv(GL_VERTEX_SHADER, subroutine_count, &index[0]);
    doRender(program, projection, view);
}

RenderSingleModel::RenderSingleModel(const Model *model) : model(model) {
    initializeOpenGLFunctions();
}


void RenderSingleModel::doRender(QGLShaderProgram &program, const QMatrix4x4 &projection, const QMatrix4x4 &view) {
    QMatrix4x4 PV = projection * view;
    Matrix4f M = model->getModelMatrix();
    QMatrix4x4 qM = qMat(M);
    program.setUniformValue("uMVPMat", PV*qM);
    program.setUniformValue("uNMat", qM.normalMatrix());
    model->render(program);
}


RenderTrafoModel::RenderTrafoModel(const Model* model, const Trafo* trafo, int progress_update_instances)
        : model(model), trafo(trafo), progress_update_instances(progress_update_instances) {
    initializeOpenGLFunctions();
}

void RenderTrafoModel::doRender(QGLShaderProgram &program, const QMatrix4x4 &projection, const QMatrix4x4 &view) {
    QMatrix4x4 PV = projection * view;
    for (uint i = 0; i < trafo->size(); ++i) {
        Matrix4f M_trafo = model->getModelMatrix() * (*trafo)[i];
        QMatrix4x4 qM = qMat(M_trafo);
        program.setUniformValue("uMVPMat", PV * qM);
        program.setUniformValue("uNMat", qM.normalMatrix());
        model->render(program);
        glFinish();
        glFlush();
        if (i % progress_update_instances == 0)
            std::cout << "  rendering " << i << "/" << trafo->size() << "\r" << std::flush;
    }
    std::cout << "  rendering done." << trafo->size() << "/" << trafo->size() << std::endl;
}

RenderTrafoModelInstanced::RenderTrafoModelInstanced(const Model* model, const Trafo* trafo, int max_instances)
        : model(model), trafo(trafo), trafo_buffer(1, trafo->size()*64, GL_DYNAMIC_DRAW),
          trafo_normal_buffer(2, trafo->size()*48, GL_STATIC_DRAW), max_instances(max_instances) {
    initializeOpenGLFunctions();
    std::cout << "fill SSBO with normal matrices" << std::endl;
    trafo_normal_buffer.bind();
    for (uint i = 0; i < trafo->size(); ++i) {
        Matrix4f M_trafo = model->getModelMatrix() * (*trafo)[i];
        QMatrix3x3 nMat = qMat(M_trafo).normalMatrix();
        QMatrix4x4 padded_nMat = QMatrix4x4(nMat);
        trafo_normal_buffer.bufferSubData(i*48, 48, padded_nMat.data());
    }
    trafo_normal_buffer.unBind();
}

void RenderTrafoModelInstanced::fillBuffer(const QMatrix4x4 &projection, const QMatrix4x4 &view) {
    std::cout << "  fill SSBO with transformation matrices" << std::endl;
    QMatrix4x4 PV = projection * view;
    trafo_buffer.bind();
    for (uint i = 0; i < trafo->size(); ++i) {
        Matrix4f M_trafo = model->getModelMatrix() * (*trafo)[i];
        QMatrix4x4 mvpMat = PV * qMat(M_trafo);
        trafo_buffer.bufferSubData(i*64, 64, mvpMat.data());
    }
    trafo_buffer.unBind();
}

void RenderTrafoModelInstanced::subroutineSelection(GLuint index[2]) {
    index[0] = 1;
    index[1] = 3;
}

void RenderTrafoModelInstanced::doRender(QGLShaderProgram &program, const QMatrix4x4 &projection, const QMatrix4x4 &view) {
    fillBuffer(projection, view);

    // render in instanced groups of size max_instance
    //(too many instance at once slows down performance or even crashes the program)
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
    std::cout << "  rendering done." << trafo->size() << "/" << trafo->size() << std::endl;
}

void ConturingWidget::storeModel() {
    if (dmcModel) {
        writeToOffFile(positions, indices, "D:/Sonstiges/Uni_Schule/CG_HIWI/MV/mv2/out/motor.off");
        main->statusBar()->showMessage ("Saving model done.",3000);
    }
}

void ConturingWidget::loadTrack() {
// QString filename = QFileDialog::getOpenFileName(this, "Load track ...", QString(), "(*.vda)" );

// if (filename.isEmpty()) return;
// statusBar()->showMessage ("Loading track ...");

// if (filename.endsWith(".vda")) {
//     std::ifstream trackfile(filename.toLatin1());
// }
    trafo = unique_ptr<Trafo>(new Trafo());
    trafo->scale = 1000.0f;
    LoadVdaFile(trafo->M,"D:/Sonstiges/Uni_Schule/CG_HIWI/MV/mv2/tracks/track.vda",trafo->scale, timestep);
    updateTrafoModel();
}

void ConturingWidget::loadModel() {
/*
    QString filename = QFileDialog::getOpenFileName(this, "Load model ...", QString(), "(*.stl *.off)" );

    if (filename.isEmpty()) return;
    statusBar()->showMessage ("Loading model ...");

    aligned_vector3f positions, normals;
    //if (filename.endsWith(".stl"))
    //    getFromStlFile(ogl->triangles,filename.toLatin1());

    if (filename.endsWith(".off"))
        LoadOffFile(filename.toLatin1(), positions, normals);
*/
    aligned_vector3f positions, normals;
    LoadOffFile("D:/Sonstiges/Uni_Schule/CG_HIWI/MV/mv2/models/motor.off", positions, normals);
    model = unique_ptr<Model>(new Model(positions, normals, GL_TRIANGLES, true, 2.0f));
    updateTrafoModel();
    camera.position = camera.rotation._transformVector(Z_AXIS) + camera.center;
    main->statusBar()->showMessage ("Loading model done.",3000);
    showModel = true;
    updateGL();
}

void ConturingWidget::sliderValueChanged(int value) {
    updateThreshold((float)value/CGMainWindow::SLIDER_GRANULARITY);
    main->t_error_label->setText(to_string(errorThreshold).c_str());
}

const float ConturingWidget::CAMERA_MOVEMENT_SPEED = 0.0025f;
const float ConturingWidget::CAMERA_SCROLL_FACTOR = 1.2f;
const float ConturingWidget::MIN_ERROR_THRESHOLD = 0.0f;
const float ConturingWidget::MAX_ERROR_THRESHOLD = 0.0001f;

ConturingWidget::ConturingWidget (CGMainWindow *mainwindow,QWidget* parent ) : QGLWidget (parent) {
    main = mainwindow;
    errorThreshold = MIN_ERROR_THRESHOLD;
    res = DEFAULT_RESOLUTION;
}

void ConturingWidget::initializeGL() {
    initializeOpenGLFunctions();
    initShaderProgram(":/shaders/vshader.glsl", ":/shaders/fshader.glsl", program);
    initShaderProgram(":/shaders/vpoints.glsl", ":/shaders/fpoints.glsl", programDebug);

    qglClearColor(Qt::black);
    glPointSize(4.0);
    glEnable(GL_DEPTH_TEST);

    //GLint max_size = 0;
    //glGetIntegerv(GL_MAX_3D_TEXTURE_SIZE, &max_size);
    //std::cout << "GL_MAX_3D_TEXTURE_SIZE " << max_size << std::endl;

    camera.position = Z_AXIS;
    rotX = rotY = 0;
    zoom = 1.0f;

    levels = i_log2(res)+1;
    voxelGridRadius = 1.0f;
    cellGridRadius = voxelGridRadius-voxelGridRadius/(res+1);
    origin = Vector3f(-cellGridRadius,-cellGridRadius,-cellGridRadius);
    v_level_count = vector<uint>(levels, 0);
    cell_level_count = vector<uint>(levels, 0);

    trafo_now = 0;
    loadTrack();
    loadModel();

    showModel = false;
    showDMCModel = false;
    showEdgeIntesections = false;
    showInGridPoints = false;
    showOutGridPoints = false;
    showDMCVertices = false;
    showCells = false;
    wireframe = false;
    selectedLevel = 0;
}

void ConturingWidget::updateTrafoModel() {
    if (model) {
        if (trafo) {
            model->init(2.0f, *trafo);
            scene = unique_ptr<RenderStrategy>(new RenderTrafoModelInstanced(model.get(), trafo.get(), MAX_INSTANCES));
            //scene = unique_ptr<RenderStrategy>(new RenderTrafoModel(model.get(), trafo.get(), MAX_INSTANCES));
        } else {
            scene = unique_ptr<RenderStrategy>(new RenderSingleModel(model.get()));
        }
    }

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
    if (dmcModel && showDMCModel) {
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
            dmcVertices->render(programDebug, v_offset[selectedLevel][selectedCell.x][selectedCell.y][selectedCell.z],
                                v_count[selectedLevel][selectedCell.x][selectedCell.y][selectedCell.z]);
        else {
            dmcVertices->render(programDebug, v_offset[selectedLevel][0][0][0],
                                v_level_count[selectedLevel]);
        }
    }
    if (cells && showCells) {
        uint levelSize = pow2(selectedLevel);
        Vector3f position((float)selectedCell.x/levelSize, (float)selectedCell.y/levelSize, (float)selectedCell.z/levelSize);
        position *= 2*cellGridRadius;
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
    zoom *= (delta < 0)? CAMERA_SCROLL_FACTOR : 1/CAMERA_SCROLL_FACTOR;
    camera.position = camera.rotation._transformVector(zoom*Z_AXIS) + camera.center;
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
        rotX -= 0.5*(y-oldY)*M_PI/180.0;
        rotY -= 0.5*(x-oldX)*M_PI/180.0;
        Quaternionf qy(AngleAxisf(rotY, Vector3f(0,1,0)));
        Quaternionf qx(AngleAxisf(rotX, Vector3f(1,0,0)));

        camera.setRotation(qy * qx);
        camera.position = camera.center - camera.forwd()*zoom;
    }

    if (button == Qt::RightButton) {
        camera.center += camera.sidewd()*CAMERA_MOVEMENT_SPEED*(oldX-x);
        camera.center += camera.upwd()*CAMERA_MOVEMENT_SPEED*(y-oldY);
        camera.translate(camera.sidewd()*CAMERA_MOVEMENT_SPEED*(oldX-x));
        camera.translate(camera.upwd()*CAMERA_MOVEMENT_SPEED*(y-oldY));
    }

    oldX = x;
    oldY = y;

    updateGL();
}

void ConturingWidget::keyPressEvent(QKeyEvent* event) {
    switch (event->key()) {
    case Qt::Key_8: if (selectedCell.z > 0) selectedCell = selectedCell.shiftZ(-1); break;
    case Qt::Key_2: if (selectedCell.z < pow2(selectedLevel)-1) selectedCell = selectedCell.shiftZ(1); break;
    case Qt::Key_4: if (selectedCell.x > 0) selectedCell = selectedCell.shiftX(-1); break;
    case Qt::Key_6: if (selectedCell.x < pow2(selectedLevel)-1) selectedCell = selectedCell.shiftX(1); break;
    case Qt::Key_7: if (selectedCell.y > 0) selectedCell = selectedCell.shiftY(-1); break;
    case Qt::Key_9: if (selectedCell.y < pow2(selectedLevel)-1) selectedCell = selectedCell.shiftY(1); break;
    }
    cout << selectedCell.x << " " << selectedCell.y <<  " " << selectedCell.z << endl;

    switch (event->key()) {
    case Qt::Key_Right: trafo_now += 1; if (trafo_now >= (int) trafo->size()) trafo_now = trafo->size()-1; break;
    case Qt::Key_Left: trafo_now -= 1; if (trafo_now < 0) trafo_now = 0; break;
    case Qt::Key_Up: trafo_now += 10; if (trafo_now >= (int) trafo->size()) trafo_now = trafo->size()-1; break;
    case Qt::Key_Down: trafo_now -= 10; if (trafo_now < 0) trafo_now = 0; break;
    case Qt::Key_PageUp: trafo_now += 1000; if (trafo_now >= (int) trafo->size()) trafo_now = trafo->size()-1; break;
    case Qt::Key_PageDown: trafo_now -= 1000; if (trafo_now < 0) trafo_now = 0; break;
    }
    std::cout << trafo_now << std::endl;

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
            corners[i] = (1.0/pow2(l))*Vector3f(corner.x, corner.y, corner.z);
        }
        // push cell edges
        for (int i = 0; i < 12; ++i) {
            positions.push_back(corners[edge_corners[i][0]]);
            positions.push_back(corners[edge_corners[i][1]]);
        }
    }
    cells = unique_ptr<Model>(new Model(positions, GL_LINES, false));
    cells->setPosition(origin);
    cells->scale = 2*cellGridRadius;
}

void ConturingWidget::updateDMCMesh() {
    if (DMC) {
        main->statusBar()->showMessage(("simplify (t = " + to_string(errorThreshold) + ")...").c_str());
        DMC->collapse(errorThreshold);
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
}

void ConturingWidget::createDMCMesh() {
    positions.clear();
    indices.clear();
    main->statusBar()->showMessage("create DMC Mesh...");
    std::cout << "create DMC Mesh..." << std::endl;
    DMC->createMesh(positions, indices);
    dmcModel = unique_ptr<Model>(new Model(positions, indices, false));
    dmcModel->setPosition(origin);
    dmcModel->scale = 2*cellGridRadius;
    main->statusBar()->showMessage(("Done. "
                                   + to_string(positions.size()) + " Vertices, "
                                   + to_string(indices.size()/3) + " Triangles").c_str());
    std::cout << ("Done. "
                  + to_string(positions.size()) + " Vertices, "
                  + to_string(indices.size()/3) + " Triangles").c_str() << std::endl;
}

void ConturingWidget::dmc() {
    aligned_vector3f positions, colors;
    DMC = unique_ptr<DualMarchingCubes>(new DualMarchingCubes());
    DMC->conturing(scene.get(), voxelGridRadius, DEFAULT_RESOLUTION, DEFAULT_WORK_RESOLUTION);
    //DMC->collapse(0.0f);
    //DMC->signSampler->inside(voxelGridRadius, positions);
    //inGridPoints = unique_ptr<Model>(new Model(positions, GL_POINTS, false));
    positions.clear();
    colors.clear();
    /*
    std::cout << " get vertices " << std::endl;
    DMC->vertices(positions, colors, v_offset, v_count);
    dmcVertices = unique_ptr<Model>(new Model(positions, colors, false, GL_POINTS));
    dmcVertices->setPosition(origin);
    dmcVertices->scale = 2*cellGridRadius;
    for (int i = 0; i < levels; ++i) {
        uint levelSize = pow2(i);
        for (uint x = 0; x < levelSize; ++x)
            for (uint y = 0; y < levelSize; ++y)
                for (uint z = 0; z < levelSize; ++z)
                    v_level_count[i] += v_count[i][x][y][z];
    }*/
    createDMCMesh();
    //positions.clear();
    //colors.clear();
    //DMC->sampler->edgeIntersections(voxelGridRadius, positions, colors);
    //edgeIntersections = unique_ptr<Model>(new Model(positions, colors, false, GL_POINTS));
/*
    main->statusBar()->showMessage("generate vertex model...");
    std::cout << "generate vertex model..." << std::endl;
    positions.clear();
    colors.clear();
    DMC->vertices(positions, colors, v_offset, v_count);
    dmcVertices = unique_ptr<Model>(new Model(positions, colors, false, GL_POINTS));
    dmcVertices->setPosition(origin);
    dmcVertices->scale = 2*cellGridRadius;
*/
    //inGridPoints = unique_ptr<Model>(new Model(positions, colors, false, GL_LINES));


    /*

    CompressedHermiteScanner* scan = new CompressedHermiteScanner(FRONT_AND_BACK_XYZ, res, voxelGridRadius, programScan);
    QMatrix4x4 V;
    Matrix4f modelMat = model->getModelMatrix();

    scan->begin(FRONT_AND_BACK_FACES_X, V);
    main->statusBar()->showMessage("scan model x-axis...");
    std::cout << "scan model x-axis..." << std::endl;
    scanModel(scan->projection, V, modelMat);
    std::cout << "read x-axis data from graphic memory..." << std::endl;
    main->statusBar()->showMessage("read x-axis data from graphic memory...");
    scan->end();

    scan->begin(FRONT_AND_BACK_FACES_Y, V);
    main->statusBar()->showMessage("scan model y-axis...");
    std::cout << "scan model y-axis..." << std::endl;
    scanModel(scan->projection, V, modelMat);
    std::cout << "read y-axis data from graphic memory..." << std::endl;
    main->statusBar()->showMessage("read y-axis data from graphic memory...");
    scan->end();

    scan->begin(FRONT_AND_BACK_FACES_Z, V);
    main->statusBar()->showMessage("scan model z-axis...");
    std::cout << "scan model z-axis..." << std::endl;
    scanModel(scan->projection, V, modelMat);
    std::cout << "read z-axis data from graphic memory..." << std::endl;
    main->statusBar()->showMessage("read z-axis data from graphic memory...");
    scan->end();

    main->statusBar()->showMessage("create sampler...");
    std::cout << "create sampler..." << std::endl;
    CompressedHermiteSampler* sampler = new CompressedHermiteSampler(scan->data.get());

    aligned_vector3f positions, colors;
    vector<uint> indices;

    main->statusBar()->showMessage("generate edge intersection model");
    std::cout << "generate edge intersection model" << std::endl;
    sampler->edgeIntersections(voxelGridRadius, positions, colors);
    edgeIntersections = unique_ptr<Model>(new Model(positions, colors, false, GL_POINTS));

    std::cout << "generate in grid model" << std::endl;
    main->statusBar()->showMessage("generate in grid model");
    positions.clear();
    sampler->inside(voxelGridRadius, positions);
    inGridPoints = unique_ptr<Model>(new Model(positions, GL_POINTS, false));

    std::cout << "generate out grid model" << std::endl;
    positions.clear();
    sampler.outside(voxelGridRadius, positions);
    outGridPoints = unique_ptr<Model>(new Model(positions, GL_POINTS, false));


    DMC = unique_ptr<DualMarchingCubes>(new DualMarchingCubes(sampler));
    main->statusBar()->showMessage("create octree...");
    std::cout << "create octree..." << std::endl;
    DMC->createOctree();
    main->statusBar()->showMessage("create vertex tree...");
    std::cout << "create vertex tree..." << std::endl;
    DMC->createVertexTree();

    main->statusBar()->showMessage(("simplify (t = " + to_string(errorThreshold) + ")...").c_str());
    std::cout << ("simplify (t = " + to_string(errorThreshold) + ")...").c_str() << std::endl;
    DMC->collapse(errorThreshold);

    main->statusBar()->showMessage("generate vertex model...");
    std::cout << "generate vertex model..." << std::endl;
    positions.clear();
    colors.clear();
    DMC->vertices(positions, colors, v_offset, v_count);
    dmcVertices = unique_ptr<Model>(new Model(positions, colors, false, GL_POINTS));
    dmcVertices->setPosition(origin);
    dmcVertices->scale = 2*cellGridRadius;

    main->statusBar()->showMessage("generate cell model...");
    positions.clear();
    DMC->cells(positions, cell_offset);
    cells = unique_ptr<Model>(new Model(positions, GL_LINES, false));
    cells->setPosition(origin);
    cells->scale = 2*cellGridRadius;

    for (int i = 0; i < levels; ++i) {
        uint levelSize = pow2(i);
        for (uint x = 0; x < levelSize; ++x)
            for (uint y = 0; y < levelSize; ++y)
                for (uint z = 0; z < levelSize; ++z)
                    v_level_count[i] += v_count[i][x][y][z];
    }

    positions.clear();
    indices.clear();
    colors.clear();


    createDMCMesh();


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

