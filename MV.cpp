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
    file->addAction ("Load model", this, SLOT(loadModel()), Qt::CTRL+Qt::Key_L);
    //file->addAction ("Load track", this, SLOT(loadTrack()), Qt::CTRL+Qt::Key_T);
    file->addAction ("DMC", ogl, SLOT(dmc()), Qt::CTRL+Qt::Key_D);
    file->addAction ("Quit", qApp, SLOT(quit()), Qt::CTRL+Qt::Key_Q);

    menuBar()->addMenu(file);               

    QMenu *view = new QMenu("&View",this);
    view->addAction ("Model", ogl, SLOT(toggleViewModel()), Qt::Key_M)->setCheckable(true);
    view->addAction ("Edge Intersections", ogl, SLOT(toggleViewEdgeIntersections()), Qt::Key_E)->setCheckable(true);
    view->addAction ("Inner Grid", ogl, SLOT(toggleViewInGridPoints()), Qt::Key_I)->setCheckable(true);
    view->addAction ("Vertices", ogl, SLOT(toggleViewDMCVertices()), Qt::Key_V)->setCheckable(true);
    view->addAction ("DMC Mesh", ogl, SLOT(toggleViewDMCModel()), Qt::Key_D)->setCheckable(true);
    view->addAction ("Cells", ogl, SLOT(toggleViewCells()), Qt::Key_C)->setCheckable(true);
    view->addAction ("Toggle Level/Cell", ogl, SLOT(toggleViewCellLevel()), Qt::Key_L);
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

    connect(threshold_slider,SIGNAL(valueChanged(int)),this,SLOT(sliderValueChanged(int)));
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

void writeToOffFile(std::vector<QVector3D>& vertex, std::vector<int>& face,const char *filename) {
    std::ofstream offFile(filename);

    offFile << "OFF" << std::endl;
    offFile << vertex.size() << " " << face.size()/3 << " " << vertex.size()+face.size()/3-2 << std::endl;

    for(size_t i=0;i<vertex.size();i++) {
        const QVector3D& a = vertex[i];
        offFile << a.x() << " " << a.y() << " " << a.z() << std::endl;
    }

    for(size_t i=0;i<face.size()/3;i++)
        offFile << 3 << " " << face[3*i+0] << " " << face[3*i+1] << " " << face[3*i+2] << std::endl;

    offFile.close();
}
*/
void CGMainWindow::loadTrack() {
// QString filename = QFileDialog::getOpenFileName(this, "Load track ...", QString(), "(*.vda)" );

// if (filename.isEmpty()) return;
// statusBar()->showMessage ("Loading track ...");

// if (filename.endsWith(".vda")) {
//     std::ifstream trackfile(filename.toLatin1());
// }

    LoadVdaFile(ogl->trafo.M,"D:/Sonstiges/Uni_Schule/CG_HIWI/MV/mv2/tracks/track.vda",ogl->trafo.scale, ogl->timestep);
    ogl->updateTrafoModel();
}

void CGMainWindow::loadModel() {
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
    ogl->model = unique_ptr<Model>(new Model(positions, normals, GL_TRIANGLES, true, 1.8));
    ogl->updateTrafoModel();
    ogl->camera.position = ogl->camera.rotation._transformVector(Z_AXIS) + ogl->camera.center;
    statusBar()->showMessage ("Loading model done.",3000);
    ogl->showModel = true;
    ogl->updateGL();
}

void CGMainWindow::sliderValueChanged(int value) {
    ogl->updateThreshold((float)value/SLIDER_GRANULARITY);
    t_error_label->setText(to_string(ogl->errorThreshold).c_str());
}

const float ConturingWidget::CAMERA_MOVEMENT_SPEED = 0.0025;
const float ConturingWidget::CAMERA_SCROLL_FACTOR = 1.2;
const float ConturingWidget::MIN_ERROR_THRESHOLD = 0;
const float ConturingWidget::MAX_ERROR_THRESHOLD = 0.001;

ConturingWidget::ConturingWidget (CGMainWindow *mainwindow,QWidget* parent ) : QGLWidget (parent) {
    main = mainwindow;
    errorThreshold = MIN_ERROR_THRESHOLD;
    res = DEFAULT_RESOLUTION;
}

bool ConturingWidget::initShaderProgram(const char *vname, const char *fname, QGLShaderProgram& program) {
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

void ConturingWidget::initializeGL() {
    initializeOpenGLFunctions();
    initShaderProgram(":/shaders/vshader.glsl", ":/shaders/fshader.glsl", program);
    initShaderProgram(":/shaders/vpoints.glsl", ":/shaders/fpoints.glsl", programColor);
    initShaderProgram(":/shaders/scan_vertShader.glsl", ":/shaders/scan_fragShader.glsl", programScan);

    qglClearColor(Qt::black);
    glPointSize(4.0);
    glEnable(GL_DEPTH_TEST);

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
    trafo.scale = 1000.0;
    //main->loadTrack();
    main->loadModel();
    showModel = false;
    showDMCModel = false;
    showEdgeIntesections = false;
    showInGridPoints = false;
    showOutGridPoints = false;
    showDMCVertices = false;
    showCells = false;
    showSingleCell = false;
    selectedLevel = 0;
}

void ConturingWidget::updateTrafoModel() {
    if (!trafo.empty() && model) {
        model->scale /= trafo.scale;
        model->center = (trafo[0] * model->center.homogeneous()).block<3,1>(0,0);
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
    programColor.bind();
    Matrix4f VM = V * model->getModelMatrix();
    Matrix4f PVM = camera.projection * VM;
    programColor.setUniformValue("uMVPMat",qMat(PVM));
    programColor.setUniformValue("useVertexColor", useVertexColor);
    programColor.setUniformValue("uColor", color);
}

void ConturingWidget::renderDebugMesh(Model* model, const Matrix4f &V, bool useVertexColor, QVector4D color) {
    bindDebugMesh(model, V, useVertexColor, color);
    model->render(programColor);
}

void ConturingWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    Matrix4f V = camera.getViewMatrix();
    if (model && showModel) {
        Matrix4f VM = V * model->getModelMatrix();
        if (!trafo.empty())
            VM *= trafo[trafo_now];
        renderModel(model.get(), VM, QVector4D(0.75,0.75,0.75,1.0));
    }
    if (dmcModel && showDMCModel) {
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
        if (showSingleCell)
            dmcVertices->render(programColor, v_offset[selectedLevel][selectedCell.x][selectedCell.y][selectedCell.z],
                                v_count[selectedLevel][selectedCell.x][selectedCell.y][selectedCell.z]);
        else {
            dmcVertices->render(programColor, v_offset[selectedLevel][0][0][0],
                                v_level_count[selectedLevel]);
        }
    }
    if (cells && showCells) {
        bindDebugMesh(cells.get(), V, false);
        if (showSingleCell)
            cells->render(programColor, cell_offset[selectedLevel][selectedCell.x][selectedCell.y][selectedCell.z],
                             CELL_EDGES_VERTEX_COUNT);
        else {
            cells->render(programColor, cell_offset[selectedLevel][0][0][0],
                                cell_level_count[selectedLevel]);
        }
    }
}

void ConturingWidget::resizeGL(int width, int height) {
    w = width;
    h = height;
    glViewport(0,0,w,h);
    if (w > h) {
        float ratio = w/(float) h;
        camera.perspective(45.0f,ratio,0.1,100);
    } else {
        float ratio = h/(float) w;
        camera.perspective(45.0f,ratio,0.1,100);
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
    if (showSingleCell) {
        switch (event->key()) {
        case Qt::Key_8: if (selectedCell.z > 0) selectedCell = selectedCell.shiftZ(-1); break;
        case Qt::Key_2: if (selectedCell.z < pow2(selectedLevel)-1) selectedCell = selectedCell.shiftZ(1); break;
        case Qt::Key_4: if (selectedCell.x > 0) selectedCell = selectedCell.shiftX(-1); break;
        case Qt::Key_6: if (selectedCell.x < pow2(selectedLevel)-1) selectedCell = selectedCell.shiftX(1); break;
        case Qt::Key_7: if (selectedCell.y > 0) selectedCell = selectedCell.shiftY(-1); break;
        case Qt::Key_9: if (selectedCell.y < pow2(selectedLevel)-1) selectedCell = selectedCell.shiftY(1); break;
        }
        cout << selectedCell.x << " " << selectedCell.y <<  " " << selectedCell.z << endl;
    }

    switch (event->key()) {
    case Qt::Key_Right: trafo_now += 1; if (trafo_now >= (int) trafo.size()) trafo_now = trafo.size()-1; break;
    case Qt::Key_Left: trafo_now -= 1; if (trafo_now < 0) trafo_now = 0; break;
    case Qt::Key_Up: trafo_now += 10; if (trafo_now >= (int) trafo.size()) trafo_now = trafo.size()-1; break;
    case Qt::Key_Down: trafo_now -= 10; if (trafo_now < 0) trafo_now = 0; break;
    case Qt::Key_PageUp: trafo_now += 1000; if (trafo_now >= (int) trafo.size()) trafo_now = trafo.size()-1; break;
    case Qt::Key_PageDown: trafo_now -= 1000; if (trafo_now < 0) trafo_now = 0; break;
    }
    std::cout << trafo_now << std::endl;

    updateGL();
}

void ConturingWidget::updateThreshold(float s) {
    errorThreshold = MIN_ERROR_THRESHOLD + (MAX_ERROR_THRESHOLD-MIN_ERROR_THRESHOLD)*s;
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
    main->statusBar()->showMessage("create DMC Mesh...");
    std::cout << "create DMC Mesh..." << std::endl;
    aligned_vector3f positions;
    vector<uint> indices;
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

void ConturingWidget::scanModel(const QMatrix4x4& P, const QMatrix4x4& V, const Matrix4f& M) {
    QMatrix4x4 PV = P * V;
    if (trafo.empty()) {
        QMatrix4x4 qM = qMat(M);
        programScan.setUniformValue("uMVPMat", PV * qM);
        programScan.setUniformValue("uNMat", qM.normalMatrix());
        model->render(programScan);
    } else {
        for (uint i = 0; i < trafo.size(); ++i) {
            Matrix4f M_trafo = M * trafo[i];
            QMatrix4x4 qM_trafo = qMat(M_trafo);
            programScan.setUniformValue("uMVPMat", PV * qM_trafo);
            programScan.setUniformValue("uNMat", qM_trafo.normalMatrix());
            model->render(programScan);
        }
    }
}

void ConturingWidget::dmc() {

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
/*
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
*/

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
/*
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
        cell_level_count[i] = levelSize*levelSize*levelSize*CELL_EDGES_VERTEX_COUNT;
        for (uint x = 0; x < levelSize; ++x)
            for (uint y = 0; y < levelSize; ++y)
                for (uint z = 0; z < levelSize; ++z)
                    v_level_count[i] += v_count[i][x][y][z];
    }
*/
    positions.clear();
    indices.clear();
    colors.clear();


    createDMCMesh();



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

