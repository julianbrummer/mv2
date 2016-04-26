#include <QApplication>
#include <QMenu>
#include <QMenuBar>
#include <QStatusBar>
#include <QFileDialog>
#include <QKeyEvent>


#include "MV.h"

#include <iostream>
#include "OffReader.h"
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
    ogl = new MyGLWidget (this,f);

    // Create a menu
    QMenu *file = new QMenu("&File",this);
    file->addAction ("Load model", this, SLOT(loadModel()), Qt::CTRL+Qt::Key_L);
    //file->addAction ("Load track", this, SLOT(loadTrack()), Qt::CTRL+Qt::Key_T);
    file->addAction ("DMC", ogl, SLOT(dmc()), Qt::CTRL+Qt::Key_D);
    file->addAction ("Quit", qApp, SLOT(quit()), Qt::CTRL+Qt::Key_Q);

    menuBar()->addMenu(file);

    QMenu *view = new QMenu("&View",this);
    view->addAction ("Model", ogl, SLOT(toggleViewModel()), Qt::Key_M);
    view->addAction ("Edge Intersections", ogl, SLOT(toggleViewEdgeIntersections()), Qt::Key_E);
    view->addAction ("Inner Grid", ogl, SLOT(toggleViewInGridPoints()), Qt::Key_I);
    view->addAction ("Outer Grid", ogl, SLOT(toggleViewOutGridPoints()), Qt::Key_O);
    view->addAction ("Vertices", ogl, SLOT(toggleViewDMCVertices()), Qt::Key_V);
    view->addAction ("Cells", ogl, SLOT(toggleViewCells()), Qt::Key_C);
    view->addAction ("Toggle Level/Cell", ogl, SLOT(toggleViewCellLevel()), Qt::Key_L);
    view->addAction ("Higher Level", ogl, SLOT(decreaseSelectedLevel()), Qt::Key_Plus);
    view->addAction ("Lower Level", ogl, SLOT(increaseSelectedLevel()), Qt::Key_Minus);
    view->addAction ("Center Camera", ogl, SLOT(centerCamera()), Qt::Key_Comma);

    menuBar()->addMenu(view);





    // Put the GL widget inside the frame
    QHBoxLayout* layout = new QHBoxLayout();
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

void CGMainWindow::loadTrack() {
// QString filename = QFileDialog::getOpenFileName(this, "Load track ...", QString(), "(*.vda)" );

// if (filename.isEmpty()) return;
// statusBar()->showMessage ("Loading track ...");

// if (filename.endsWith(".vda")) {
//     std::ifstream trackfile(filename.toLatin1());
// }

    getFromVdaFile(ogl->trafo,"tracks/track.vda",ogl->timestep);
}
*/
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
    LoadOffFile("D:/Sonstiges/Uni_Schule/CG_HIWI/MV/mv2/models/bunny.off", positions, normals);
    ogl->model = unique_ptr<Model>(new Model(positions, normals, GL_TRIANGLES, true, 1.8));
    ogl->camera.position = ogl->camera.rotation._transformVector(Z_AXIS) + ogl->camera.center;
    statusBar()->showMessage ("Loading model done.",3000);
    ogl->showModel = true;
    ogl->updateGL();
}

const float MyGLWidget::CAMERA_MOVEMENT_SPEED = 0.0025;
const float MyGLWidget::CAMERA_SCROLL_FACTOR = 1.2;

MyGLWidget::MyGLWidget (CGMainWindow *mainwindow,QWidget* parent ) : QGLWidget (parent) {
    main = mainwindow;
}

bool MyGLWidget::initShaderProgram(const char *vname, const char *fname, QGLShaderProgram& program) {
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

void MyGLWidget::initializeGL() {
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

    res = 64;
    levels = i_log2(res)+1;
    voxelGridRadius = 1.0f;
    cellGridRadius = voxelGridRadius-voxelGridRadius/(res+1);
    origin = Vector3f(-cellGridRadius,-cellGridRadius,-cellGridRadius);
    v_level_count = vector<uint>(levels, 0);
    cell_level_count = vector<uint>(levels, 0);

    //main->loadTrack();
    main->loadModel();
    showModel = false;
    showEdgeIntesections = false;
    showInGridPoints = false;
    showOutGridPoints = false;
    showDMCVertices = false;
    showCells = false;
    showSingleCell = false;
    selectedLevel = 0;
}

void MyGLWidget::bindDebugMesh(Model* model, const Matrix4f& V, bool useVertexColor, QVector4D color) {
    programColor.bind();
    Matrix4f VM = V * model->getModelMatrix();
    Matrix4f PVM = camera.projection * VM;
    programColor.setUniformValue("uMVPMat",qMat(PVM));
    programColor.setUniformValue("useVertexColor", useVertexColor);
    programColor.setUniformValue("uColor", color);
}

void MyGLWidget::renderDebugMesh(Model* model, const Matrix4f &V, bool useVertexColor, QVector4D color) {
    bindDebugMesh(model, V, useVertexColor, color);
    model->render(programColor);
}

void MyGLWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    Matrix4f V = camera.getViewMatrix();
    if (model && showModel) {
        program.bind();
        Matrix4f VM = V * model->getModelMatrix();
        Matrix4f PVM = camera.projection * VM;
        program.setUniformValue("uMVPMat",qMat(PVM));
        program.setUniformValue("uNMat", qMat(VM).normalMatrix());
        program.setUniformValue("uColor", QVector4D(0.75,0.75,0.75,1.0));
        model->render(program);
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
        bindDebugMesh(dmcVertices.get(), V, false);
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

void MyGLWidget::resizeGL(int width, int height) {
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

void MyGLWidget::wheelEvent(QWheelEvent* event) {
    int delta = event->delta();
    zoom *= (delta < 0)? CAMERA_SCROLL_FACTOR : 1/CAMERA_SCROLL_FACTOR;
    camera.position = camera.rotation._transformVector(zoom*Z_AXIS) + camera.center;
    updateGL();
}

void MyGLWidget::trackballCoord(int x, int y, Vector3f& v) {
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

Quaternionf MyGLWidget::trackball(const Vector3f& u, const Vector3f& v) {
    Vector3f uxv = u.cross(v);
    float uTv = u.transpose() * v;
    Quaternionf ret(1+uTv,uxv.x(), uxv.y(), uxv.z());
    ret.normalize();
    return ret;
}

void MyGLWidget::keyPressEvent(QKeyEvent* event) {
    if (showSingleCell) {
        switch (event->key()) {
        case Qt::Key_8: if (selectedCell.z > 0) selectedCell = selectedCell.shiftZ(-1); break;
        case Qt::Key_2: if (selectedCell.z < pow2(selectedLevel)-1) selectedCell = selectedCell.shiftZ(1); break;
        case Qt::Key_4: if (selectedCell.x > 0) selectedCell = selectedCell.shiftX(-1); break;
        case Qt::Key_6: if (selectedCell.x < pow2(selectedLevel)-1) selectedCell = selectedCell.shiftX(1); break;
        case Qt::Key_7: if (selectedCell.y > 0) selectedCell = selectedCell.shiftY(-1); break;
        case Qt::Key_9: if (selectedCell.y < pow2(selectedLevel)-1) selectedCell = selectedCell.shiftY(1); break;
        }
    }
    updateGL();
}

void MyGLWidget::mousePressEvent(QMouseEvent *event) {
    button = event->button();
    oldX = event->x();
    oldY = event->y();

    updateGL();
}

void MyGLWidget::mouseReleaseEvent(QMouseEvent*) {}

void MyGLWidget::mouseMoveEvent(QMouseEvent* event) {
    int x = event->x();
    int y = event->y();

    if (button == Qt::MiddleButton) {
        Vector3f u,v;

        trackballCoord(oldX,oldY,u);
        trackballCoord(x,y,v);

        Quaternionf q = trackball(u,v);
        model->rotate(q);
    }

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

void MyGLWidget::dmc() {

    std::cout << "scan model" << std::endl;
    CompressedHermiteScanner scan(FRONT_AND_BACK_XYZ, res, voxelGridRadius, programScan);
    QMatrix4x4 V;
    Matrix4f modelMat = model->getModelMatrix();
    QMatrix4x4 M = qMat(modelMat);

    scan.begin(FRONT_AND_BACK_FACES_X, V);
    QMatrix4x4 VM = V * M;
    programScan.setUniformValue("uMVPMat", scan.projection * VM);
    programScan.setUniformValue("uNMat", M.normalMatrix());
    model->render(programScan);
    scan.end();

    scan.begin(FRONT_AND_BACK_FACES_Y, V);
    VM = V * M;
    programScan.setUniformValue("uMVPMat", scan.projection * VM);
    programScan.setUniformValue("uNMat", M.normalMatrix());
    model->render(programScan);
    scan.end();

    scan.begin(FRONT_AND_BACK_FACES_Z, V);
    VM = V * M;
    programScan.setUniformValue("uMVPMat", scan.projection * VM);
    programScan.setUniformValue("uNMat", M.normalMatrix());
    model->render(programScan);
    scan.end();

    std::cout << "create sampler" << std::endl;
    CompressedHermiteSampler sampler(*(scan.data));

    std::cout << "generate edge intersection model" << std::endl;
    aligned_vector3f positions, colors;
    sampler.edgeIntersections(voxelGridRadius, positions, colors);
    edgeIntersections = unique_ptr<Model>(new Model(positions, colors, false, GL_POINTS));

    std::cout << "generate in grid model" << std::endl;
    positions.clear();
    sampler.inside(voxelGridRadius, positions);
    inGridPoints = unique_ptr<Model>(new Model(positions, GL_POINTS, false));

    std::cout << "generate out grid model" << std::endl;
    positions.clear();
    sampler.outside(voxelGridRadius, positions);
    outGridPoints = unique_ptr<Model>(new Model(positions, GL_POINTS, false));


    DualMarchingCubes DMC(sampler);
    std::cout << "create octree" << std::endl;
    DMC.createOctree();
    std::cout << "create vertex tree" << std::endl;
    DMC.createVertexTree();

    std::cout << "generate vertex model" << std::endl;
    positions.clear();
    DMC.vertices(voxelGridRadius, positions, v_offset, v_count);
    dmcVertices = unique_ptr<Model>(new Model(positions, GL_POINTS, false));

    std::cout << "generate cell model" << std::endl;
    positions.clear();
    DMC.cells(voxelGridRadius, positions, cell_offset);
    cells = unique_ptr<Model>(new Model(positions, GL_LINES, false));

    for (int i = 0; i < levels; ++i) {
        uint levelSize = pow2(i);
        cell_level_count[i] = levelSize*levelSize*levelSize*CELL_EDGES_VERTEX_COUNT;
        for (uint x = 0; x < levelSize; ++x)
            for (uint y = 0; y < levelSize; ++y)
                for (uint z = 0; z < levelSize; ++z)
                    v_level_count[i] += v_count[i][x][y][z];
    }

    resizeGL(w,h);
    std::cout << "done." << std::endl;
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

