#ifndef OFF_H
#define OFF_H
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <QMatrix4x4>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <clocale>

using namespace Eigen;
using namespace std;
typedef std::vector<Vector3f, aligned_allocator<Vector3f>> aligned_vector3f;

void LoadOffFile(const char* as_FileName, aligned_vector3f& vertices,
                aligned_vector3f& normals)
{
    aligned_vector3f ak_Vertices;
    std::vector<int> ak_Indices;

	std::cout << "LoadOffFile(\"" << as_FileName << "\");" << std::endl;

	std::vector<float> lk_Faces;
	int li_TriangleCounter = 0;
	int li_VerticeLength = 0;
	int li_FaceCounter = 0;

	std::ifstream lk_InStream(as_FileName);
	if(!lk_InStream)
	{
        std::cout << "Cannot open off-file" << std::endl;
		return;
	}
	std::string ls_FileHeader;
	std::getline(lk_InStream, ls_FileHeader);
	if (ls_FileHeader.length() > 0 && ls_FileHeader[ls_FileHeader.length()-1] == 0xd) 
				ls_FileHeader.erase (ls_FileHeader.length()-1,1);

	if(ls_FileHeader.compare ("OFF") == 0)
	{
        std::cout << "Read OFF-File..." << std::endl;
	}
	else 
	{
        std::cout << "Invalid OFF-File" << std::endl;
		return;
	}

	lk_InStream >> li_VerticeLength;
    std::cout << "Vertices: " << li_VerticeLength << std::endl;
	lk_InStream >> li_FaceCounter;
    std::cout << "Faces: " << li_FaceCounter << std::endl;
	int li_Edges;
	lk_InStream >> li_Edges;
    std::cout << "Edges: " << li_Edges << std::endl;

	for(int i = 0; i < li_VerticeLength; i++) 
	{
		float x, y, z;
		lk_InStream >> x;
		lk_InStream >> y;
		lk_InStream >> z;
        ak_Vertices.push_back(Vector3f(x, y, z));
	}

	int li_Temp1, li_Temp2, li_Temp3;

	for (int i = 0; i < li_FaceCounter; i++) 
	{
		int li_Kanten;
		lk_InStream >> li_Kanten;
		for(int j = 0; j < li_Kanten - 2; j++)
		{
			if (j == 0)
			{
				lk_InStream >> li_Temp1;
				lk_InStream >> li_Temp2;
				lk_InStream >> li_Temp3;
			}
			if (j > 0)
			{
				li_Temp2 = li_Temp3;
				lk_InStream >> li_Temp3;
			}
			li_TriangleCounter++;
			lk_Faces.push_back(float(li_Temp1));
			lk_Faces.push_back(float(li_Temp2));
			lk_Faces.push_back(float(li_Temp3));
		}
	}

	for (int i = 0; i < li_TriangleCounter * 3; i++)
	{
		ak_Indices.push_back(int(lk_Faces[i]));
	}

    for (int i = 0; i < li_TriangleCounter * 3; i+=3) {
        Vector3f v0 = ak_Vertices[ak_Indices[i]];
        Vector3f v1 = ak_Vertices[ak_Indices[i+1]];
        Vector3f v2 = ak_Vertices[ak_Indices[i+2]];
        Vector3f n = (v1 - v0).cross(v2 - v0);
        n.normalize();
        vertices.push_back(v0);
        vertices.push_back(v1);
        vertices.push_back(v2);
        normals.push_back(n);
        normals.push_back(n);
        normals.push_back(n);
    }
}

void LoadVdaFile(vector<Matrix4f>& trafo, const char* filename, float scale, qreal& timestep) {
    std::setlocale(LC_NUMERIC,"C");

    std::cout << "LoadVdaFile(\"" << filename << "\");" << std::endl;
    std::ifstream vdafile( filename );
    if (!vdafile) {
        std::cerr << "Cannot open vda-file" << std::endl;
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

    Matrix4f M;
    M.setIdentity();
    Matrix4f S = Scaling(Vector4f(scale, scale, scale, 1));

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

            trafo.push_back(M * S);
            n++;
    }

    vdafile.close();

    std::cout << "Loaded " << filename << " with " << trafo.size() << " transformations" << std::endl;
}

void writeToOffFile(aligned_vector3f& positions, vector<uint>& indices, const char *filename) {
    std::ofstream offFile(filename);

    offFile << "OFF" << std::endl;
    offFile << positions.size() << " " << indices.size()/3 << " " << positions.size()+indices.size()/3-2 << std::endl;

    for(uint i=0; i < positions.size(); i++) {
        const Vector3f& v = positions[i];
        offFile << v.x() << " " << v.y() << " " << v.z() << std::endl;
    }

    for(uint i=0; i < indices.size()/3; i++)
        offFile << 3 << " " << indices[3*i+0] << " " << indices[3*i+1] << " " << indices[3*i+2] << std::endl;

    offFile.close();
}

#endif


