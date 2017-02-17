#ifndef SAVEMESH_H
#define SAVEMESH_H
#include <igl/viewer/Viewer.h>
#include <igl/viewer/ViewerData.h>
#include <igl/writeSTL.h>
#include <iostream>
using namespace std;
//#include "igl/viewer/ViewerPlugin.h"
namespace qrcode
{
	bool SaveMesh(const char* mesh_file_name, igl::viewer::Viewer viewer, igl::viewer::ViewerData data);
}

#endif // !SAVEMESH_H

