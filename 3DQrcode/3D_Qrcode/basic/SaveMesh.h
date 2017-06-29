/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * SaveMesh.h  2017/02/26 16:34
 * TODO: This function is used to save model 
 *
*/
#ifndef SAVEMESH_H_
#define SAVEMESH_H_
#include <igl/viewer/Viewer.h>
#include <igl/viewer/ViewerData.h>
#include <igl/writeSTL.h>
#include <iostream>
using namespace std;
namespace qrcode
{
	//************************************
	// Method:    qrcode::saveMesh
	// Returns:   bool
	//
	// @Param igl::viewer::Viewer & viewer
	// @Param igl::viewer::ViewerData & data
	//************************************
	bool saveMesh( igl::viewer::Viewer &viewer,igl::viewer::ViewerData& data);
	//************************************
	// Method:    qrcode::saveMesh
	// Returns:   bool
	//
	// @Param const char * mesh_file_name
	// @Param igl::viewer::Viewer & viewer
	// @Param igl::viewer::ViewerData & data
	//************************************
	bool saveMesh(const char* mesh_file_name, igl::viewer::Viewer &viewer, igl::viewer::ViewerData& data);
}

#endif // !SAVEMESH_H

