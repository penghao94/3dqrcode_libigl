/*
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Lorrain <lorrain0407@gmail.com>
 *
 * saveResult.h  2017/03/09 18:10
 * TODO: This function is used to save the result of 3D printed QR Code with a format of png
 *
*/
#pragma once
#ifndef SAVERESULT_H
#define SAVERESULT_H
#include <iostream>
#include <igl\viewer\Viewer.h>
#include<Eigen\Core>
#include <igl\file_dialog_save.h>
#include<igl\png\writePNG.h>

namespace qrcode {

	//************************************
	// Method:    saveResult
	// FullName:  qrcode::saveResult
	// Access:    public 
	// Returns:   bool
	// Qualifier:
	// Parameter: igl::viewer::Viewer & viewer
	//************************************
	bool saveResult(igl::viewer::Viewer &viewer);

	//************************************
	// Method:    saveResult
	// FullName:  qrcode::saveResult
	// Access:    public 
	// Returns:   bool
	// Qualifier:
	// Parameter: igl::viewer::Viewer & viewer
	// Parameter: const std::string file
	//************************************
	bool saveResult(igl::viewer::Viewer &viewer, const std::string file);

}

#endif // !SAVERESULT_H
