/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * writePNG.h  2017/03/13 17:06
 * TODO:
 *
*/
#pragma once
#ifndef PRINTPNG_H
#define PRINTPNG_H
#include <iostream>
#include <igl/viewer/Viewer.h>
namespace qrcode {
	bool printPNG(igl::viewer::Viewer &viewer);
	bool printPNG(igl::viewer::Viewer &viewer, const std::string file);
}
#endif // !PRINTPNG_H


