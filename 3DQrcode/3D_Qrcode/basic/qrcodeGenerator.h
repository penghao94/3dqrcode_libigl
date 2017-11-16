/*!
* This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
*
* Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
*
* qrcodeGenerator.h  2017/03/07 10:29
* TODO:
*
*/
#pragma once
#ifndef QRCODEGENERATOR_H_
#define QRCODEGENERATOR_H_
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include<fstream>
#include <string>
#include <vector>
#include "Eigen/core"
#include "../qrgen/Qrcoder.h"
#include <igl/matlab/matlabinterface.h>
namespace qrcode {

	//************************************
	// Method:    qrcode::qrCodeGenerator
	// Returns:   bool
	//
	// @Param std::string text	User-supplied text
	// @Param const qrcodegen::QrCode::Ecc & errColLvl	Error correction level
	// @Param int mask the kind of mask from 0~7
	// @Param int border the white border per module
	// @Param Eigen::MatrixXd & Q	the QR Code symbol
	//************************************
	
	bool qrCodeGenerator(std::string text,int errCol,int mask,int border,Eigen::MatrixXd &Q);
	bool qrCodeGenerator(std::string text,int errColLvl, int mask, int border, Eigen::MatrixXd &Q, Eigen::MatrixXd &F);
	bool writePNG(std::string file, Eigen::MatrixXd &Q, int scale);
	void writePNG(Engine *engine,std::string file, int scale, Eigen::MatrixXd &D, Eigen::MatrixXd &F);
}
#endif // !QRCODEGENERATOR_H_
