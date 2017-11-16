// This project is a C++ version of qart4j,for more information see https://github.com/dieforfree/qart4j
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.

#ifndef QRCODEREFORM_H_
#define QRCODEREFORM_H_
#include <string>
#include<limits.h>

#include <Eigen/core>
#include "CodeWord.h"
#include "Qrcoder.h"
namespace qrgen {

	//************************************
	// Method:    qrgen::qrcodeReform
	// Returns:   void
	//
	// @Param std::string text
	// @Param int level
	// @Param int mask
	// @Param int border
	// @Param Eigen::MatrixXd & Modules
	// @Param Eigen::MatrixXd & Functions
	//************************************
	void qrcodeReform(Engine *engine,std::string text, int level, int mask, int border, Eigen::MatrixXd &Modules, Eigen::MatrixXd &Functions);
}
#endif // !QRCODEREFORM_H_

