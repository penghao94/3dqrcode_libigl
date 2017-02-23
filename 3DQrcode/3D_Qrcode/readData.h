#pragma once
#ifndef READDATA_H_
#define READDATA_H_
#include <iostream>
#include <Eigen/core>
#include <stb_image.h>
namespace qrcode {
	bool readData(Eigen::MatrixXi &D);
	bool readData(const std::string file, Eigen::MatrixXi &D);
}
#endif // !READDATA_H_

