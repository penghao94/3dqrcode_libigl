/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * test.cpp  2017/03/21 16:22
 * TODO:
 *
*/
#include "test.h"


bool qrcode::test(Eigen::MatrixXd & D, Eigen::MatrixXd & T)
{
	D.setZero(201,201);
	T.setZero(201,201);
	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 8; j++) {
			int CENT_X = i * 25 + 12;
			int CENT_Y = j * 25 + 12;
			for (int x = CENT_X - j-1; x < CENT_X + j+3; x++) {
				for (int y = CENT_Y -j-1; y < CENT_Y + j +3; y++) {
					if(x < CENT_X + j + 2&& y < CENT_Y + j + 2)
						D(x, y) = 1.0;
					T(x, y) = double(i+1);
				}
			}

		}
	}
	return false;
}
