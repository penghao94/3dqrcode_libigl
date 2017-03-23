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
	D.setZero(150, 150);
	T.setZero(150, 150);
	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 10; j++) {
			int CENT_X = i * 15 + 8;
			int CENT_Y = j * 15 + 8;
			for (int x = CENT_X - floor((j+1) / 2); x < CENT_X + j+2-floor((j+1) / 2); x++) {
				for (int y = CENT_Y - floor((j + 1) / 2); y < CENT_Y + j + 2 - floor((j + 1) / 2); y++) {
					if(x < CENT_X + j + 1 - floor((j + 1) / 2)&& y < CENT_Y + j + 1 - floor((j + 1) / 2))
						D(x,y) = 1.0;
					T(x, y) = double(i);
				}
			}

		}
	}
	return false;
}
