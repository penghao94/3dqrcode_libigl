// This file is part of 3DQrcode
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.


#ifndef QRCODEMODIFIER_H
#define QRCODEMODIFIER_H
#include <iostream>
#include <algorithm>
#include <climits>
#include <cmath>
#include <cstddef>
#include <sstream>
#include<QrCode.hpp>
namespace qrcode {
	void qrcodeModefier(qrcodegen::QrCode &qr);
	void drawFinderPatterns(qrcodegen::QrCode &qr,int x,int y);
	void drawAlignmentPattern(qrcodegen::QrCode &qr,int x,int y);
}
#endif // !QRCODEMODEFIER_H
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 