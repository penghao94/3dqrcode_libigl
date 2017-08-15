#include "qrcodeModifier.h"

void qrcode::qrcodeModefier(qrcodegen::QrCode & qr)
{
	// Draw 3 finder patterns (all corners except bottom right; overwrites some timing modules)
	drawFinderPatterns(qr,3, 3);
	drawFinderPatterns(qr,qr.size - 4, 3);
	drawFinderPatterns(qr,3, qr.size - 4);

	// Draw the numerous alignment patterns
	const std::vector<int> alignPatPos(qr.getAlignmentPatternPositions(qr.version));
	int numAlign = alignPatPos.size();
	for (int i = 0; i < numAlign; i++) {
		for (int j = 0; j < numAlign; j++) {
			if ((i == 0 && j == 0) || (i == 0 && j == numAlign - 1) || (i == numAlign - 1 && j == 0))
				continue;  // Skip the three finder corners
			else
				drawAlignmentPattern(qr,alignPatPos.at(i), alignPatPos.at(j));
		}
	}
}

void qrcode::drawFinderPatterns(qrcodegen::QrCode & qr, int x, int y)
{
	for (int i = -4; i <= 4; i++) {
		for (int j = -4; j <= 4; j++) {
			int max = std::max(std::abs(i), std::abs(j));
			int min = std::min(std::abs(i), std::abs(j));
			int xx = x + j, yy = y + i;
			if (0 <= xx && xx < qr.size && 0 <= yy && yy < qr.size)
				qr.setFunctionModule(xx, yy, min == 0 && (max == 0 || max % 2 == 1));
		}
	}
}

void qrcode::drawAlignmentPattern(qrcodegen::QrCode & qr, int x, int y)
{
	for (int i = -2; i <= 2; i++) {
		for (int j = -2; j <= 2; j++) {
			int max = std::max(std::abs(i), std::abs(j));
			int min = std::min(std::abs(i), std::abs(j));
			qr.setFunctionModule(x + j, y + i, min == 0 && (max == 0 || max == 2));
		}
	}
}
