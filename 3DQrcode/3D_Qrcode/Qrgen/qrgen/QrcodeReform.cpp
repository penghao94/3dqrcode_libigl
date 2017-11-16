#include "QrcodeReform.h"

void qrgen::qrcodeReform(Engine *engine, std::string text, int level, int mask, int border, Eigen::MatrixXd & Modules, Eigen::MatrixXd & Functions)
{

	/*Generate origin QR code*/
	Bits bits;
	Version*version = getMinVersion(text, static_cast<LEVEL>(level), bits);

	/*Code word*/
	std::vector<CodeWord> codeword(bits.getSize()/8,CodeWord());

	/*Priority*/
	VerInfo info=Version::VERSION_INFOS[version->getVersion()];
	int num_block = info.lvlInfos[level].num_of_block;
	int num_check_byte = info.lvlInfos[level].cbytes_pre_block;
    
	int num_block_l = info.lvlInfos[L].num_of_block;
	int num_check_byte_l = info.lvlInfos[L].cbytes_pre_block;

	const int PRIORITY_MAX =floor(static_cast<double>(num_check_byte*num_block - num_check_byte_l*num_block_l) / (2.0*num_block));

	std::vector<int> Block_Priority(num_block, PRIORITY_MAX);
	
	/*Property*/
	std::vector<std::vector<PixelProperty>> Property;
	int m = -1;
	double area=std::numeric_limits<double>::max();

	if (mask == -1) {
		for (int i = 0; i < 8; i++) {

			Property.clear();
			std::vector<std::vector<Pixel>> pixels = encode(bits, version, static_cast<LEVEL>(level), new Mask(i));
			toEigenMatrix(pixels, border, Modules, Functions);
			setArea(engine, Modules, Functions, Property);

			int count = 0;
			double temp_area = 0;

			for (int j = 0; j < Property.size(); j++) {
				count += Property[i].size();
				for (auto k : Property[j]) temp_area += k.area;
			}
			temp_area /= count;

			if (temp_area < area) {
				m = i;
				area = temp_area;
			}

		}
	}
	else {
		m = mask;
	}

	/*iterator*/
	std::vector<std::vector<Pixel>> pixels = encode(bits, version, static_cast<LEVEL>(level), new Mask(m));
	toEigenMatrix(pixels, border, Modules, Functions);
	setArea(engine, Modules, Functions, Property);
	
	for (int j = 0; j < Property.size(); j++) {
		for (auto k : Property[j]) {
			Pixel pixel = pixels[k.y][k.x];
			/* if (x,y) is not data or checksum ,its priority is zero, 
			else if the codeword it belongs to is replaced, setting the greatest priority*/
			if (pixel.getPixelRole() != Pixel::PixelRole::DATA&&pixel.getPixelRole() != Pixel::PixelRole::CHECK)
				k.priority = 0;
			else
				if (codeword[pixel.getOffset() / 8].getStatus())  k.priority = PRIORITY_MAX;
		}
	}

}
/*
TODO:
1.设置迭代停止条件
2.在迭代过程中注意更改codeword的状态
3.设置adapt函数

*/