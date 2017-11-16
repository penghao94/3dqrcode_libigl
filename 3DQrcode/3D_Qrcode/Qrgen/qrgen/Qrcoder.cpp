#include "Qrcoder.h"
typedef std::vector<std::vector<qrgen::Pixel>> MatrixP;

qrgen::Version * qrgen::getMinVersion(std::string&text, LEVEL level,Bits &bits)
{
	
	for (int i = Version::MIN_VERSION; i < Version::MAX_VERSION; i++) {
		Bits b;
		//Alpha(text).encode(bits, new Version(i));
		Raw(text).encode(b,new Version(i));
		VerInfo v = Version::VERSION_INFOS[i];
		int block_num =v.lvlInfos[level].num_of_block;//number of blocks
		int check_bytes_num = v.lvlInfos[level].cbytes_pre_block;//number of check bytes per block
		int data_bytes = v.bytes - check_bytes_num*block_num;//number of data bytes

		if (b.getBits().size() > data_bytes ) continue;

		int size = b.getSize();
		if (size < data_bytes * 8)
			b.pad(data_bytes * 8 - size);
		assert(!(b.getSize() != data_bytes * 8) && "qrcode has too mush data");

		b.addCheckBytes(new Version(i), level);
		bits = b;
		return new Version(i);
	}
}
MatrixP qrgen::encode(Bits& bits,Version *version, LEVEL level, Mask * mask)
{
	
	Plan *plan = Plan::newPlan(version, level, mask);
	MatrixP pixels = plan->getPixels();

	for (int y = 0; y < pixels.size(); y++) {
		for (int x = 0; x < pixels.size(); x++) {
			Pixel::PixelRole role = pixels[y][x].getPixelRole();
			if (role == Pixel::PixelRole::DATA || role == Pixel::PixelRole::CHECK) {
				int index = pixels[y][x].getOffset();
				int value = static_cast<int>((bits.getBits()[index / 8] >> (7 - index & 0x7)) & 0x1);
				pixels[y][x].xorPixel(value);
			}
		}
	}

	return pixels;
}

std::string qrgen::toSvgString(MatrixP &pixels, int border)
{
	if (border < 0)
		throw "Border must be non-negative";
	std::ostringstream sb;
	sb << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
	sb << "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n";
	sb << "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" viewBox=\"0 0 ";
	sb << (pixels.size() + border * 2) << " " << (pixels.size() + border * 2) << "\">\n";
	sb << "\t<rect width=\"100%\" height=\"100%\" fill=\"#FFFFFF\" stroke-width=\"0\"/>\n";
	sb << "\t<path d=\"";
	bool head = true;
	for (int y = 0; y < pixels.size(); y++) {
		for (int x =0; x < pixels.size(); x++) {
			if (pixels[y][x].getPixel() == 1) {
				if (head)
					head = false;
				else
					sb << " ";
				sb << "M" << (x + border) << "," << (y + border) << "h1v1h-1z";
			}
		}
	}
	sb << "\" fill=\"#000000\" stroke-width=\"0\"/>\n";
	sb << "</svg>\n";
	std::ofstream out("qr.svg");
	out << sb.str();
	out.close();
	return sb.str();
}

void qrgen::toEigenMatrix(MatrixP & pixels,int border, Eigen::MatrixXd & Modules, Eigen::MatrixXd & Functions)
{
	Modules.setZero(2 * border + pixels.size() + 1, 2 * border + pixels.size() + 1);
	Functions.setZero(2 * border + pixels.size() + 1, 2 * border + pixels.size() + 1);

	for (int y = 0; y < pixels.size(); y++) {
		for (int x = 0; x < pixels.size(); x++) {
			Modules(y + border, x + border) = pixels[y][x].getPixel();
			qrgen::Pixel::PixelRole role = pixels[y][x].getPixelRole();
			if (role != qrgen::Pixel::DATA&&role != qrgen::Pixel::CHECK&&role != qrgen::Pixel::EXTRA&&pixels[y][x].getPixel() == 1)
				Functions(y + border, x + border) = 1.0;
		}
	}
}

void qrgen::setArea(Engine * engine, Eigen::MatrixXd & Modules, Eigen::MatrixXd & Functions, std::vector<std::vector<qrgen::PixelProperty>>& Property)
{
	int size = Modules.rows();

	Eigen::MatrixXd BW_label;
	qrcode::bwlabel(engine, Modules, 4, BW_label);

	std::vector<Eigen::MatrixXi> BW_vertex;
	qrcode::bwindex(BW_label, BW_vertex);
	Property.resize(BW_vertex.size());

	for (int y = 0; y < size - 1; y++) {
		for (int x = 0; x < size - 1; x++) {
			if (Modules(y, x) > 0 && Functions(y, x) == 0.0) {
				std::vector<Eigen::Vector2d> Bound;
				qrcode::lightRegion(Eigen::RowVector2d(y + 0.5, x + 0.5), BW_vertex[BW_label(y, x) - 1], 1, size, Bound);

				boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > poly;
				for (auto i : Bound) poly.outer().emplace_back(i(0), i(1));
				//Property[BW_label(y, x) - 1].emplace_back(x, y, 0, boost::geometry::area(poly), 0);
			}
		}
	}
}

void qrgen::writePng(std::string & file, int scale, Eigen::MatrixXd & Modules, Eigen::MatrixXd & Functions)
{
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;
	R.resize((Modules.rows() - 1)*scale, (Modules.cols() - 1)*scale);
	G.resize((Modules.rows() - 1)*scale, (Modules.cols() - 1)*scale);
	B.resize((Modules.rows() - 1)*scale, (Modules.cols() - 1)*scale);
	A.resize((Modules.rows() - 1)*scale, (Modules.cols() - 1)*scale);

	for (int i = 0; i < Modules.rows() - 1; i++) {
		for (int j = 0; j < Modules.cols() - 1; j++) {
			for (int x = 0; x < scale; x++) {
				for (int y = 0; y < scale; y++) {

					if (Modules(Modules.cols() - 2 - j, i) == 0) {
						R(i*scale + x, j*scale + y) = 255;
						G(i*scale + x, j*scale + y) = 255;
						B(i*scale + x, j*scale + y) = 255;
					}
					else {
						R(i*scale + x, j*scale + y) = 0;
						G(i*scale + x, j*scale + y) = 0;
						B(i*scale + x, j*scale + y) = 0;
					}
					A(i*scale + x, j*scale + y) = 255;
				}
			}
		}
	}
	igl::png::writePNG(R, G, B, A, file);
}
