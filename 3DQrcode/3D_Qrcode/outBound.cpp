#include "outBound.h"
#include <vector>
bool qrcode::outBound(Eigen::MatrixXi & E, int start,int scale, Eigen::MatrixXi & B)
{
	using namespace std;
	vector<int> Bound, Node, Step, temp;
	bool isBound, isNode,isEnd = false;
	Node.push_back(-1);
	while (!isEnd) {
		Step.clear();
		if (Bound.empty()) {
			for (int i = 0; i < E.rows(); i++) {
				if (E(i, 0) == E(start, 1)) {
					Step.push_back(i);
				}
			}
			if (Step.size() == 1) {
				Bound.push_back(Step.back());
				//cout << int(E(Bound.back(), 1) / scale) << "	" << int(E(Bound.back(), 1) % scale) << endl;
			}
			else if (Step.size() == 2) {
				Node.push_back(Step[0]);
				temp.push_back(Step[1]);
				//cout << "STEP:" << int(E(Step[0], 1) / scale) << "	" << int(E(Step[0], 1) % scale) << endl;
				//cout << "STEP:" << int(E(Step[1], 1) / scale) << "	" << int(E(Step[1], 1) % scale) << endl;
				Bound.push_back(Node.back());
				//cout <<"NODE:"<< int(E(Bound.back(), 1) / scale) << "	" << int(E(Bound.back(), 1) % scale) << endl;
			}
			
		}
		else if (E(Bound.back(), 0) != E(start, 0) ||E(Bound.back(), 1) != E(start, 1)) {
			
			for (int i = 0; i < E.rows(); i++) {
				if (E(i, 0) == E(Bound.back(), 1)) {
					Step.push_back(i);
					//cout << "STEP:" << int(E(Step.back(), 1) / scale) << "	" << int(E(Step.back(), 1) % scale) << endl;
				}
			}

			if (Step.size() == 1) {
				isNode = true;
				for (int i = 0; i < Node.size(); i++) {
					if (Step.back() == Node[i]) {
						while (Bound.back() != Node[i]) {
							Bound.pop_back();
							//cout << int(E(Bound.back(), 1) / scale) << "	" << int(E(Bound.back(), 1) % scale) << endl;
						}
						Bound.pop_back();
						while (Node.back() != Node[i])
							Node.pop_back();
						Node.pop_back();
						Bound.push_back(Node.back());
						//cout << int(E(Bound.back(), 1) / scale) << "	" << int(E(Bound.back(), 1) % scale) << endl;
						isNode = false;
					}
				}
				if (isNode) {
					Bound.push_back(Step.back());
					//cout << int(E(Bound.back(), 1) / scale) << "	" << int(E(Bound.back(), 1) % scale) << endl;
				}
				//Step.pop_back();

			}
			else if (Step.size() ==2) {

				isNode = true;
				for (int j = 0; j < Step.size(); j++) {
					
					for (int i = 0; i < Node.size(); i++) {
						if (Step[j] == Node[i]) {
							while (Bound.back() != Node[i]) {
								//cout << "TO:" << int(E(Node[i], 1) / scale) << "	" << int(E(Node[i], 1) % scale) << endl;
								//cout <<"BACK:"<< int(E(Bound.back(), 1) / scale) << "	" << int(E(Bound.back(), 1) % scale) << endl;
								//cout << Bound.size() << endl;
								Bound.pop_back();
								
							}
							Bound.pop_back();
							//cout << "TRAN:" << int(E(Bound.back(), 1) / scale) << "	" << int(E(Bound.back(), 1) % scale) << endl;
							while (Node.back() != Node[i]) {
								Node.pop_back();
								temp.pop_back();
							}
								
							Node.back()=temp.back();
							Bound.push_back(Node.back());
							//cout <<"NODE:"<< int(E(Bound.back(), 1) / scale) << "	" << int(E(Bound.back(), 1) % scale) << endl;
							isNode = false;
						}
					}
					if (!isNode)
						break;
				}
				if (isNode) {
					
					Node.push_back(Step[0]);
					temp.push_back(Step[1]);
					Bound.push_back(Node.back());
					//cout << "NODE:" << int(E(Bound.back(), 1) / scale) << "	" << int(E(Bound.back(), 1) % scale) << endl;
				}
				//Step.clear();
			}
		}
		else if (E(Bound.back(), 0) == E(start, 0) && E(Bound.back(), 1) == E(start, 1)) {
			isEnd = true;
		}

	}
	B.resize(Bound.size(), 2);
	for (int i = 0; i < B.rows(); i++) {
		B.row(B.rows() - 1 - i) << E.row(Bound.back());
		Bound.pop_back();
	}
	Node.clear();
	temp.clear();
	Step.clear();
	return true;
}
