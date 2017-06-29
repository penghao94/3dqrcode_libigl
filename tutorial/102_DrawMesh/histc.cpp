#include "histc.h"
double qrcode::histc(Eigen::VectorXd & C)
{
	double r = (double)rand() / (double)RAND_MAX;
	if (r<0 || r>C(C.size() - 1)) {
		histc(C);
	}
	else
	{
		//find r in C
		int l = 0;
		int h = C.size() - 1;
		int k = l;
		while ((h - l) > 1)
		{
			assert(r >= C(l));
			assert(r <= C(h));
			k = (h + l) / 2;
			if (r < C(k)){
				h = k;
			}else{
				l = k;
			}
		}
		if (r == C(h))
		{
			k = h;
		}
		else
		{
			k = l;
		}
		return k;
	}
	
}
