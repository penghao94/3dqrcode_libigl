#include "halfedge.h"
#include "stdio.h"
#include <iostream>
using namespace std;
qrcode::eList::eList()
{
	head = NULL;
}

qrcode::eList::~eList()
{
	while (head != NULL) {
		eNode *temp = head;
		head = head->next;
		delete(temp);
	}
	delete(head);
}

void qrcode::eList::add(int x, int y)
{
	eNode* node = new eNode();
	node->s = x;
	node->d = y;
	node->next = NULL;
	eNode *index=NULL;
	eNode *predex=NULL;
	if (head ==NULL) {
		head = node;
		cout << "head:" << head << endl;
	}
	else
	{
		index = head;
		cout << "head:" << index << endl;
		while (index!=NULL)
		{
			if (index->s!=node->d||index->d!=node->s)
			{
				predex = index;
				index = index->next;
				cout << "predex:" << predex << endl;
				cout << "index:" << index << endl;
			}
			else
			{
				
				if (index == head)
				{
					head = index->next;
					delete(index);
					delete(node);
					//cout << "delete head" << endl;
					return;
				}
				else
				{
					//cout << "delete other" << endl;
					predex->next = index->next;
					delete(index);
					delete(node);
					return;
				}
			}
		}
		index = node;
		predex->next = index;
	}
}



void qrcode::eList::matrix(Eigen::MatrixXi &E)
{
	E.resize(0, 2);
	eNode *t = head;
	while (head != NULL) {
		E.conservativeResize(E.rows() + 1, 2);
		E.row(E.rows() - 1) << head->s, head->d;
		head = head->next;
	}
	head = t;
}
