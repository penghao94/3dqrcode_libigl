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

void qrcode::eList::add(int x, int y,int z)
{
	eNode* node = new eNode();
	node->s = x;
	node->d = y;
	node->id = z;
	node->next = NULL;
	eNode *index=NULL;
	eNode *predex=NULL;
	if (head ==NULL) 
		head = node;
	else
	{
		index = head;
		while (index!=NULL)
		{
			if (index->s!=node->d||index->d!=node->s)
			{
				predex = index;
				index = index->next;
			}
			else
			{
				
				if (index == head)
				{
					head = index->next;
					delete(index);
					delete(node);
					return;
				}
				else
				{
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
	E.resize(0, 3);
	eNode *t = head;
	while (head != NULL) {
		E.conservativeResize(E.rows() + 1, 3);
		E.row(E.rows() - 1) << head->s, head->d,head->id;
		head = head->next;
	}
	head = t;
}
