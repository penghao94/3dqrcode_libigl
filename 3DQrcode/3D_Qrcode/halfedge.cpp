#include "halfedge.h"
#include "stdio.h"
#include <iostream>

qrcode::eList::eList()
{
	node = NULL;
	head = NULL;
}

qrcode::eList::~eList()
{
	while (head != NULL) {
		eNode *temp=head;
		head = head->next;
		delete(temp);
	}
	delete(head);
}

void qrcode::eList::add(int x, int y)
{
	eNode *temp;
	eNode *t;
	node = new eNode();
	node->s = x;
	node->d = y;
	node->next = NULL;
	if (head == NULL) {
		head = node;
	}
	else {
		temp = head;
		t = temp;
		//std::cout << temp->s << "   " << temp->d << std::endl;
		while (temp != NULL) {
			//std::cout << node->s << "   " << node->d << std::endl;
			if (node->s != temp->s && node->d != temp->d) {
				int m = y;
				int n = x;
				if (node->s != m && node->d != n) {
					t = temp;
					temp = temp->next;
				}	
				else {
					if (temp == head) {
						head = temp->next;
						delete(temp);
					}
					else {
						t->next = temp->next;
						delete(temp);
						temp = NULL;
					}
					return;
				}
			}
			else {
				if (temp == head) {
					head = temp->next;
					delete(temp);
					temp = NULL;
				}
				else {
					t->next = temp->next;
					delete(temp);
					return;
				}//end of third else
			}//end of second else
		}//end of while
		temp = node;
		t->next = temp;
	}//end of first else
}//end of void
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

