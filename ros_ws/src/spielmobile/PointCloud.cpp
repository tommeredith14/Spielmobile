 #include "PointCloud.h"
#include <iostream>

bool CKDTree::Insert(const Point& p) {
	KDNode* curNode = m_root;
	int dimension = 0;
	if (m_root == nullptr) {
		m_root = new KDNode(p);
		m_size++;
		return true;
	}

	while (curNode != nullptr) {
		//check what side to go on
		if (p[dimension] > (*curNode)[dimension]) {
			//go to right
			if (curNode->right != nullptr) {
				curNode = curNode->right;
				dimension = (dimension+1)%2;
				continue;
			} else {
				//insert
				curNode->right = new KDNode(p);
				m_size++;
				return true;
			}
		} else {
			//go to left
			if (curNode->left != nullptr) {
				curNode = curNode->left;
				dimension = (dimension+1)%2;
				continue;
			} else {
				//insert
				curNode->left = new KDNode(p);
				m_size++;
				return true;
			}
		}

	}
	return false;

}

void KDNode::Print() {
	if (left)
		left->Print();
	std::cout << coords[0] << "," << coords[1] << "\n";

	if (right)
		right->Print();

}

void CKDTree::PrintTree() {
	if (m_size == 0)
		std::cout << "Empty\n";
	KDNode* curNode = m_root;
	curNode->Print();
}
