#include "PointCloud.h"


int main(int argc, char **argv)  {

	CKDTree tree;

	tree.Insert(Point(5,5));
	tree.Insert(Point(4,5));
	tree.Insert(Point(6,5));
	tree.Insert(Point(4.5,8));
	tree.Insert(Point(4.5,1));

	tree.PrintTree();
	return 0;
}
