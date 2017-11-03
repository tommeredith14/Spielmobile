#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <vector>

class Point {
public:
	float coords[2];
	Point() 
	{
		coords[0] = 0.0f;
		coords[1] = 0.0f;
	}
	Point(float x, float y) 
	{
		coords[0] = x;
		coords[1] = y;
	}
	float x() {return coords[0];}
	float y() {return coords[1];}

	float operator[](int index) const {return coords[index];}
};

class KDNode : public Point{
public:
	KDNode* left;
	KDNode* right;

	KDNode() 
	: left(nullptr),
	  right(nullptr)
	{
		coords[0] = 0.0f;
		coords[1] = 0.0f;
	}
	KDNode(float x, float y) 
	: left(nullptr),
	  right(nullptr)
	{
		coords[0] = x;
		coords[1] = y;
	}

	KDNode(const Point& p) 
	: left(nullptr),
	  right(nullptr)
	{
		coords[0] = p[0];
		coords[1] = p[1];
	}
	~KDNode() {
		if (left) delete left;
		if (right) delete right;
	}
	void Print();
};

class CKDTree {
public:
	CKDTree()
	: m_root(nullptr),
	  m_size(0)
	{

	}

	bool Insert(const Point& p);
	Point FindNearestNeighbour(const Point& p);
	void PrintTree();
protected:
	int m_size;
private:
	KDNode* m_root;
};


class PointCloud : CKDTree{

};

#endif //POINTCLOUD_H