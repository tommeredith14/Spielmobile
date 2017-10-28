#ifndef MAP_H
#define MAP_H

#include <vector>
#include <string>

class map {
public:
	map(std::string file);
	~map();
	bool IsSpaceOccupied(double x, double y);



private:
	double m_width;
	double m_height;


};


#endif //MAP_H

