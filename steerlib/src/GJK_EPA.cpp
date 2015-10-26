/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"

SteerLib::GJK_EPA::GJK_EPA()
{
	

}



//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	//Simplex to be build unsing points found via Minkowski differences
	std::vector<Util::Vector> simplex;

	//flag to be set true if the simplex contains origin (0,0)
	bool col = false;

    return col; // There is no collision
}

bool GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	//Simplex to be build using points found via Minkowski differences
	std::vector<Util::Vector> simplex;

	//flag to be set true if the simplex contains origin (0,0)
	bool is_colliding = false;

	//calculates for collision
	//gets centers of Minkowski difference viadifference of center of both shapes.
	Util::Vector direction = calculateCenter(_shapeA) - calculateCenter(_shapeB);
	if (direction == (0, 0, 0)) {
		return true;
	}
	simplex.push_back(support(_shapeA,_shapeB,direction));
	direction = direction*-1;
	simplex.push_back(support(_shapeA, _shapeB, direction));

	if (is_colliding) {
		//Still searching for collision
		return (simplex, true);
	}
	else {
		//there is no collision
		return (NULL, false);
	}
}
Util::Vector calculateCenter(const std::vector<Util::Vector>& _shape) {
	Util::Vector retVect(0, 0, 0);

	//add the x/y/z vals of every point in the shape
	for (std::vector<Util::Vector>::const_iterator iter = _shape.begin(); iter != _shape.end(); ++iter)
	{
		retVect.x = retVect.x + iter->x;
		retVect.y = retVect.y + iter->y;
		retVect.z = retVect.z + iter->z;
	}

	//divide by number of points to get average
	if (_shape.size() > 0)
	{
		retVect.x = retVect.x / _shape.size();
		retVect.y = retVect.y / _shape.size();
		retVect.z = retVect.z / _shape.size();
	}

	return retVect;
}
Util::Vector support(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, Util::Vector direction)
{
	Util::Vector pointA = getVector(_shapeA, direction);
	Util::Vector pointB = getVector(_shapeB, (direction*-1));

	return (pointA - pointB);

}

Util::Vector getVector(const std::vector<Util::Vector>& _shape, Util::Vector direction)
{
	std::vector<Util::Vector>::const_iterator ret_vect = _shape.begin();
	float dotprod = 0;
	float greatest = -999999999;
	
	//calculate point farthest from direction
	for (std::vector<Util::Vector>::const_iterator vect = _shape.begin(); vect != _shape.end(); vect++) {
		dotprod = vect->x*direction.x + vect->y*direction.y + vect->z*direction.z;
		if (dotprod  >greatest) {
			greatest = dotprod;
			ret_vect = vect;
		}

	}

	Util::Vector returnVect(ret_vect->x, ret_vect->y, ret_vect->z);
	return returnVect;
}