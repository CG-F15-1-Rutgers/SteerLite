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
	
	//first 2 points on the simplex
	simplex.push_back(support(_shapeA,_shapeB,direction));
	direction = direction*-1;
	simplex.push_back(support(_shapeA, _shapeB, direction));

	//build the simplex
	while (true) {
		//get new direction
		direction = calDir(simplex);

		//add new minkowski difference vertex
		simplex.push_back(support(_shapeA, _shapeB, direction));

		//check simplex contains origin


	}




	if (is_colliding) {
		//Still searching for collision
		return (simplex, true);
	}
	else {
		//there is no collision
		return (NULL, false);
	}
}

Util::Vector support(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, Util::Vector direction)
{
	Util::Vector pointA = farthestPoint(_shapeA, direction);
	Util::Vector pointB = farthestPoint(_shapeB, (direction*-1));

	return (pointA - pointB);

}

Util::Vector farthestPoint(const std::vector<Util::Vector>& _shape, Util::Vector direction)
{
	//iterator to keep track of current farthest point
	std::vector<Util::Vector>::const_iterator currentFarthest = _shape.begin();

	float currentDot = 0;
	float highestDot = -999999999;

	//loop through all points in shape, find one with greatest dot product with the desired direction. will be farthest point
	for (std::vector<Util::Vector>::const_iterator iter = _shape.begin(); iter != _shape.end(); ++iter)
	{
		currentDot = (iter->x * direction.x) + (iter->y * direction.y) + (iter->z * direction.z);
		if (currentDot > highestDot)
		{
			highestDot = currentDot;
			currentFarthest = iter;
		}
	}
	Util::Vector returnVector(currentFarthest->x, currentFarthest->y, currentFarthest->z);
	return returnVector;
}

Util::Vector calDir(const std::vector<Util::Vector>& simplex)
{
	std::vector<Util::Vector>::const_reverse_iterator reverse = simplex.rbegin();
	Util::Vector last(reverse->x,reverse->y,reverse->z);
	reverse = reverse + 1;
	Util::Vector nextLast(reverse->x, reverse->y, reverse->z);


	//calculates new direction in way that most likely encloses origin if collision exists
	Util::Vector Origin(0, 0, 0);
	Util::Vector AB = nextLast - last;
	Util::Vector AO = Origin - nextLast;

	Util::Vector retDir = AO*(AB*AB) - AB*(AB*AO);
	return retDir;
}
