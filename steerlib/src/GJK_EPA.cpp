/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"

SteerLib::GJK_EPA::GJK_EPA()
{


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

Util::Vector support(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, Util::Vector direction)
{
	Util::Vector pointA = farthestPoint(_shapeA, direction);
	Util::Vector pointB = farthestPoint(_shapeB, (direction*-1));

	return (pointA - pointB);

}

Util::Vector calculateDirection(const std::vector<Util::Vector>& simplex)
{
	std::vector<Util::Vector>::const_reverse_iterator reverse = simplex.rbegin();
	Util::Vector last(reverse->x, reverse->y, reverse->z);
	reverse = reverse + 1;
	Util::Vector nextLast(reverse->x, reverse->y, reverse->z);


	//calculates new direction in way that most likely encloses origin if collision exists
	Util::Vector Origin(0, 0, 0);
	Util::Vector AB = nextLast - last;
	Util::Vector AO = Origin - last;

	Util::Vector retDir = cross(cross(AB,AO), AB);
	return retDir;
}


// (AC x AB) x AB = AB(AB.dot(AC)) - AC(AB.dot(AB))
Util::Vector tripCross(Util::Vector AC, Util::Vector AB) {
	
	return cross(cross(AC,AB),AB);
}



bool checkSimplex(std::vector<Util::Vector>& simplex)
{  // get the last point added to the simplex
	std::vector<Util::Vector>::const_reverse_iterator reverse = simplex.rbegin();
	Util::Vector A(reverse->x, reverse->y, reverse->z);
	// compute AO 
	Util::Vector AO = Util::Vector(0, 0, 0) - A;
	if (simplex.size() == 3) {

		// then its the triangle case
		// get b and c
		reverse++;
		Util::Vector B(reverse->x, reverse->y, reverse->z);
		reverse++;
		Util::Vector C(reverse->x, reverse->y, reverse->z);
		// compute the edges
		Util::Vector AB = B - A;
		Util::Vector AC = C - A;
		// compute the normals
		Util::Vector ABperp = tripCross(AC, AB);
		Util::Vector ACperp = tripCross(AB, AC);
		
		//Util::Vector ABperp = tripCross(AB, AC); // (AB X AC) X AC
		//Util::Vector ACperp = tripCross(AC, AB); // (AC X AB) X AC

		// is the origin in R4
		if (dot(ABperp,AO) > 0) {
			// remove point c
			simplex.erase(simplex.begin());
			return false;
		}
		else {
			// is the origin in R3
			if (dot(ACperp,AO) > 0) {
				// remove point b
				simplex.erase(simplex.begin() + 1);
				return false;
			}
			else {
				// otherwise we know its in R5 so we can return true
				return true;
			}
		}
	}
	return true;
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
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
	simplex.push_back(support(_shapeA, _shapeB, direction));
	direction = direction*-1;
	simplex.push_back(support(_shapeA, _shapeB, direction));

	//build the simplex
	while(true) {
		//get new direction
		direction = calculateDirection(simplex);

		//add new minkowski difference vertex
		simplex.push_back(support(_shapeA, _shapeB, direction));

		//check simplex contains origin
		if (simplex[2] * direction <= 0) {
			//simplex past origin
			return false;
		}
		else {
			if (checkSimplex(simplex))
			{
				
				return true;
			}
		}

	}
}
