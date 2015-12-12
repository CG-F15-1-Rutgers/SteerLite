//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"
#include <cmath>


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 0
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner() {}

	AStarPlanner::~AStarPlanner() {}

	bool AStarPlanner::canBeTraversed(int id)
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x - OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z - OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i <= x_range_max; i += GRID_STEP)
		{
			for (int j = z_range_min; j <= z_range_max; j += GRID_STEP)
			{
				if (i < 0 || j < 0) {
					continue;
				}

				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				traversal_cost += gSpatialDatabase->getTraversalCost(index);

			}
		}

		if (traversal_cost > COLLISION_COST)
			return false;
		return true;
	}

	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}

/*
*
*
*
*
*
*/
	int AStarPlanner::getIndexFromPoint(Util::Point p)
	{
		return gSpatialDatabase->getCellIndexFromLocation(p);
	}

	double AStarPlanner::heuristic(Util::Point start, Util::Point goal)
	{
		double dist = 0;

		//Use Euclidean
		if (false) {
			dist = (double)sqrt((double)(start.x - goal.x)*(start.x - goal.x)
				+ (double)(start.y - goal.y)*(start.y - goal.y) //not necessary but oh well
				+ (double)(start.z - goal.z)*(start.z - goal.z));
		}
		//Use Manhattan
		else {
			dist = (double)abs((double)(start.x - goal.x))
				+ (double)abs((double)(start.y - goal.y))
				+ (double)abs((double)(start.z - goal.z));
		}

		return dist;
	}



	//is the node on the grid
	bool AStarPlanner::onGrid(SteerLib::GridDatabase2D * _gSpatialDatabase, int nodeIndex)
	{
		unsigned int xIndex, zIndex;
		_gSpatialDatabase->getGridCoordinatesFromIndex(nodeIndex, xIndex, zIndex);
		if (xIndex >= _gSpatialDatabase->getNumCellsX() || zIndex >= _gSpatialDatabase->getNumCellsZ())
		{
			return false;
		}
		return true;
	}


	void AStarPlanner::reconstructPath(std::vector<Util::Point>& agent_path, std::map<int, int> cameFrom, int goalIndex)
	{
		int currentIndex = goalIndex;
		agent_path.push_back(getPointFromGridIndex(currentIndex));
		while (cameFrom.count(currentIndex))
		{
			currentIndex = cameFrom[currentIndex];
			agent_path.insert(agent_path.begin(), getPointFromGridIndex(currentIndex));
		}
	}

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;
		// Setup
		std::set<int> closedSet;
		std::set<int> openSet;
		std::map<int, int> cameFrom;
		std::map<int, double> gScore;
		std::map<int, double> fScore;

		openSet.insert(getIndexFromPoint(start));
		gScore.insert(std::pair<int, double>(getIndexFromPoint(start), 0));
		fScore.insert(std::pair<int, double>(getIndexFromPoint(start), heuristic(start, goal)));

		while (!openSet.empty())
		{

			//get first index
			double minF = DBL_MAX, minG = DBL_MAX;
			int currentIndex = -1;

			std::set<int>::iterator it = openSet.begin();
			std::set<int>::iterator it_end = openSet.end();
			for (; it != it_end; ++it)
			{
				int index = *it;
				double f_value = fScore[index];
				double g_value = gScore[index];
				if (minF > f_value || (minF == f_value && minG > g_value))
				{
					minF = f_value;
					minG = g_value;
					currentIndex = index;
				}
			}

			openSet.erase(currentIndex);

			if (currentIndex == getIndexFromPoint(goal))
			{
				// currentIndex is goal 
				reconstructPath(agent_path, cameFrom, currentIndex);
				return true;
			}
			closedSet.insert(currentIndex);

			//Get Neighbors
			std::vector<int> neighbors;

			unsigned int x, z;
			gSpatialDatabase->getGridCoordinatesFromIndex(currentIndex, x, z);
			int minX, maxX, minZ, maxZ;

			minX = MAX(x - GRID_STEP, 0);
			maxX = MIN(x + GRID_STEP, gSpatialDatabase->getNumCellsX());

			minZ = MAX(z - GRID_STEP, 0);
			maxZ = MIN(z + GRID_STEP, gSpatialDatabase->getNumCellsZ());

			for (int i = minX; i <= maxX; i += GRID_STEP)
			{
				for (int j = minZ; j <= maxZ; j += GRID_STEP)
				{
					int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
					if (index != currentIndex)
					{
						neighbors.insert(neighbors.end(), index);
					}
				}
			}
			std::vector<int>::iterator neighborsIter = neighbors.begin();
			std::vector<int>::iterator neighborsIterEnd = neighbors.end();


			for (; neighborsIter != neighborsIterEnd; ++neighborsIter)
			{
				int neighborIndex = *neighborsIter;

				if (closedSet.count(neighborIndex) == 1)
				{
					continue;
				}
				if (!canBeTraversed(neighborIndex))
				{
					// don't count things in obstacles
					continue;
				}
				if (!onGrid(gSpatialDatabase, neighborIndex))
				{
					// Don't count because off grid
					continue;
				}

				double tentativeGScore = gScore[currentIndex] + 1.0;

				if (gScore.count(neighborIndex) == 0 || tentativeGScore < gScore[neighborIndex])
				{
					cameFrom[neighborIndex] = currentIndex;
					gScore[neighborIndex] = tentativeGScore;
					fScore[neighborIndex] = gScore[neighborIndex] + heuristic(getPointFromGridIndex(neighborIndex), goal);

					assert(currentIndex != neighborIndex);

					openSet.insert(neighborIndex);
				}
			}
		}

		return false;
	}
}