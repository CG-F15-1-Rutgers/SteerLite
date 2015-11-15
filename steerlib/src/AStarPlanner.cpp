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


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))
#define WEIGHT 8
#define DIAG 1

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}

	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}

	double AStarPlanner::heuristic(int start_index, int goal_index) {
		double dist = 0;

		Util::Point start, goal;
		start = getPointFromGridIndex(start_index);
		goal = getPointFromGridIndex(goal_index);

		//Use Euclidean
		if (heur) {
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

	bool AStarPlanner::reconstruct_path(std::vector<Util::Point>& agent_path, int curr, std::map<int, AStarPlannerNode*> nodeMap) {
		//where things "came_from"
		std::vector<Util::Point> came_from;

		//total path
		AStarPlannerNode* tot_Path = nodeMap[curr];
		
		//Traverse and populate came_from
		came_from.push_back((*tot_Path).point);
		while ((*tot_Path).parent != nullptr) {
			tot_Path = (*tot_Path).parent;
			came_from.push_back((*tot_Path).point);
		}

		//Construct forward path
		for (int i = came_from.size() - 1; i >= 0; --i) {
			agent_path.push_back(came_from.at(i));
		}


		std::cout << "\nPath length: " << came_from.size() << '\n';
		return true;
	}


	int AStarPlanner::getCurr(std::set<int> openset, std::map<int, AStarPlannerNode*> nodeMap) {
		std::set<int>::iterator it;
		double temporary = INFINITY;
		/*
		for f score ties
		true = favor larger g score
		false = favor lower g score
		*/
		bool larger = true;
		for (std::set<int>::iterator i = openset.begin(); i != openset.end(); ++i)
		{
			if ((*nodeMap[(*i)]).f < temporary) {
				temporary = (*nodeMap[(*i)]).f;
				it = i;
			}
			else if ((*nodeMap[(*i)]).f == temporary)
			{
				//Tie Breaker
				if (larger) {
					if ((*nodeMap[(*it)]).g < (*nodeMap[(*i)]).g) {
						it = i;
					}
				}
				else
				{
					if ((*nodeMap[(*it)]).g >(*nodeMap[(*i)]).g)
					{
						it = i;
					}
				}
			}
		} return (*it);
	}


	void AStarPlanner::expandNeighbor(int curr, int goal, std::set<int>& open, std::set<int> closed, std::map<int, AStarPlannerNode*>& plannerNodeMap) {
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(curr, x, z);
		expandedNum++;
		for (int a = MAX(x - 1, 0); a < MIN(x + 2, gSpatialDatabase->getNumCellsX()); a = a + GRID_STEP) {

			for (int b = MAX(z - 1, 0); b < MIN(z + 2, gSpatialDatabase->getNumCellsZ()); b = b + GRID_STEP) {
				int neighbor = gSpatialDatabase->getCellIndexFromGridCoords(a, b);
				if (closed.count(neighbor) == 0 && canBeTraversed(neighbor) == true) {
					double g_temp;

					if ((a == x) || (b == z)) {
						g_temp = (*plannerNodeMap[curr]).g + (DIAG*gSpatialDatabase->getTraversalCost(neighbor));
					}
					else {
						g_temp = (*plannerNodeMap[curr]).g + gSpatialDatabase->getTraversalCost(neighbor);
					}

					if ((*plannerNodeMap[neighbor]).g > g_temp) {
						(*plannerNodeMap[neighbor]).g = g_temp;
						(*plannerNodeMap[neighbor]).f = (*plannerNodeMap[neighbor]).g + WEIGHT * heuristic(neighbor, goal);

						//if there is one neighbor
						if (open.count(neighbor) == 1) {
							open.erase(open.find(neighbor));
						}
						open.insert(neighbor);
						(*plannerNodeMap[neighbor]).parent = plannerNodeMap[curr];
					}
				}
			}
		}
	}


	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//TODO

		/*
		Switch heuristics here
		true for Euclidean, false for manhattan
		*/
		heur = false;
		expandedNum = 0;
		//Get f and g scores in map
		std::map<int, AStarPlannerNode*> plannerNodeMap;
		for (int a = 0; a < gSpatialDatabase->getNumCellsX(); a++) {
			for (int b = 0; b < gSpatialDatabase->getNumCellsZ(); b++) {
				int index = gSpatialDatabase->getCellIndexFromGridCoords(a, b);
				plannerNodeMap[index] = new AStarPlannerNode(getPointFromGridIndex(index), (double)INFINITY, (double)INFINITY, nullptr);
			}
		}
		int startID = gSpatialDatabase->getCellIndexFromLocation(start);
		int goalID = gSpatialDatabase->getCellIndexFromLocation(goal);
		Util::Point startCenter = getPointFromGridIndex(startID);
		Util::Point goalCenter = getPointFromGridIndex(goalID);
		(*plannerNodeMap[startID]).g = 0;
		(*plannerNodeMap[startID]).f = (*plannerNodeMap[startID]).g + WEIGHT*heuristic(startID, goalID);

		std::set<int> closed;
		std::set<int> open;
		open.insert(startID);

		while (!open.empty()) {
			//find node in open with lowest f
			int curr = getCurr(open, plannerNodeMap);

			//remove from open and add to closed
			closed.insert(curr);
			open.erase(open.find(curr));

			//check for goal!!!!!!!!!!!!!!!!!!!!!!
			if (curr == goalID) {
				return reconstruct_path(agent_path, curr, plannerNodeMap);
			}

			//check neighbor's g and f scores, add to open
			expandNeighbor(curr, goalID, open, closed, plannerNodeMap);
			std::cout << "\nExpanded Nodes: " << expandedNum << std::endl;
		}


		std::cout << "\nIn A*";

		return false;
	}
}
