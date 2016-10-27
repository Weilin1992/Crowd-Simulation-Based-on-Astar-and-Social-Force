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
#define INT_MAX       2147483647  
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

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


	/*	for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}
*/
		traversal_cost = gSpatialDatabase->getTraversalCost(current_id);
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



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		agent_path.clear();
		gSpatialDatabase = _gSpatialDatabase;

		//std::vector<Util::Point> temp_agent_path;
		//Util::Point start, goal;
		//start.x = start0.x + 0.5f;
		//start.z = start0.z + 0.5f;
		//start.y = start0.y;
		//goal.x = goal0.x + 0.5f;
		//goal.z = goal0.z + 0.5f;
		//goal.y = goal0.y;

		int X = gSpatialDatabase->getNumCellsX();
		int Z = gSpatialDatabase->getNumCellsZ();
		float cellsizeX = gSpatialDatabase->getCellSizeX();
		float cellsizeZ = gSpatialDatabase->getCellSizeZ();
		float originX = gSpatialDatabase->getOriginX();
		float originZ = gSpatialDatabase->getOriginZ();

		std::vector<std::vector<AStarPlannerNode>> gridcell;
		gridcell.resize(X);
		for (size_t i = 0; i < X; i++ )
			gridcell[i].resize(Z);

		unsigned int index_s = gSpatialDatabase->getCellIndexFromLocation(start);
		unsigned int x_s, z_s;
		gSpatialDatabase->getGridCoordinatesFromIndex(index_s, x_s, z_s);
		unsigned int index_g = gSpatialDatabase->getCellIndexFromLocation(goal);
		unsigned int x_g, z_g;
		gSpatialDatabase->getGridCoordinatesFromIndex(index_g, x_g, z_g);

		std::vector<AStarPlannerNode> open;
		std::vector<AStarPlannerNode> closed;
		std::vector<AStarPlannerNode> incon;
		int weight = 1.0f;
		int weight2 = 1.0f;
		
		for (int i = 0; i < X; i++)
			for (int j = 0; j < Z; j++)
			{
				gridcell[i][j].g = INT_MAX;
			}

		gridcell[x_s][z_s].g = 0.0f;
		gridcell[x_s][z_s].h = calcH_by_Euclidian(x_s, z_s, x_g, z_g);
		gridcell[x_s][z_s].f = gridcell[x_s][z_s].g + weight*gridcell[x_s][z_s].h;
		gridcell[x_s][z_s].point = getPointFromGridIndex(index_s);

		open.push_back(gridcell[x_s][z_s]);

		float err = 0.000001f;
		bool flag = true;
		update(start, goal, gSpatialDatabase, weight, weight2, open, closed, incon, gridcell, flag);

		AStarPlannerNode node = gridcell[x_g][z_g];
		std::vector<AStarPlannerNode> temp;
		temp.push_back(node);
		//std::cout << "check" << std::endl;
		while (!((std::abs(node.point.x - gridcell[x_s][z_s].point.x) < err) && (std::abs(node.point.z - gridcell[x_s][z_s].point.z) < err)))
		{
			node = (*(node.parent));
			temp.push_back((node));
		}

		//agent_path.push_back(start);
		for (auto i = temp.end() - 1; i > temp.begin() - 1; i--)
		{
			agent_path.push_back((*i).point);
		}

		//std::cout << "num of steps is   " << agent_path.size() - 1 << std::endl;
		//std::cout << "total path length is   " << gridcell[x_g][z_g].g << std::endl;
		//std::cout << "num of expanded nodes is  " << closed.size() << std::endl;

		return true;
	}

	bool AStarPlanner::update(Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, float weight, float weight2, std::vector<AStarPlannerNode>& open, std::vector<AStarPlannerNode>& closed, std::vector<AStarPlannerNode>& incon, std::vector<std::vector<AStarPlannerNode>>& gridcell, bool flag)
	{
		gSpatialDatabase = _gSpatialDatabase;
		int X = gSpatialDatabase->getNumCellsX();
		int Z = gSpatialDatabase->getNumCellsZ();
		float cellsizeX = gSpatialDatabase->getCellSizeX();
		float cellsizeZ = gSpatialDatabase->getCellSizeZ();
		float originX = gSpatialDatabase->getOriginX();
		float originZ = gSpatialDatabase->getOriginZ();

		unsigned int index_s = gSpatialDatabase->getCellIndexFromLocation(start);
		unsigned int x_s, z_s;
		gSpatialDatabase->getGridCoordinatesFromIndex(index_s, x_s, z_s);
		unsigned int index_g = gSpatialDatabase->getCellIndexFromLocation(goal);
		unsigned int x_g, z_g;
		gSpatialDatabase->getGridCoordinatesFromIndex(index_g, x_g, z_g);


		//int loop = 0;
		std::vector<AStarPlannerNode>::iterator min;
		min = std::min_element(open.begin(), open.end(), comp);

		int iit = min - open.begin();

		unsigned int index_neighbour, index_n, x_n, z_n;
		int x_range_min, x_range_max, z_range_min, z_range_max;
		Util::Point current_position;
		float err = 0.00001f;
		float smallest_f = open[iit].f;

		while (gridcell[x_g][z_g].g - smallest_f > 0)
		{
			if (!(std::find(closed.begin(), closed.end(), open[iit]) != closed.end()))
			{
				closed.push_back(open[iit]);
			}
			//std::cout << "size of open list   " << open.size()  << std::endl;

			//update surrounding cells cost
			index_n = gSpatialDatabase->getCellIndexFromLocation(open[iit].point);
			gSpatialDatabase->getGridCoordinatesFromIndex(index_n, x_n, z_n);
			x_range_min = MAX(x_n - OBSTACLE_CLEARANCE, 0);
			x_range_max = MIN(x_n + OBSTACLE_CLEARANCE, X);
			z_range_min = MAX(z_n - OBSTACLE_CLEARANCE, 0);
			z_range_max = MIN(z_n + OBSTACLE_CLEARANCE, Z);

			//down
			if (x_range_max == x_n + 1)
			{
				index_neighbour = gSpatialDatabase->getCellIndexFromGridCoords(x_n + 1, z_n);
				if (canBeTraversed(index_neighbour))
				{
					if (gridcell[x_n + 1][z_n].g - gridcell[x_n][z_n].g - cellsizeX > 0)
					{
						gridcell[x_n + 1][z_n].g = gridcell[x_n][z_n].g + cellsizeX;
						current_position = getPointFromGridIndex(index_neighbour);
						gridcell[x_n + 1][z_n].point = current_position;
						gridcell[x_n + 1][z_n].h = calcH_by_Euclidian(x_n + 1, z_n, x_g, z_g);
						gridcell[x_n + 1][z_n].parent = &gridcell[x_n][z_n];
						gridcell[x_n + 1][z_n].f = gridcell[x_n + 1][z_n].g + weight*gridcell[x_n + 1][z_n].h;
						if (flag == true)
							update_list1(open, closed, incon, gridcell[x_n + 1][z_n]);
						else
							update_list2(open, closed, incon, gridcell[x_n + 1][z_n]);

					}
				}
			}

			//up
			if (x_range_min == x_n - 1)
			{
				index_neighbour = gSpatialDatabase->getCellIndexFromGridCoords(x_n - 1, z_n);
				if (canBeTraversed(index_neighbour))
				{
					if (gridcell[x_n - 1][z_n].g - gridcell[x_n][z_n].g - cellsizeX > 0)
					{
						gridcell[x_n - 1][z_n].g = gridcell[x_n][z_n].g + cellsizeX;
						current_position = getPointFromGridIndex(index_neighbour);
						gridcell[x_n - 1][z_n].point = current_position;
						gridcell[x_n - 1][z_n].h = calcH_by_Euclidian(x_n - 1, z_n, x_g, z_g);
						gridcell[x_n - 1][z_n].parent = &gridcell[x_n][z_n];
						gridcell[x_n - 1][z_n].f = gridcell[x_n - 1][z_n].g + weight*gridcell[x_n - 1][z_n].h;
						if (flag == true)
							update_list1(open, closed, incon, gridcell[x_n - 1][z_n]);
						else
							update_list2(open, closed, incon, gridcell[x_n - 1][z_n]);
					}
				}
			}

			//right
			if (z_range_max == z_n + 1)
			{
				index_neighbour = gSpatialDatabase->getCellIndexFromGridCoords(x_n, z_n + 1);
				if (canBeTraversed(index_neighbour))
				{
					if (gridcell[x_n][z_n + 1].g - gridcell[x_n][z_n].g - cellsizeZ > 0)
					{
						gridcell[x_n][z_n + 1].g = gridcell[x_n][z_n].g + cellsizeZ;
						current_position = getPointFromGridIndex(index_neighbour);
						gridcell[x_n][z_n + 1].point = current_position;
						gridcell[x_n][z_n + 1].h = calcH_by_Euclidian(x_n, z_n + 1, x_g, z_g);
						gridcell[x_n][z_n + 1].parent = &gridcell[x_n][z_n];
						gridcell[x_n][z_n + 1].f = gridcell[x_n][z_n + 1].g + weight*gridcell[x_n][z_n + 1].h;
						if (flag == true)
							update_list1(open, closed, incon, gridcell[x_n][z_n + 1]);
						else
							update_list2(open, closed, incon, gridcell[x_n][z_n + 1]);
					}
				}
			}

			//left
			if (z_range_min == z_n - 1)
			{
				index_neighbour = gSpatialDatabase->getCellIndexFromGridCoords(x_n, z_n - 1);
				if (canBeTraversed(index_neighbour))
				{
					if (gridcell[x_n][z_n - 1].g - gridcell[x_n][z_n].g - cellsizeZ > 0)
					{
						gridcell[x_n][z_n - 1].g = gridcell[x_n][z_n].g + cellsizeZ;
						current_position = getPointFromGridIndex(index_neighbour);
						gridcell[x_n][z_n - 1].point = current_position;
						gridcell[x_n][z_n - 1].h = calcH_by_Euclidian(x_n, z_n - 1, x_g, z_g);
						gridcell[x_n][z_n - 1].parent = &gridcell[x_n][z_n];
						gridcell[x_n][z_n - 1].f = gridcell[x_n][z_n - 1].g + weight*gridcell[x_n][z_n - 1].h;
						if (flag == true)
							update_list1(open, closed, incon, gridcell[x_n][z_n - 1]);
						else
							update_list2(open, closed, incon, gridcell[x_n][z_n - 1]);
					}
				}
			}

			//down-left
			if (x_range_max == x_n + 1 && z_range_min == z_n - 1)
			{
				index_neighbour = gSpatialDatabase->getCellIndexFromGridCoords(x_n + 1, z_n - 1);
				if (canBeTraversed(index_neighbour))
				{
					if (gridcell[x_n + 1][z_n - 1].g - gridcell[x_n][z_n].g - weight2*std::sqrt(cellsizeX*cellsizeX + cellsizeZ*cellsizeZ) > 0)
					{
						gridcell[x_n + 1][z_n - 1].g = gridcell[x_n][z_n].g + weight2*std::sqrt(cellsizeX*cellsizeX + cellsizeZ*cellsizeZ);
						current_position = getPointFromGridIndex(index_neighbour);
						gridcell[x_n + 1][z_n - 1].point = current_position;
						gridcell[x_n + 1][z_n - 1].h = calcH_by_Euclidian(x_n + 1, z_n - 1, x_g, z_g);
						gridcell[x_n + 1][z_n - 1].parent = &gridcell[x_n][z_n];
						gridcell[x_n + 1][z_n - 1].f = gridcell[x_n + 1][z_n - 1].g + weight*gridcell[x_n + 1][z_n - 1].h;
						if (flag == true)
							update_list1(open, closed, incon, gridcell[x_n + 1][z_n - 1]);
						else
							update_list2(open, closed, incon, gridcell[x_n + 1][z_n - 1]);
					}
				}
			}

			//up-left
			if (x_range_min == x_n - 1 && z_range_min == z_n - 1)
			{
				index_neighbour = gSpatialDatabase->getCellIndexFromGridCoords(x_n - 1, z_n - 1);
				if (canBeTraversed(index_neighbour))
				{
					if (gridcell[x_n - 1][z_n - 1].g - gridcell[x_n][z_n].g - weight2*std::sqrt(cellsizeX*cellsizeX + cellsizeZ*cellsizeZ) > 0)
					{
						gridcell[x_n - 1][z_n - 1].g = gridcell[x_n][z_n].g + weight2*std::sqrt(cellsizeX*cellsizeX + cellsizeZ*cellsizeZ);
						current_position = getPointFromGridIndex(index_neighbour);
						gridcell[x_n - 1][z_n - 1].point = current_position;
						gridcell[x_n - 1][z_n - 1].h = calcH_by_Euclidian(x_n - 1, z_n - 1, x_g, z_g);
						gridcell[x_n - 1][z_n - 1].parent = &gridcell[x_n][z_n];
						gridcell[x_n - 1][z_n - 1].f = gridcell[x_n - 1][z_n - 1].g + weight*gridcell[x_n - 1][z_n - 1].h;
						if (flag == true)
							update_list1(open, closed, incon, gridcell[x_n - 1][z_n - 1]);
						else
							update_list2(open, closed, incon, gridcell[x_n - 1][z_n - 1]);
					}
				}
			}

			//up-right
			if (x_range_min == x_n - 1 && z_range_max == z_n + 1)
			{
				index_neighbour = gSpatialDatabase->getCellIndexFromGridCoords(x_n - 1, z_n + 1);
				if (canBeTraversed(index_neighbour))
				{
					if (gridcell[x_n - 1][z_n + 1].g - gridcell[x_n][z_n].g - weight2*std::sqrt(cellsizeX*cellsizeX + cellsizeZ*cellsizeZ) > 0)
					{
						gridcell[x_n - 1][z_n + 1].g = gridcell[x_n][z_n].g + weight2*std::sqrt(cellsizeX*cellsizeX + cellsizeZ*cellsizeZ);
						current_position = getPointFromGridIndex(index_neighbour);
						gridcell[x_n - 1][z_n + 1].point = current_position;
						gridcell[x_n - 1][z_n + 1].h = calcH_by_Euclidian(x_n - 1, z_n + 1, x_g, z_g);
						gridcell[x_n - 1][z_n + 1].parent = &gridcell[x_n][z_n];
						gridcell[x_n - 1][z_n + 1].f = gridcell[x_n - 1][z_n + 1].g + weight*gridcell[x_n - 1][z_n + 1].h;
						if (flag == true)
							update_list1(open, closed, incon, gridcell[x_n - 1][z_n + 1]);
						else
							update_list2(open, closed, incon, gridcell[x_n - 1][z_n + 1]);
					}
				}
			}

			//down-right
			if (x_range_max == x_n + 1 && z_range_max == z_n + 1)
			{
				index_neighbour = gSpatialDatabase->getCellIndexFromGridCoords(x_n + 1, z_n + 1);
				if (canBeTraversed(index_neighbour))
				{
					if (gridcell[x_n + 1][z_n + 1].g - gridcell[x_n][z_n].g - weight2*std::sqrt(cellsizeX*cellsizeX + cellsizeZ*cellsizeZ) > 0)
					{
						gridcell[x_n + 1][z_n + 1].g = gridcell[x_n][z_n].g + weight2*std::sqrt(cellsizeX*cellsizeX + cellsizeZ*cellsizeZ);
						current_position = getPointFromGridIndex(index_neighbour);
						gridcell[x_n + 1][z_n + 1].point = current_position;
						gridcell[x_n + 1][z_n + 1].h = calcH_by_Euclidian(x_n + 1, z_n + 1, x_g, z_g);
						gridcell[x_n + 1][z_n + 1].parent = &gridcell[x_n][z_n];
						gridcell[x_n + 1][z_n + 1].f = gridcell[x_n + 1][z_n + 1].g + weight*gridcell[x_n + 1][z_n + 1].h;
						if (flag == true)
							update_list1(open, closed, incon, gridcell[x_n + 1][z_n + 1]);
						else
							update_list2(open, closed, incon, gridcell[x_n + 1][z_n + 1]);
					}
				}
			}
			open.erase(open.begin() + iit);
			if (open.empty() == true)
			{
				std::cout << " cant reach the target" << std::endl;
				return false;
			}
			//loop++;
			min = std::min_element(open.begin(), open.end(), comp);

			iit = min - open.begin();
			smallest_f = open[iit].f;
		}
	}

	void AStarPlanner::update_list2(std::vector<AStarPlannerNode>& open, std::vector<AStarPlannerNode>& closed, std::vector<AStarPlannerNode>& incon, AStarPlannerNode AStarPlannerNode)
	{
		//not 1st time update node, if the incon node in closed, insert to incon list, else insert to open with new value
		if (std::find(closed.begin(), closed.end(), AStarPlannerNode) != closed.end())
		{
			closed.push_back(AStarPlannerNode);
		}
		else
		{
			if (std::find(open.begin(), open.end(), AStarPlannerNode) == open.end())
				open.push_back(AStarPlannerNode);
		}
	}

	void AStarPlanner::update_list1(std::vector<AStarPlannerNode>& open, std::vector<AStarPlannerNode>& closed, std::vector<AStarPlannerNode>& incon, AStarPlannerNode AStarPlannerNode)
	{
		//1st time update node, add all the incons nodes to open list 
		if (std::find(open.begin(), open.end(), AStarPlannerNode) != open.end())
		{
			open.erase(std::remove(open.begin(), open.end(), AStarPlannerNode), open.end());
		}
		open.push_back(AStarPlannerNode);
	}


	double AStarPlanner::calcH_by_Euclidian(unsigned int x_n, unsigned int z_n, unsigned int x_g, unsigned int z_g )
	{
		float cellsizeX = gSpatialDatabase->getCellSizeX();
		float cellsizeZ = gSpatialDatabase->getCellSizeZ();
		return std::sqrt(pow(std::abs((int)(x_n - x_g))*cellsizeX, 2) + pow(std::abs((int)(z_n - z_g))*cellsizeZ, 2));
	}

	double AStarPlanner::calcH_by_Manhattan(unsigned int x_n, unsigned int z_n, unsigned int x_g, unsigned int z_g)
	{
		float cellsizeX = gSpatialDatabase->getCellSizeX();
		float cellsizeZ = gSpatialDatabase->getCellSizeZ();
		return (std::abs((int)(x_n - x_g))*cellsizeX + std::abs((int)(z_n - z_g))*cellsizeZ);
	}

	bool AStarPlanner::comp(AStarPlannerNode i, AStarPlannerNode j)
	{
		if (i.f < j.f && std::abs(i.f - j.f) > 0.0000001f)
			return true;
		else if (i.f > j.f && std::abs(i.f - j.f) > 0.0000001f)
			return false;
		else
		{
			if (i.g > j.g)
				return true;
			else
				return false;
		}
	}
}