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

		double smallest_F = gridcell[x_s][z_s].f;

		//std::cout << " NumCellsX and NumCellsZ " << " " << X << " " << Z << std::endl;
		//std::cout << " start point" << " " << start.x << " " << start.z << std::endl;
		//std::cout << " start index " << " " << index_s << " " << x_s << " " << z_s << std::endl;
		//std::cout << " goal point" << " " << goal.x << " " << goal.z << std::endl;
		//std::cout << " goal index" << " " << index_g <<  " " << x_g << " " << z_g << std::endl;
		////std::cout << " gridcell[x_g][z_g].g " << " " << gridcell[x_g][z_g].g << std::endl;
		////std::cout << " smallest_F " << " " << smallest_F << std::endl;
		//std::cout << " cellsizeX " << " " << cellsizeX << std::endl;
		//std::cout << " cellsizeZ " << " " << cellsizeZ << std::endl;
		//std::cout << " originX " << " " << originX << std::endl;
		//std::cout << " originZ " << " " << originZ << std::endl;

		int iit = 0;
		int loop = 0;
		unsigned int index_neighbour, index_n, x_n, z_n;
		int x_range_min, x_range_max, z_range_min, z_range_max;
		Util::Point current_position;
		std::vector<AStarPlannerNode>::iterator min;
		float err = 0.01f;
		while (gridcell[x_g][z_g].g - smallest_F > -err)
		{
			if (!(std::find(closed.begin(), closed.end(), open[iit]) != closed.end()))
			{
				closed.push_back(open[iit]);
			}

			//std::cout << " loop" << loop << std::endl;
			////std::cout << " close_size" << closed.size() << std::endl;
			//std::cout << " open_size" << open.size() << std::endl;
			//std::cout << " smallest_F" << smallest_F << std::endl;
			////std::cout << " f(goal)" << gridcell[x_g][z_g].g << std::endl;
			//std::cout << " iit" << iit << std::endl;
			//std::cout << " iit" << open[iit].point.x << " " << open[iit].point.z << std::endl;

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
						//delete from the list if already presented
						if (std::find(open.begin(), open.end(), gridcell[x_n + 1][z_n]) != open.end())
						{
							open.erase(std::remove(open.begin(), open.end(), gridcell[x_n + 1][z_n]), open.end());
						}
						open.push_back(gridcell[x_n + 1][z_n]);
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
						//delete from the list if already presented
						if (std::find(open.begin(), open.end(), gridcell[x_n - 1][z_n]) != open.end())
						{
							open.erase(std::remove(open.begin(), open.end(), gridcell[x_n - 1][z_n]), open.end());
						}
						open.push_back(gridcell[x_n - 1][z_n]);
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
						//delete from the list if already presented
						if (std::find(open.begin(), open.end(), gridcell[x_n][z_n + 1]) != open.end())
						{
							open.erase(std::remove(open.begin(), open.end(), gridcell[x_n][z_n + 1]), open.end());
						}
						open.push_back(gridcell[x_n][z_n + 1]);
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
						//delete from the list if already presented
						if (std::find(open.begin(), open.end(), gridcell[x_n][z_n - 1]) != open.end())
						{
							open.erase(std::remove(open.begin(), open.end(), gridcell[x_n][z_n - 1]), open.end());
						}
						open.push_back(gridcell[x_n][z_n - 1]);
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
						//delete from the list if already presented
						if (std::find(open.begin(), open.end(), gridcell[x_n + 1][z_n - 1]) != open.end())
						{
							open.erase(std::remove(open.begin(), open.end(), gridcell[x_n + 1][z_n - 1]), open.end());
						}
						open.push_back(gridcell[x_n + 1][z_n - 1]);
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
						//delete from the list if already presented
						if (std::find(open.begin(), open.end(), gridcell[x_n - 1][z_n - 1]) != open.end())
						{
							open.erase(std::remove(open.begin(), open.end(), gridcell[x_n - 1][z_n - 1]), open.end());
						}
						open.push_back(gridcell[x_n - 1][z_n - 1]);
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
						//delete from the list if already presented
						if (std::find(open.begin(), open.end(), gridcell[x_n - 1][z_n + 1]) != open.end())
						{
							open.erase(std::remove(open.begin(), open.end(), gridcell[x_n - 1][z_n + 1]), open.end());
						}
						open.push_back(gridcell[x_n - 1][z_n + 1]);
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
						//delete from the list if already presented
						if (std::find(open.begin(), open.end(), gridcell[x_n + 1][z_n + 1]) != open.end())
						{
							open.erase(std::remove(open.begin(), open.end(), gridcell[x_n + 1][z_n + 1]), open.end());
						}
						open.push_back(gridcell[x_n + 1][z_n + 1]);
					}
				}
			}
			open.erase(open.begin() + iit);
			if (open.empty() == true)
			{
				std::cout << " cant reach the target" << std::endl;
				return false;
			}
			std::vector<AStarPlannerNode>::iterator min;
			min = std::min_element(open.begin(), open.end());

			
			iit = min - open.begin();
			smallest_F = open[iit].f;
			loop++;
		}

		//std::cout << "search finished" << std::endl;
		//std::cout << " loop" << loop << std::endl;
		////std::cout << " close_size" << closed.size() << std::endl;
		//std::cout << " open_size" << open.size() << std::endl;
		//std::cout << " smallest_F" << smallest_F << std::endl;
		//std::cout << " f(goal)" << gridcell[x_g][z_g].g << std::endl;

		AStarPlannerNode node = gridcell[x_g][z_g];
		std::vector<AStarPlannerNode> temp;
		temp.push_back(node);
		//std::cout << "check" << std::endl;
		while (!((std::abs(node.point.x - gridcell[x_s][z_s].point.x) < err) && (std::abs(node.point.z - gridcell[x_s][z_s].point.z) < err)))
		{
			node = (*(node.parent));
			//std::cout << "node  " << node.point.x << " " << node.point.z << " " << std::endl;
			temp.push_back((node));
			//std::cout << "check1" << std::endl;
		}

		//std::cout << "before agent_path is modified" << std::endl;
		//agent_path.push_back(start);
		for (auto i = temp.end() - 1; i > temp.begin() - 1; i--)
		{
			agent_path.push_back((*i).point);
		}

		//std::cout << "num of steps is   " << agent_path.size() - 1 << std::endl;
		//std::cout << "total path length is   " << gridcell[x_g][z_g].g << std::endl;
		//std::cout << "num of expanded nodes is  " << closed.size() << std::endl;

		//std::cout<<"\nIn A*";
		return true;
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