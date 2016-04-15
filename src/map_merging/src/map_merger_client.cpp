#include "grid_map.h"
#include "io.h"
#include "common.h"
#include "hough.h"
#include "manipulatemap.h"

#include <iterator>
#include <iostream>
#include <algorithm>

#include <sys/time.h>

//ROS libraries
#include "map_merging/MergeTwoMaps.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

const unsigned int n_hypothesis = 4;

using namespace mapmerge;

#define FREE_CELL_MAP 255
#define OCCUPIED_CELL_MAP 0
#define UNKNOWN_CELL_MAP 127

#define FREE_CELL_OCC 0
#define OCCUPIED_CELL_OCC 100
#define UNKNOWN_CELL_OCC -1

void toOccGridMapRepr(std::vector<std::vector<unsigned int> > data, std::vector<std::vector<signed char> > &occ);

double RESOLUTION;
int SIZE;

nav_msgs::OccupancyGrid initializeOccupancyGrid(int length, double resolution) {
    RESOLUTION = resolution;
    SIZE  = length;
    nav_msgs::OccupancyGrid og;
    // og.info.resolution = resolution;
    // og.header.frame_id = "/world";
    // og.info.origin.position.x = -SIZE/2;	//not sure why the negative or the divide by 2 but it works
    // og.info.origin.position.y = -SIZE/2;	//not sure why the negative or the divide by 2 but it works
    // og.header.stamp = ros::Time::now();
    og.info.width = SIZE;
    og.info.height = SIZE;
    og.data.resize(SIZE * SIZE);
    return og;
}

nav_msgs::OccupancyGrid updateOccupancyGrid(nav_msgs::OccupancyGrid og, int x, int y,int val) {
    og.data[((y*og.info.width)+x)] = val;
    return og;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_merger_client");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<map_merging::MergeTwoMaps>("map_merger");
	map_merging::MergeTwoMaps srv;

	char file1[] = "/home/andrestoga/ros_map_merging/src/map_merging/src/datasets/LongRun6.txt";
	char file2[] = "/home/andrestoga/ros_map_merging/src/map_merging/src/datasets/LongRun7.txt";

	ROS_INFO("Loading %s and %s", file1, file2);
	grid_map a,b;
	a.load_map(900,900,file1);
	b.load_map(900,900,file2);

	// int8[] data
	// unsigned int

	// convertToOccupancyGridMap(a.grid, srv.request.map1);
	// convertToOccupancyGridMap(b.grid, srv.request.map2);

	std::vector< std::vector<signed char> > occ_a;

	toOccGridMapRepr(a.grid, occ_a);

	int size=900;
	// int map[5][5]= {		{ 0, 0, 0, 0, 0},			//Dummy Map
	// 						{-1,-1,-1,-1, 0},
	// 						{ 0, 0,-1,-1, 0},
	// 						{ 0, 0,-1,-1, 0},
	// 						{ 0, 0, 0, 0, 0}};
	nav_msgs::OccupancyGrid og= initializeOccupancyGrid(size,2);		//initialize a SIZExSIZE OccupancyGrid
	for(int row=0;row<size;row++)
	{
		for(int col=0;col<size;col++)
		{
			// og=updateOccupancyGrid(og,col,row,occ[row][col]);	//fill in OccupancyGrid with map

			og.data[((row*og.info.width)+col)] = occ_a[row][col];

			// ROS_INFO("Updating...");
		}
	}

	srv.request.map1 = og;

	srv.request.n_hypothesis = n_hypothesis;
	srv.request.hough_increment = 1;
	srv.request.randomized = false;
	srv.request.fraction = 0.5;

	if (client.call(srv))
	{
		ROS_INFO("Success!!!");

		// for (int i = 0; i < n_hypothesis; ++i)
		// {
		// 	ROS_INFO_STREAM("From the client: " << srv.response.transformations[i].ai << " " << srv.response.transformations[i].deltax << " " << srv.response.transformations[i].deltay << " " << srv.response.transformations[i].rotation);
		// }
	}
	else
	{
		ROS_ERROR("Failed to call service map_merger");
		return 1;
	}

	return 0;
}

void toOccGridMapRepr(std::vector<std::vector<unsigned int> > data, std::vector<std::vector<signed char> > &occ)
{
	occ.resize(data.size());

	// occ.resize(data.size());
	// map.info.height = data[0].size();

	// map.data.resize(map.info.width * map.info.height);

	for (int i = 0; i < data.size(); ++i)
	{
		occ[i].resize(data[i].size());

		for (int j = 0; j < data[i].size(); ++j)
		{
			switch(data[i][j])
			{
				case FREE_CELL_MAP:

					// map.data.push_back(FREE_CELL_OCC);
					occ[i][j] = FREE_CELL_OCC;

				break;

				case OCCUPIED_CELL_MAP:

					occ[i][j] = OCCUPIED_CELL_OCC;
					// map.data.push_back(OCCUPIED_CELL_OCC);

				break;

				case UNKNOWN_CELL_MAP:

					occ[i][j] = UNKNOWN_CELL_OCC;
					// map.data.push_back(UNKNOWN_CELL_OCC);

				break;

				default:
					ROS_INFO_STREAM("Error. The code shouldn't reach here!");
			}
		}	
	}
}