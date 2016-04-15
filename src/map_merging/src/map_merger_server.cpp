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

#define FREE_CELL_MAP 255
#define OCCUPIED_CELL_MAP 0
#define UNKNOWN_CELL_MAP 127

#define FREE_CELL_OCC 0
#define OCCUPIED_CELL_OCC 100
#define UNKNOWN_CELL_OCC -1

using namespace mapmerge;

grid_map convertToGridMap(nav_msgs::OccupancyGrid map);

bool merge_two_maps(map_merging::MergeTwoMaps::Request  &req,
         map_merging::MergeTwoMaps::Response &res);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_merger_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("map_merger", merge_two_maps);
  ROS_INFO("Ready to merge maps.");
  ros::spin();

  return 0;
}

grid_map convertToGridMap(nav_msgs::OccupancyGrid map)
{
  grid_map a;

  a.resize_map(map.info.width, map.info.height);

  for (int i = 0, count = 0; i < map.info.width; ++i)
  {
    for (int j = 0; j < map.info.height; ++j, ++count)
    {
      switch(map.data[count])
      {
        case FREE_CELL_OCC:

          a.grid[i][j] = static_cast<unsigned int>(FREE_CELL_MAP);

        break;

        case OCCUPIED_CELL_OCC:

          a.grid[i][j] = static_cast<unsigned int>(OCCUPIED_CELL_MAP);

        break;

        case UNKNOWN_CELL_OCC:

          a.grid[i][j] = static_cast<unsigned int>(UNKNOWN_CELL_MAP);

        break;

        default:
          ROS_INFO_STREAM("Error. The code shouldn't reach here!");
      }
    } 
  }

  return a;
}

bool merge_two_maps(map_merging::MergeTwoMaps::Request  &req,
         map_merging::MergeTwoMaps::Response &res)
{

  grid_map map1 = convertToGridMap(req.map1); 
  grid_map map2 = convertToGridMap(req.map2);

  // std::vector<transformation> hyp = get_hypothesis(map1, map2, req.n_hypothesis, req.hough_increment, req.randomized);

  char file1[] = "/home/andrestoga/ros_map_merging/src/map_merging/src/datasets/LongRun6.txt";
  char file2[] = "/home/andrestoga/ros_map_merging/src/map_merging/src/datasets/LongRun7.txt";

  grid_map a,b;
  a.load_map(900,900,file1);
  b.load_map(900,900,file2);

  // ROS_INFO("Rows: %u %u and Cols: %u %u", map1.get_rows(), map2.get_rows(), map1.get_cols(), map2.get_cols());

  // for (int i = 0; i < req.map1.info.width; ++i)
  // {
  //   for (int j = 0; j < req.map1.info.height; ++j)
  //   {
  //     // if (a.grid[i][j] != map1.grid[i][j])
  //     // {
  //     //   ROS_INFO("Not equal... %u  %u ", a.grid[i][j], map1.grid[i][j]);
  //     // }

  //     ROS_INFO("%u_%u ",a.grid[i][j], map1.grid[i][j]);
  //   }
  //   ROS_INFO("\n");
  // }

  map1.save_map("/home/andrestoga/ros_map_merging/src/map_merging/src/fromMsg.txt");
  // map1.save_map("/home/andrestoga/map_merging/src/map_merging/src/fromMsg.txt");

  // std::vector<transformation> hyp = get_hypothesis(a, b, req.n_hypothesis, req.hough_increment, req.randomized);

  // res.transformations.resize(req.n_hypothesis);

  // for ( unsigned int i = 0 ; i < req.n_hypothesis ; i++ )
  // {
  //   res.transformations[i].ai = hyp[i].ai;
  //   res.transformations[i].deltax = hyp[i].deltax;
  //   res.transformations[i].deltay = hyp[i].deltay;
  //   res.transformations[i].rotation = hyp[i].rotation;

  //   ROS_INFO_STREAM("From the server: " << hyp[i].ai << " " << hyp[i].deltax << " " << hyp[i].deltay << " " <<  hyp[i].rotation);
  // }

  return true;
}