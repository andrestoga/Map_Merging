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
// #include "map_merging/MergeTwoMaps.h"
// #include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

const unsigned int n_hypothesis = 4;

using namespace mapmerge;

// std::vector< std::vector<unsigned int> > convertToVectorOfVectors(nav_msgs::OccupancyGrid map);

// bool merge_two_maps(map_merging::MergeTwoMaps::Request  &req,
//          map_merging::MergeTwoMaps::Response &res);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_merger_server");
  ros::NodeHandle n;

  // ros::ServiceServer service = n.advertiseService("map_merger", merge_two_maps);
  // ROS_INFO("Ready to merge maps.");
  // ros::spin();

  // char file1[] = "datasets/LongRun6.txt";
  char file1[] = "/home/andrestoga/map_merging/src/map_merging/src/datasets/LongRun6.txt";
  char file2[] = "/home/andrestoga/map_merging/src/map_merging/src/datasets/LongRun7.txt";

  ROS_INFO_STREAM("Loading " << file1 << " and " << file2);
  grid_map a,b;
  a.load_map(900,900,file1);
  b.load_map(900,900,file2);
  
  std::vector<transformation> hyp = get_hypothesis(a,b,n_hypothesis,1,false);
    
  for ( unsigned int i = 0 ; i < n_hypothesis ; i++ )
    ROS_INFO_STREAM(hyp[i].ai << " " << hyp[i].deltax << " " << hyp[i].deltay << " " <<  hyp[i].rotation);

  return 0;
}

// std::vector< std::vector<unsigned int> > convertToVectorOfVectors(nav_msgs::OccupancyGrid map)
// {
//   std::vector< std::vector<unsigned int> > data;

//   data.resize(map.info.width * map.info.height);

//   for (int i = 0, count = 0; i < map.info.width; ++i)
//   {
//     for (int j = 0; j < map.info.height; ++j, ++count)
//     {
//       data[i][j] = (unsigned int)map.data[count];
//     } 
//   }

//   return data;
// }

// bool merge_two_maps(map_merging::MergeTwoMaps::Request  &req,
//          map_merging::MergeTwoMaps::Response &res)
// {

//   std::vector<std::vector<unsigned int> >  map1 = convertToVectorOfVectors(req.map1);

//   std::vector<std::vector<unsigned int> >  map2 = convertToVectorOfVectors(req.map2);

//   // std::vector<transformation> hyp = get_hypothesis(map1, map2, req.n_hypothesis, req.hough_increment, req.randomized);

//   for ( unsigned int i = 0 ; i < n_hypothesis ; i++ )
//   {
//     // res[i].ai = hyp[i].ai;
//     // res[i].deltax = hyp[i].deltax;
//     // res[i].deltay = hyp[i].deltay;
//     // res[i].rotation = hyp[i].rotation;

//     // ROS_INFO_STREAM(hyp[i].ai << " " << hyp[i].deltax << " " << hyp[i].deltay << " " <<  hyp[i].rotation);
//   }

//   return true;
// }