#include "ros/ros.h"
#include "RT_RRT_star.hpp"


int main(int argc, char **argv) {

    RT_RRT<int> test(start,finish);
    // test.setEndPoints(start,finish);
    // test.setCheckPointFunction(*(check));
    // test.setStepLength(200);
    // test.setHalfDimensions(3000.0,2000.0);
    // test.setBiasParameter(100);
    // test.setOrigin(origin);
    // test.setMaxIterations(10000);
    // test.plan();
    std::cout<<"#################################################"<<std::endl;
    while(1)
    {
        Utils::Point<int> now;
        now.x = rand()%100;
        now.y = rand()%100;
        test.add_node_to_tree(now);
        std::cout<<"Generated a point\n";
    }
    // for(int i=0;i<path.size();i++)
    //     cout<<path[i].x<<","<<path[i].y<<endl;
  //ros::init(argc, argv, "RT_RRT_star");

  ros::NodeHandle nh;

}