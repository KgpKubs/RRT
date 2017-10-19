//#include "ros/ros.h"
#include "../src/RT_RRT_star.cpp"
using namespace rrt;

int main(int argc, char **argv) {

    srand(time(NULL));
    Utils::Point<int> start,finish;
    start.x=10;
    start.y=10;
    finish.x=1000;
    finish.y=730;

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
        now.x = rand()%1000;
        now.y = rand()%8000;
        test.add_node_to_tree(now);
        std::cout<<"Generated a point\n";
    }
    // for(int i=0;i<path.size();i++)
    //     cout<<path[i].x<<","<<path[i].y<<endl;
  //ros::init(argc, argv, "RT_RRT_star");

  //ros::NodeHandle nh;

}
