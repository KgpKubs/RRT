#ifndef _RT_RRT_STAR_HPP_
#define _RT_RRT_STAR_HPP_

#include "utils.hpp"

namespace rrt {

  template <class T>
  class RT_RRT {
  private:

    // Main Tree
    std::vector<std::pair<Utils::Point<T>, Utils::Point<T> > > tree;

    // Queue at the point of addition of the point to the tree
    std::queue<std::pair<Utils::Point<T>, Utils::Point<T> > > Qr;
    // Queue at the root of the tree
    std::queue<std::pair<Utils::Point<T>, Utils::Point<T> > > Qs;

    // Grid based subset of nodes for grid based search
    std::map<std::pair<unsigned int, unsigned int> , std::vector<Utils::Point<T> > > grid_nodes;

    // Position of the agent
    Utils::Point<T> Xa;

    // Path is planned if a point is added to tree within this radius of goal point
    T goal_radius;

    // All the nodes within this radius of an obstacle are given infinite cost
    T obstacle_radius;

    // We care about obstacles within this radius from the agent
    T search_radius;

    // Minimum distance between any two nodes in the tree
    T node_thresh;

    // Used for finding the subset of nodes near to a given node
    T epsilon_radius;

    // Grid density
    long int k_max;

  public:
    RT_RRT(){};

  };

}

#endif
