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

    // Position of the goal
    Utils::Point<T> Xg;

    // Position of all the obtacles
    std::vector<Utils::Point<T> > Xobs;

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

    RT_RRT(T goal_radius, T obstacle_radius, T search_radius, T node_thresh,
      T epsilon_radius, long int k_max);

    /** @brief Simultaneously expand and rewire the tree
     */
    void expand_rewire();

    /** @brief Find all the nodes which are near to @p query
     */
    std::vector<Utils::Point<T> > find_near_nodes(Utils::Point<T> query,
      std::pair<unsigned int, unsigned int> gride_idx);

    /** @brief Add a new point to the tree
     */
    void add_node_to_tree(Utils::Point<T> rand, Utils::Point<T> closest,
      std::vector<Utils::Point<T> > nearest_nodes);

    /** @brief Rewire the tree from the latest node added to the tree
     */
    void rewire_node(std::queue<std::pair<Utils::Point<T>, Utils::Point<T> > > Qr);

    /** @brief Rewire the tree from the agent's position
    */
    void rewire_root(std::queue<std::pair<Utils::Point<T>, Utils::Point<T> > > Qs);

    /** @brief Compute the distance between two nodes
    */
    double dist(Utils::Point<T> first, Utils::Point<T> second);
  };

}

#endif
