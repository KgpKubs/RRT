#ifndef _RT_RRT_STAR_HPP_
#define _RT_RRT_STAR_HPP_

#include "utils.hpp"

namespace rrt {

  template <class T>
  class RT_RRT {
  private:

    // Main Tree
    std::vector<std::pair<Utils::Point<T>, Utils::Point<T> > > tree;

    // Queue at the Point of addition of the Point to the tree
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

    // Path is planned if a Point is added to tree within this radius of goal Point
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

    // Useful parameters for setting some search parameters
    unsigned int width, length;

  public:
    RT_RRT();

    RT_RRT(T goal_radius, T obstacle_radius, T search_radius, T node_thresh,
      T epsilon_radius, long int k_max);

    /** @brief Compute the distance between two nodes
    */

    double dist(Utils::Point<T> first, Utils::Point<T> second)
    {
      return sqrt(pow(first.x-second.x,2)+pow(first.y-second.y,2));
    }

    /** @brief Return Grid Id
     */
    std::pair<unsigned int, unsigned int> Grid_Id(Utils::Point<T> gride_idx);

    /** @brief Return cost
     */
    std::pair<int,Utils::Point<T> > cost(Utils::Point<T> child, int count=0)
    {
              if (child==Xa)
                return std::pair<int,Utils::Point<T> > (0,Xa);
              for(int j=0;j<tree.size();j++)
              {
                if(tree[j].first==child)
                {
                  std::pair<int,Utils::Point<T> > here = cost(tree[j].second,1);
                  if (count)
                    return std::pair<int,Utils::Point<T> >  (dist(child,tree[j].second)+here.first.first,here.second);
                  else
                    return std::pair<int,Utils::Point<T> >  (dist(child,tree[j].second)+here.first.first,tree[j].second);
                }
              }
    }

    /** @brief Simultaneously expand and rewire the tree
     */
    void expand_rewire();

    /** @brief Find all the nodes which are near to @p query
     */
    std::vector<Utils::Point<T> > find_near_nodes(Utils::Point<T> query);

    /** @brief Add a new Point to the tree
     */
    void add_node_to_tree(Utils::Point<T> rand);

    /** @brief Rewire the tree from the latest node added to the tree
     */
    void rewire_node(std::queue<std::pair<Utils::Point<T>, Utils::Point<T> > > &Q)
    {
      Utils::Point<T> me = Q.pop();
      std::pair<int,Utils::Point<T>> now = getCost(me);
      int cost = now.first;
      Utils::Point<T> parent = now.second;
      std::vector<Utils::Point<T> > neighbours= find_near_nodes(Q);
      for(size_t i = 0; i < neighbours.size(); i++)
          Q.push(neighbours[i]);
      for (int i=0;i<neighbours.size();i++)
      {
        Utils::Point<T> neighbour = neighbours[i];
        if (cost > cost - dist(me,parent) + dist(parent,neighbour) + dist(neighbour,me))
        {
              for(int j=0;j<tree.size();j++)
              {
                if(tree[j].first==me)
                  tree[j].second = neighbour[i];
              }
        }
      }
    }

    /** @brief Rewire the tree from the agent's position
    */
    void rewire_root(std::queue<std::pair<Utils::Point<T>, Utils::Point<T> > > Qs)
    {
      int j;
      std::vector<Utils::Point<T> > neighbours= find_near_nodes(Xa);
      for(size_t i = 0; i < neighbours.size(); i++)
          Qs.push(neighbours[i]);
      rewire_node(Qs);
    }

    /** @brief Update the radius of neareset nodes prior to calling `find_near_nodes`
    */
    void update_epsilon_radius();

    /** @brief Find the closest node
    */
    Utils::Point<T> closest_node(Utils::Point<T> rand);

    /** @brief Verify if the line crosses any obstacle
    */
    bool line_path_obs(Utils::Point<T> p1, Utils::Point<T> p2);
  };

}

#endif
