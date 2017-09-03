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

    // Grid based subset of nodes for grid based search, value === id of point in main tree
    std::map<std::pair<unsigned int, unsigned int> , std::vector<int > > grid_nodes;

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

    // Useful parameters for setting some search parameters
    unsigned int width, length;

    //half Dimension
    unsigned int halfDimensionX, halfDimensionY;

    unsigned int bucketSize;

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
    std::pair<unsigned int, unsigned int> Grid_Id(Utils::Point<T> gride_idx){
      int x = gride_idx.x + halfDimensionX;
      int y = gride_idx.y + halfDimensionY;

      unsigned int gridX = x/bucketSize;
      unsigned int gridY = y/bucketSize;

      unsigned int maxGridInX = 2*halfDimensionX/bucketSize;
      unsigned int maxGridInY = 2*halfDimensionY/bucketSize;

      if (gridX == maxGridInX)
      {
        gridX --;
      }

      if (gridY == maxGridInY)
      {
        gridY --;
      }

      return std::pair<unsigned int, unsigned int > (gridX, gridY);
    }

    /** @brief Return cost
     */
    std::pair<double,Utils::Point<T> > cost(Utils::Point<T> child)
    {
        if (child==Xa)
          return std::pair<int,Utils::Point<T> > (0,Xa);
        else{
          Utils::Point<T> parent = getParent(child);
          return std::pair<int, Utils::Point<T> >(cost(parent).first + dist(child,parent), parent);
       }
    }

    /** @brief Simultaneously expand and rewire the tree
     */
    void expand_rewire();

    /** @brief Find all the nodes which are near to @p query
     */
    std::vector<Utils::Point<T> > find_near_nodes(Utils::Point<T> query);

    /** @brief Add a new point to the tree
     */
    void add_node_to_tree(Utils::Point<T> rand);

    /** @brief Rewire the tree from the latest node added to the tree
     */
    void rewire_node(std::queue<std::pair<Utils::Point<T>, Utils::Point<T> > > Qr)
    {
      float start = clock();
      float timeLimit;
      while(clock() - start < timeLimit && !Qr.empty()){
        Utils::Point<T> cur = Qr.front();
        std::pair<int,Utils::Point<T>> now = cost(cur);
        double myScore = now.first;
        Utils::Point<T> parent = now.second;
        std::vector<Utils::Point<T> > neighbours= find_near_nodes(cur);

        int myId;
        std::pair<unsigned int, unsigned int> gridId = Grid_Id(cur);
        std::vector<int> ids = grid_nodes[gridId];
        for (int i = 0; i < ids.size(); ++i)
        {
          if (tree[ids[i]].first == cur)
          {
            myId = ids[i];
            break;
          }
        }

        for (int i=0;i<neighbours.size();i++)
        {
          Utils::Point<T> neighbour = neighbours[i];
          if (cost(neighbour).first + dist(neighbour,cur) < myScore)
          {
            myScore = cost(neighbour).first + dist(neighbour,cur);
            tree[myId].second = neighbour;
          }
          if (find(Qr.front(), Qr.back() + 1 , neighbour) != Qr.back() + 1)
          {
            Qr.push(neighbour);
          }
        }
        Qr.pop();
      }
    }

    /** @brief Rewire the tree from the agent's position
    */
    void rewire_root(std::queue<std::pair<Utils::Point<T>, Utils::Point<T> > > Qs)
    {

      if (Qs.empty())
      {
        Qs.push(Xa);
      }

      float start = clock();


      float timeLimit; //to declare by calling file

      while(!Qs.empty() && (clock() - start) > timeLimit){
        Utils::Point<T> cur = Qs.front();
        std::vector<Utils::Point<T> > neighbours= find_near_nodes(cur);
        float myScore = cost(cur).first;
        std::pair<unsigned int, unsigned int> gridID = Grid_Id(cur);
        int myId;
        for (int i = 0; i < grid_nodes[gridID].size(); ++i)
        {
          if (tree[i].first == cur)
          {
            myId = i;
            break;
          }
        }
        for (int i=0;i<neighbours.size();i++)
        {
          Utils::Point<T> neighbour = neighbours[i];
          float nbScore = cost(neighbour).first;
          if (nbScore + dist(neighbour,cur) < myScore)
          {
            myScore = nbScore;
            tree[myId].second = neighbour;
          }
          if (find(Qs.front(), Qs.back() + 1, neighbour) != Qs.back() +1)
          {
            Qs.push(neighbour);
          }
        }
        Qs.pop();
      }
      
    }

    /** @brief Update the radius of neareset nodes prior to calling `find_near_nodes`
    */
    void update_epsilon_radius();

    /** @brief Find the closest node
    */
    Utils::Point<T> closest_node(Utils::Point<T> rand){
      Utils::Point<T> closest;

      std::pair<unsigned int, unsigned int> gridId = Grid_Id(rand);
      unsigned int maxGridInX = 2*halfDimensionX/bucketSize;
      unsigned int maxGridInY = 2*halfDimensionY/bucketSize;
      std::vector<std::pair<unsigned int, unsigned int> > adjacentGrid;
      adjacentGrid.push_back(gridId);
      int x[] = {-1,-1,-1,0,0,1,1,1};
      int y[] = {-1,0,1,-1,1,-1,0,1};
      for (int i = 0; i < 8; ++i)
      {
        unsigned int gridX = gridId.first + x[i];
        unsigned int gridY = gridId.second + y[i];
        if (gridX >= 0 && gridX < maxGridInX && gridY >=0 && gridY < maxGridInY)
        {
          adjacentGrid.push_back(std::pair<unsigned int, unsigned int > (gridX,gridY));
        }
      }
      float minDist = 999999;
      for (int i = 0; i < adjacentGrid.size(); ++i)
      {
        std::vector<int> ids = grid_nodes[adjacentGrid[i]];
        for (int j = 0; j < ids.size(); ++j)
        {
          float distance = dist(rand,tree[ids[j]].first);
          if (distance < minDist)
          {
            minDist = distance;
            closest = tree[ids[j]].first;
          }
        }
      }

      return closest;
    }

    /** @brief Verify if the line crosses any obstacle
    */
    bool line_path_obs(Utils::Point<T> p1, Utils::Point<T> p2);

    
    /**@brief Return Parent of Node
    */
    Utils::Point<T> getParent(Utils::Point<T> cur)
    {
      std::pair<unsigned int, unsigned int> gridId= Grid_Id(cur);
      std::vector<int> ids = grid_nodes[gridId];
      // iterate over cur point bucket
      for(int i=0; i<ids.size() ;i++)
      {
        if(tree[ids[i]].first==cur){
          return tree[ids[i]].second;
        }
      }
    }


  };

}

#endif
