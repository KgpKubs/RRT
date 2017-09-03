#include "RT_RRT_star.hpp"

namespace rrt {

  template <class T>
  RT_RRT<T>::RT_RRT() {
    // TODO : set these parameters
    // TODO : Set all parameters in one scale
    goal_radius = 20.0;
    obstacle_radius = 20.0;
    search_radius = 20.0;
    node_thresh = 20.0;
    epsilon_radius = 20.0;
    k_max = 20.0;

    width = 6;
    length = 9;
  }

  template <class T>
  RT_RRT<T>::RT_RRT(T goal_radius, T obstacle_radius, T search_radius,
    T node_thresh, T epsilon_radius, long int k_max) {

    this->goal_radius = goal_radius;
    this->obstacle_radius = obstacle_radius;
    this->search_radius = search_radius;
    this->node_thresh = node_thresh;
    this->epsilon_radius = epsilon_radius;
    this->k_max = k_max;
    width = 6;
    length = 9;
  }

  template <class T>
  void RT_RRT<T>::update_epsilon_radius() {
    size_t total_nodes = this->tree.size();

  }

  template <class T>
  std::vector<Utils::Point<T> > RT_RRT<T>::find_near_nodes(Utils::Point<T> query) {

    std::pair<unsigned int, unsigned int> grid_idx = this->Grid_Id(query);
    std::vector<Utils::Point<T> > nearest_points;
    for (typename std::vector<Utils::Point<T> >::iterator
      itr = grid_nodes[grid_idx].begin(); itr != grid_nodes[grid_idx].end();
      ++itr) {
        this->update_epsilon_radius();
        if (this->dist(query, *itr) < this->epsilon_radius) {
          nearest_points.push_back(*itr);
        }
      }
    return nearest_points;
  }

  

  template <class T>
  void RT_RRT<T>::add_node_to_tree(Utils::Point<T> rand) {

    Utils::Point<T> closest = closest_node(rand);
    std::vector<Utils::Point<T> > x_near = find_near_nodes(rand);

    Utils::Point<T> x_min = closest;
    int c_min = cost(closest) + dist(closest, rand);
    for (typename std::vector<Utils::Point<T> >::iterator itr = x_near.begin();
      itr != x_near.end(); ++itr) {

      int c_new = cost(*itr) + dist(*itr, rand);

    }

  }

}
