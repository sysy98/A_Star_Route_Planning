#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// Expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (auto& neighbor : current_node -> neighbors) {
    neighbor->parent = current_node;
    neighbor->h_value = CalculateHValue(neighbor);
    neighbor->g_value += current_node->g_value + current_node->distance(*neighbor);
    neighbor->visited = true;
    open_list.push_back(neighbor);
  }
}

 
RouteModel::Node *RoutePlanner::NextNode() {
  // Sort the open list and return the next node.
  std::sort(open_list.begin(), open_list.end(), [] (const RouteModel::Node* a, const RouteModel::Node* b) 
  {return a->h_value + a->g_value > b->h_value + b->g_value;});

  auto lowest_node = open_list.back();
  open_list.pop_back();
  return lowest_node;
}


// Return the final path found from A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node != start_node) {
      path_found.emplace_back(*current_node);
      auto parent_node = current_node -> parent;
      distance += current_node -> distance(*parent_node);
      current_node = parent_node;
    }
    path_found.emplace_back(*start_node);
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    open_list.emplace_back(start_node);
    start_node->visited = true;
  
    while(!open_list.empty()) {
      current_node = NextNode();
      
      if(current_node == end_node) {
        m_Model.path = ConstructFinalPath(current_node);
        return;
      }
      AddNeighbors(current_node);
    }
} 

 