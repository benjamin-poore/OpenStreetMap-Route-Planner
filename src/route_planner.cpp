#include "route_planner.h"
#include <algorithm>
#include <iostream>
#include <vector>

using std::vector;
using std::sort;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

  this->start_node = &m_Model.FindClosestNode(start_x, start_y);
  this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*this->end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  vector<RouteModel::Node*> current_neighbors = current_node->neighbors;
  for(auto neighbor: current_neighbors){;
    neighbor->h_value = CalculateHValue(neighbor);
    neighbor->g_value = current_node->distance(*neighbor) + current_node->g_value;
    neighbor->visited = true;
    neighbor->parent = current_node;
    this->open_list.push_back(neighbor);    
  }
}

bool Compare(RouteModel::Node* a, RouteModel::Node* b){
  double n1 = a->g_value + a->h_value; 
  double n2 = b->g_value + b->h_value; 
  return n1 > n2; 
}
RouteModel::Node* RoutePlanner::NextNode() {
  sort(this->open_list.begin(), this->open_list.end(), Compare);
  RouteModel::Node* lowest_sum_node = this->open_list.back();
  this->open_list.pop_back();
  return lowest_sum_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
  path_found.push_back(*current_node);

  while(path_found.back().parent != nullptr){
    distance += path_found.back().distance(*(path_found.back().parent));
    path_found.push_back(*(path_found.back().parent));
  }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
  std::reverse(path_found.begin(), path_found.end());
  return path_found;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
  current_node = start_node;
  current_node->visited = true;

  AddNeighbors(current_node);
  
  while(this->open_list.size() > 0){ 
    current_node = NextNode();
    double x = current_node->x;
    double y = current_node->y;
    double end_x = this->end_node->x;
    double end_y = this->end_node->y;
    if(x == end_x && y == end_y ){
      m_Model.path = ConstructFinalPath(current_node);
     return;
    }
    AddNeighbors(current_node);
    }
}