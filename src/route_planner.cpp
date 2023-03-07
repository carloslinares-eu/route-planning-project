#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (auto* neighbour_node : current_node->neighbors) {
        neighbour_node->parent = current_node;
        neighbour_node->g_value = current_node->g_value + neighbour_node->distance(*current_node);
        neighbour_node->h_value = RoutePlanner::CalculateHValue(neighbour_node);

        neighbour_node->visited = true;
        open_list.emplace_back(neighbour_node);

    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    SortOpenListBySumOfGH();
    RouteModel::Node* pointer_2_next_node = open_list[0];
    open_list.erase(open_list.begin());
    return pointer_2_next_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    path_found.insert(path_found.begin(), *current_node);

    while (current_node != start_node) {
        path_found.insert(path_found.begin(), *current_node->parent);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }

    distance *= m_Model.MetricScale();
    return path_found;

}

void RoutePlanner::AStarSearch() {
    auto current_node = start_node;
    current_node->visited = true;
    open_list.push_back(current_node);
    while (current_node != end_node){
        AddNeighbors(current_node);
        current_node = NextNode();
    }

    m_Model.path = ConstructFinalPath(current_node);

}

void RoutePlanner::SortOpenListBySumOfGH(){
    bool swapped = true;
    int j = 0;

    while (swapped) {
        swapped = false;
        j++;
        for (int i = 0; i < open_list.size() - j; i++){
            if (open_list[i]->get_h_and_g_sum() > open_list[i+1]->get_h_and_g_sum()) {
                std::swap(open_list[i], open_list[i+1]);
                swapped = true;
            }
        }
    }
}
