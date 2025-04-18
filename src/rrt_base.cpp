#include "rrt_base.hpp"

std::vector<std::pair<Configuration, Configuration>> 
GoalBiasedGreedySteerKNeighborhoodRRTStarBase::get_path_to_goal(){
    return look_for_goal(root);
}

std::vector<std::pair<Configuration, Configuration>> 
GoalBiasedGreedySteerKNeighborhoodRRTStarBase::look_for_goal(Node* node) {
    std::vector<Node*> children = node->getChildren();

    // case: goal is found directly under this node
    for(Node* child : children){
        if(allclose(child->coordinates, goal_coordinates)){
            return {{node->coordinates, child->coordinates}};
        }

        // case: goal not directly under this node, recurse through the subtree
        std::vector<std::pair<Configuration, Configuration>> subpath = look_for_goal(child);
        if(!subpath.empty()){
            subpath.insert(subpath.begin(), {node->coordinates, child->coordinates});
            return subpath;
        }
    }

    // case: goal not found in this branch
    return {};
}

std::vector<std::pair<Configuration, Configuration>> 
GoalBiasedGreedySteerKNeighborhoodRRTStarBase::get_all_edges(){
    std::vector<std::pair<Configuration, Configuration>> edges;

    // if root is null, return empty edge list
    if(!root) return edges;

    // use a queue for traversal and initialize with root node
    std::vector<Node*> queue = {root};

    // loop until all nodes have been visited
    while(!queue.empty()){
        //take last node from queue
        Node* current = queue.back();
        queue.pop_back(); // remove

        // get all children of the current node
        const std::vector<Node*>& children = current->getChildren();
        
        // for each child
        for(Node* child : children){
            // add the edge from teh current to child to the edge list
            edges.push_back({current->coordinates, child->coordinates});
            //add the child to the queue to visit its children later
            queue.push_back(child);
        }
    }

    //return list of all edges found in the tree
    return edges;
}

double GoalBiasedGreedySteerKNeighborhoodRRTStarBase::get_goal_cost(){
    //get current path from root to goal
    std::vector<std::pair<Configuration, Configuration>> path = get_path_to_goal();

    //if goal is not reachable, return infinity
    if(path.empty()){
        return std::numeric_limits<double>::infinity();
    }

    //initialize cost to 0
    double total_cost = 0.0;

    for(const auto& edge : path){
        const Configuration& from = edge.first;
        const Configuration& to = edge.second;
        total_cost += distance(from, to); // use distance() from subclass
    }

    //return cost of the path
    return total_cost;
}
