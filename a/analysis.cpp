#include "analysis.h"
#include "tf.h"
#include "tf2.h"
#include <cmath>
#include <iostream>

using std::vector;
using std::pair;
using geometry_msgs::Point;

/**
 * @brief Constructor for Analysis class that accepts a vector of goal positions.
 * @param goals Vector of enemy/target positions in the global frame.
 */
Analysis::Analysis(std::vector<Point> goals) :
    goals_(goals)
{
    // Initialization of the goals vector
}


/**
 * @brief Computes the time required for the drone to reach each goal.
 * 
 * The time to impact is computed as the sum of:
 *  1) The time to rotate to face the target (using max angular velocity)
 *  2) The time to translate towards the target (using max linear velocity)
 *
 * @param origin The current pose of the drone.
 * @return A vector of time-to-impact values for each goal.
 */
vector<double> Analysis::timeToImpact(Pose origin) {
    // Constants for maximum velocities provided by the Display class
    // Display::OMEGA_MAX is the maximum angular velocity (rad/s)
    // Display::V_MAX is the maximum linear velocity (m/s)

    vector<double> times;
    
    // Process each goal (enemy position)
    for (int i = 0; i < goals_.size(); i++) {
        // Convert global enemy position into the drone's local frame (relative polar coordinates)
        RangeBearingStamped relative = tf2::global2local(goals_.at(i), origin);

        // Compute rotation time (how long it takes to turn towards the target)
        double rot_time = abs(tf2::normaliseAngle(relative.bearing)) / Display::OMEGA_MAX;
        // Debug prints (can be removed or commented)
        // std::cout << "rotation: " << rot_time << std::endl;
        
        // Compute translation time (how long it takes to cover the distance to the target)
        double trans_time = relative.range / Display::V_MAX;
        // Debug prints (can be removed or commented)
        // std::cout << "translation: " << trans_time << std::endl;
        
        // Total time is the sum of rotation and translation times
        times.push_back(rot_time + trans_time);
    }
    return times;
}


/**
 * @brief Constructs a fully connected graph from the enemy (goal) positions.
 *
 * Each node represents a goal, and each edge weight is the Euclidean distance between two goals.
 * The graph is represented as an adjacency list where each inner vector contains pairs:
 *   - First element: Euclidean distance (weight)
 *   - Second element: Node id of the connected goal.
 *
 * @return An adjacency list representing the fully connected graph.
 */
AdjacencyList Analysis::exportGraph() {
    AdjacencyList graph;

    // Loop over each goal to create a node in the graph
    for (int i = 0; i < goals_.size(); i++) {
        vector<EdgeInfo> connections;
        // Get the current node's global position
        pair<double, double> i_pos = {goals_.at(i).x, goals_.at(i).y};

        // Create connections from this node to every other node (fully connected graph)
        for (int j = 0; j < goals_.size(); j++) {
            if (j != i) { // Avoid self-loop
                EdgeInfo result;
                // Get the other node's position
                pair<double, double> j_pos = {goals_.at(j).x, goals_.at(j).y};
                // Compute the Euclidean distance between nodes
                double distance = sqrt(pow(i_pos.first - j_pos.first, 2) + pow(i_pos.second - j_pos.second, 2));
                result.first = distance; // Edge weight (distance)
                result.second = j;       // Connected node id
                connections.push_back(result);
            }
        }
        // Add the connections for the current node into the graph
        graph.push_back(connections);
    }

    return graph;
}
