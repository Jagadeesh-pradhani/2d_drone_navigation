#include <iostream>
#include <vector>
#include "display.h"
#include "tf.h"
#include "tf2.h"
#include "types.h"
#include "analysis.h"

using std::vector;

/**
 * @brief Utility function to print radar data and computed bogie (enemy) position.
 * @param rb A RangeBearingStamped measurement (radar reading).
 * @param bogie A computed enemy position in the global frame.
 */
void printInfo(RangeBearingStamped rb, Point bogie) {
    std::cout << rb.timestamp << " [r,b]=[" << rb.range 
              << "," << rb.bearing * 180 / M_PI << "]" << std::endl;
    std::cout << "pose [x,y]=[" << bogie.x << "," << bogie.y << "]" << std::endl;
}

int main (void) {
    // Create a Display object with 5 randomly generated enemy (bogie) positions.
    Display display(5);

    // Define the aircraft's pose: positioned at the origin and oriented at 45 degrees.
    Pose aircraft;
    aircraft.position = {0, 0, 0};
    aircraft.orientation = tf::yawToQuaternion(M_PI / 4);

    // Vector to store enemy positions in the global frame.
    std::vector<Point> bogies;

    // Use the radar sensor simulation to scan and obtain bogie positions.
    // The scan function returns measurements in the aircraft's local frame.
    std::vector<RangeBearingStamped> rbVec = display.scan(aircraft);
    for (auto rb : rbVec) {
        // Convert local (radar) coordinates to global coordinates.
        Point bogie = tf2::local2Global(rb, aircraft);
        printInfo(rb, bogie);
        bogies.push_back(bogie);
    }

    // Analysis is performed on the enemy positions.
    Analysis analysis(bogies);

    // Calculate the time required for the aircraft to reach each enemy.
    vector<double> times = analysis.timeToImpact(aircraft);

    // Build a fully connected graph from the enemy positions.
    AdjacencyList graph = analysis.exportGraph();

    // Print the graph: node id and its connections (neighbor id and distance).
    int node = 0;
    for (auto edges : graph) {
        std::cout << "Node " << node << ":" << std::endl;
        for (auto edge : edges) {
            std::cout << "  connects to node " << edge.second << " with distance " << edge.first << std::endl;
        }
        node++;
    }

    return 0;
}
