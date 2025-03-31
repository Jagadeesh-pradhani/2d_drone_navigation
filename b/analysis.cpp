#include "analysis.h"
#include <iostream> // For debugging output (if needed)
#include <thread>
#include <chrono>

using std::vector;

/**
 * @brief Constructor for the Analysis class in folder b.
 *
 * This version of the Analysis class works with a Radar sensor.
 * @param radarPtr A shared pointer to a Radar object.
 */
Analysis::Analysis(std::shared_ptr<Radar> radarPtr) :
    radarPtr_(radarPtr)
{
    // Initialization of the radar pointer
}

/**
 * @brief Computes the scanning speed of the radar sensor.
 *
 * This function queries the radar for a number of samples and measures the total time taken.
 * The scanning speed is the average time taken per sample.
 *
 * @param samples The number of samples to collect.
 * @param scanningSpeed (Output) The computed average time per sample in seconds.
 */
void Analysis::computeScanningSpeed(unsigned int samples, double& scanningSpeed) {
    // Start timer before beginning the sample collection
    auto start = std::chrono::steady_clock::now();
  
    vector<double> results; 
    // Query the radar repeatedly for the specified number of samples
    for (unsigned int i = 0; i < samples; i++) {
        // Call getData() to retrieve the latest radar data
        results = radarPtr_->getData();
    }
    // End timer after collecting all samples
    auto end = std::chrono::steady_clock::now();

    // Compute the total duration in seconds
    std::chrono::duration<double> diff = end - start;
    // Calculate average time per sample
    scanningSpeed = diff.count() / samples;

    return;
}
