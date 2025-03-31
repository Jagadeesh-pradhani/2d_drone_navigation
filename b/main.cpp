#include <iostream>
#include <thread>
#include <vector>
#include "radar.h"
#include "analysis.h"

/**
 * @brief Main function to demonstrate the radar scanning speed computation.
 *
 * The radar sensor is initialized and started. A separate thread is created to compute the
 * average scanning speed by gathering a set number of samples.
 */
int main (void) {
    // Create a shared pointer to a Radar object.
    std::shared_ptr<Radar> radarPtr(new Radar());

    // Set the radar's maximum detection distance.
    radarPtr->setMaxDistance(80.0);
    // Start the radar sensor (internally creates a thread for continuous data acquisition).
    radarPtr->start();

    // Create a shared pointer to the Analysis object for radar analysis.
    std::shared_ptr<Analysis> analysisPtr(new Analysis(radarPtr));

    double scanningSpeed = 0;    // Variable to hold the computed scanning speed.
    unsigned int samples = 100;  // Number of radar samples to use for the computation.

    // Create a thread to run the computeScanningSpeed function concurrently.
    std::thread timing_thread(&Analysis::computeScanningSpeed, analysisPtr, samples, std::ref(scanningSpeed));

    // Wait for the scanning speed computation thread to complete.
    timing_thread.join();

    // Output the results.
    std::cout << "Queried [" << samples << "] samples from sensor" << std::endl;
    std::cout << "Refresh rate [" << scanningSpeed << "] seconds per sample" << std::endl;

    return 0;
}
