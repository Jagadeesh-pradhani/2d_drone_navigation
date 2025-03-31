#include "tf2.h"
#include "tf.h"
#include <cmath> // for trig operations

namespace tf2 {

    /**
     * @brief Converts local (radar) polar coordinates to global (Cartesian) coordinates.
     *
     * The conversion uses the aircraft's current position and orientation.
     * Global angle is computed as the sum of the aircraft's yaw and the measured bearing.
     * Then, global coordinates are calculated using cosine for the x-component and sine for the y-component.
     *
     * @param rangeBearing Radar measurement (range and bearing).
     * @param aircraft Current pose of the aircraft.
     * @return Point representing the enemy's global position.
     */
    Point local2Global(RangeBearingStamped rangeBearing, Pose aircraft) {
        // Retrieve aircraft's yaw (rotation about the Z axis)
        double aircraft_yaw = tf::quaternionToYaw(aircraft.orientation);
        // Compute the global angle by adding the radar bearing to the aircraft's yaw.
        double global_angle = aircraft_yaw + rangeBearing.bearing;
        
        // Compute global x and y positions
        double x_pos = rangeBearing.range * cos(global_angle) + aircraft.position.x;
        double y_pos = rangeBearing.range * sin(global_angle) + aircraft.position.y;
        
        // Create a Point for the enemy's position in the global frame.
        Point p;
        p.x = x_pos;
        p.y = y_pos;
        p.z = aircraft.position.z; // Assume same altitude as the aircraft
        return p;
    }

    /**
     * @brief Converts a global enemy position to local (radar) polar coordinates relative to the aircraft.
     *
     * The function computes the difference between the enemy's and aircraft's positions,
     * calculates the angle (bearing) using atan2, and adjusts it relative to the aircraft's orientation.
     *
     * @param globalEnemy Enemy's position in the global frame.
     * @param aircraft Current pose of the aircraft.
     * @return RangeBearingStamped containing the range and bearing in the aircraft's frame.
     */
    RangeBearingStamped global2local(Point globalEnemy, Pose aircraft) {
        // Initialize a RangeBearingStamped structure
        RangeBearingStamped rbstamped = {0, 0, 0};

        // Compute differences in x and y coordinates
        double x_pos = globalEnemy.x - aircraft.position.x;
        double y_pos = globalEnemy.y - aircraft.position.y;
        
        // Compute the angle from aircraft to enemy in the global frame
        double theta = normaliseAngle(atan2(y_pos, x_pos));
        
        // Compute the Euclidean distance (range) between aircraft and enemy
        rbstamped.range = sqrt(pow(x_pos, 2) + pow(y_pos, 2));
        
        // Adjust the angle relative to the aircraft's yaw
        double angle = normaliseAngle(theta - tf::quaternionToYaw(aircraft.orientation));
        rbstamped.bearing = angle;
        
        return rbstamped;
    }

    /**
     * @brief Normalises an angle to be within the range -PI to PI.
     *
     * This function adjusts angles outside the desired range.
     *
     * @param theta The input angle in radians.
     * @return Normalised angle in radians between -PI and PI.
     */
    double normaliseAngle(double theta) {
      if (theta > (2 * M_PI))
        theta = theta - (2 * M_PI);
      else if (theta < 0)
        theta = theta + (2 * M_PI);

      if (theta > M_PI) {
          theta = -((2 * M_PI) - theta);
      }

      return theta;
    }

}
