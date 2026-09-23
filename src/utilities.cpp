#include "utilities.h"
#include <Arduino.h>

Utilities::Utilities(int num_filaments, float filament_diameter, float mandrel_diameter, float step_angle, int microstep) {
    
    // fiber pump description
    this->N = num_filaments;        // number of filaments used in pump
    this->df = filament_diameter;   // filament diameter
    this->di = mandrel_diameter;    // inner mandrel diameter

    this->total_motor_steps = (360 / step_angle) * microstep;

}

float Utilities::getHelixAngle() {

    float circ = PI * (this->di + this->df);

    float helix_angle = acos( (this->N * this->df) / circ);

    return helix_angle;
}

float Utilities::getPitch() {
    
    float helix_angle = getHelixAngle();
    
    return PI * (this->di + this->df) / tan(helix_angle);
}

float Utilities::getSteps(float scaling_factor) {

    return scaling_factor * this->total_motor_steps;

}