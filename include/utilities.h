#ifndef Utilities_h
#define Utilities_h

class Utilities {
    public:

        Utilities(int num_filaments, float filament_diameter, float mandrel_diameter, float step_angle, int microstep);

        float getHelixAngle();

        float getPitch();

        float getLinearRes(int lead);
        
        float getRotaryRes(int gear);

        float getSteps(float scaling_factor);

    private:

        int N;
        float df;
        float di;
        float total_motor_steps;

};

#endif