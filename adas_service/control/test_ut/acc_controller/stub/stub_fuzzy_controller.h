#ifndef STUB_FUZZY_CONTROLLER_H
#define STUB_FUZZY_CONTROLLER_H

#include <algorithm>

class FuzzyController
{
    public:
        FuzzyController() {};

        // Struct for Fuzzy Controller Parameters
        typedef struct
        {
            float Ke;
            float Ke_dot;
            float Ku;
            float uk_1;
            float ek_1;
            float a;
            float b;
            float c;
        } FuzzyParameters;

        float pdFuzzyController(float reference_distance, float actual_distance, float sample_time, FuzzyParameters *fuzzy_parameters)
        {
            return reference_distance;
        }

        float piFuzzyController(float reference_velocity, float actual_velocity, float sample_time, FuzzyParameters *fuzzy_parameters)
        {
            return reference_velocity;
        }
};

#endif