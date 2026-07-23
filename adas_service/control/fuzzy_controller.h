#ifndef __FUZZY_CONTROLLER_H__
#define __FUZZY_CONTROLLER_H__

#include <algorithm>

class FuzzyController
{
protected:
    // Structs that describe fuzzy variable of error, derivation of error and output signal control
    struct Error
    {
        float NB, NS, ZE, PS, PB;
    };
    struct Error_dot
    {
        float NE, ZE, PO;
    };
    struct U_dot
    {
        float NB, NM, NS, ZE, PS, PM, PB;
    };

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
#ifndef UT_TEST
    float fuzzyVariableTriangle(float x, float a, float b, float c);
    float fuzzyVariableTrapezoidal(float x, float a, float b, float c, float d);
    float runFuzzy(float error, float error_dot, FuzzyParameters *fuzzy_parameters);
    void limitRange(float *x, float upper_limit, float lower_limit);
#else
    virtual float fuzzyVariableTriangle(float x, float a, float b, float c);
    virtual float fuzzyVariableTrapezoidal(float x, float a, float b, float c, float d);
    virtual float runFuzzy(float error, float error_dot, FuzzyParameters *fuzzy_parameters);
    virtual void limitRange(float *x, float upper_limit, float lower_limit);
#endif

public:
    FuzzyController();
#ifndef UT_TEST
    float pdFuzzyController(float reference_distance, float actual_distance, float sample_time, FuzzyParameters *fuzzy_parameters ); 
    float piFuzzyController(float reference_velocity, float actual_velocity, float sample_time, FuzzyParameters *fuzzy_parameters); 
#else
    virtual float pdFuzzyController(float reference_distance, float actual_distance, float sample_time, FuzzyParameters *fuzzy_parameters ); 
    virtual float piFuzzyController(float reference_velocity, float actual_velocity, float sample_time, FuzzyParameters *fuzzy_parameters);
#endif
};

#endif // FUZZY_CONTROLLER_H