#include "fuzzy_controller.h"

FuzzyController::FuzzyController() {}

float FuzzyController::fuzzyVariableTriangle(float x, float a, float b, float c)
{
    if (x < a || x > c)
        return 0;
    if (x <= b)
        return (x - a) / (b - a);
    return (c - x) / (c - b);
}

float FuzzyController::fuzzyVariableTrapezoidal(float x, float a, float b, float c, float d)
{
    if (x < a || x > d)
        return 0;
    if (x <= b)
        return (a == b) ? 1 : (x - a) / (b - a);
    if (x <= c)
        return 1;
    return (c == d) ? 1 : (d - x) / (d - c);
}

float FuzzyController::runFuzzy(float error, float error_dot, FuzzyParameters* fuzzy_parameters)
{
    float out;
    float r[15]; // 15 value beta <=> 15 rule
    Error e;
    Error_dot e_dot;
    U_dot u_dot;

    e.NB = fuzzyVariableTrapezoidal(error, -2, -1, -0.22f, -0.17f);
    e.NS = fuzzyVariableTriangle(error, -0.22f, -0.11f, 0.001f);
    e.ZE = fuzzyVariableTriangle(error, -0.11f, 0.0f, 0.11f);
    e.PS = fuzzyVariableTriangle(error, 0.001f, 0.11f, 0.22f);
    e.PB = fuzzyVariableTrapezoidal(error, 0.17f, 0.22f, 1.0f, 2.0f);

    e_dot.NE = fuzzyVariableTrapezoidal(error_dot, -2.0f, -1.0f, -0.22f, -0.17f);
    e_dot.ZE = fuzzyVariableTriangle(error_dot, -0.4f, 0.0f, 0.4f);
    e_dot.PO = fuzzyVariableTrapezoidal(error_dot, 0.17f, 0.22f, 1.0f, 2.0f);

    // Calculate beta (0->1) based on MAX-MIN, "and" => MIN
    // e - e_dot - u_dot
    r[0] = std::min(e.NB, e_dot.NE);  // NB-NE-  NB
    r[1] = std::min(e.NB, e_dot.ZE);  // NB-ZE-  NM
    r[2] = std::min(e.NB, e_dot.PO);  // NB-PO-  NS
    r[3] = std::min(e.NS, e_dot.NE);  // NS-NE-  NM
    r[4] = std::min(e.NS, e_dot.ZE);  // NS-ZE-  NS
    r[5] = std::min(e.NS, e_dot.PO);  // NS-PO-  ZE
    r[6] = std::min(e.ZE, e_dot.NE);  // ZE-NE-  NS
    r[7] = std::min(e.ZE, e_dot.ZE);  // ZE-ZE-  ZE
    r[8] = std::min(e.ZE, e_dot.PO);  // ZE-PO-  PS
    r[9] = std::min(e.PS, e_dot.NE);  // PS-NE-  ZE
    r[10] = std::min(e.PS, e_dot.ZE); // PS-ZE- PS
    r[11] = std::min(e.PS, e_dot.PO); // PS-PO- PM
    r[12] = std::min(e.PB, e_dot.NE); // PB-NE- PS
    r[13] = std::min(e.PB, e_dot.ZE); // PB-ZE- PM
    r[14] = std::min(e.PB, e_dot.PO); // PB-PO- PB

    // Sugeno fuzzy system
    u_dot.NB = r[0];
    u_dot.NM = std::max(r[1], r[3]);
    u_dot.NS = std::max({ r[2], r[4], r[6] });
    u_dot.ZE = std::max({ r[5], r[7], r[9] });
    u_dot.PS = std::max({ r[8], r[10], r[12] });
    u_dot.PM = std::max(r[11], r[13]);
    u_dot.PB = r[14];

    // Weighted average defuzzification method
    float sum_beta = u_dot.NB + u_dot.NM + u_dot.NS + u_dot.ZE + u_dot.PS + u_dot.PM + u_dot.PB;
    float sum_beta_y = - fuzzy_parameters->c * u_dot.NB + -fuzzy_parameters->b * u_dot.NM +
                       - fuzzy_parameters->a * u_dot.NS + 0.0f * u_dot.ZE + fuzzy_parameters->a * u_dot.PS 
                       + fuzzy_parameters->b * u_dot.PM + fuzzy_parameters->c * u_dot.PB;

    out = sum_beta_y / sum_beta;
    return out;
}

void FuzzyController::limitRange(float* x, float upper_limit, float lower_limit)
{
    *x = std::min(std::max(*x, lower_limit), upper_limit);
}

float FuzzyController::pdFuzzyController(float reference_distance, float actual_distance, float sample_time,
                                         FuzzyParameters* fuzzy_parameters)
{
    float ek, uk;
    float P_part, D_part;
    ek = - reference_distance + actual_distance;

    P_part = fuzzy_parameters->Ke * ek;
    limitRange(&P_part, 1.0, -1.0);

    D_part = fuzzy_parameters->Ke_dot * (ek - fuzzy_parameters->ek_1) / sample_time;
    limitRange(&D_part, 1.0, -1.0);

    uk = runFuzzy(P_part, D_part, fuzzy_parameters);
    limitRange(&uk, 1.0, -1.0);
    fuzzy_parameters->uk_1 = uk;
    fuzzy_parameters->ek_1 = ek;
    uk = fuzzy_parameters->Ku * uk;
    return uk;
}

float FuzzyController::piFuzzyController(float reference_velocity, float actual_velocity, float sample_time,
                                         FuzzyParameters* fuzzy_parameters)
{
    float ek, uk, uk_dot;
    float P_part, D_part;
    ek = reference_velocity - actual_velocity;

    P_part = fuzzy_parameters->Ke * ek;
    limitRange(&P_part, 1.0, -1.0);

    D_part = fuzzy_parameters->Ke_dot * (ek - fuzzy_parameters->ek_1) / sample_time;
    limitRange(&D_part, 1.0, -1.0);

    uk_dot = runFuzzy(P_part, D_part, fuzzy_parameters);
    uk = fuzzy_parameters->uk_1 + uk_dot * sample_time;
    limitRange(&uk, 1.0, -1.0);

    fuzzy_parameters->uk_1 = uk;
    fuzzy_parameters->ek_1 = ek;
    uk = fuzzy_parameters->Ku * uk;
    return uk;
}
