#ifndef FRENET_ENVIRONMENT_HPP
#define FRENET_ENVIRONMENT_HPP

#include <vector>
#include "stub_frenet_environment_entities.hpp"
class FrenetEnvironment
{
public:
    //> Functions
    FrenetEnvironment() = default;
    std::vector<CandidatePath>& getCandidatePath() { return candidate_paths_; }

    std::vector<CandidatePath> candidate_paths_;
};


#endif