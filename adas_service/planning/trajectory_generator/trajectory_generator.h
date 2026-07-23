#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H

#ifndef UT_TEST
#include <vector>
#include <string>

#include "frenet_environment/frenet_environment.hpp"

#else
#include "lib4test.h"
#endif

typedef enum {
    DEFAULT_GENERATION,
    FIFTH_ORDER
} TrajectoryGenerationPolicy;

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(FrenetEnvironment* frenet_environment_ptr);

    void execute(TrajectoryGenerationPolicy trajectory_generation_policy);

private:
    struct ObjRect {
        CoorPoint2i top_left;
        CoorPoint2i top_right;
        CoorPoint2i bottom_left;
        CoorPoint2i bottom_right;
        // TODO: must change this into deprecated code if frenet frame is good
        ObjRect(CoorPoint2i tl, CoorPoint2i tr, CoorPoint2i bl, CoorPoint2i br) : 
            top_left(tl), top_right(tr), bottom_left(bl), bottom_right(br) {}; 
    };

    //Class private method(s)
    ObjRect generateFourPointRect(CollisionObject &obj);
    std::vector<CoorPoint2i> checkTwoLineIntersect(CoorPoint2i tail_line1, CoorPoint2i head_line1, 
                                                   CoorPoint2i tail_line2, CoorPoint2i head_line2);

    // Line Utils
    bool areCollinear(CoorPoint2i p1, CoorPoint2i p2, CoorPoint2i p3);
    bool doLineSegmentsIntersect(CoorPoint2i p1, CoorPoint2i p2, CoorPoint2i q1, CoorPoint2i q2);
    CoorPoint2i findIntersection(CoorPoint2i p1, CoorPoint2i p2, CoorPoint2i q1, CoorPoint2i q2);

    void generateCandidatePaths();
    void findCandidatePathsCollisionWithObjects();
    void findCandidatePathsCollisionWithLanes();
    void activateDodging();
    #ifndef UT_TEST
        std::vector<CoorPoint2i> checkPathCollisionSpecificCar(CandidatePath &cpath, 
                                                               ObjRect &specific_car);
    #else
        virtual std::vector<CoorPoint2i> checkPathCollisionSpecificCar(CandidatePath &cpath, 
                                                                       ObjRect &specific_car);
    #endif

    int offset_ = 25;
    std::vector<CandidatePath> current_candidate_paths_;
    FrenetEnvironment* frenet_environment_ptr_ = nullptr;
};


#endif // TRAJECTORYGENERATOR_H
