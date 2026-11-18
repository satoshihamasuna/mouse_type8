#ifndef TASK_INC_MOTION_HEADING_H_
#define TASK_INC_MOTION_HEADING_H_

#include <math.h>

// The common marginal passed between passage and turn estimators. Position
// remains in each estimator's own frame; this prior is never a wall observation.
namespace MotionHeading {
struct Prior {
    float heading_rad=0, variance_rad2=0;
    bool valid=false;
    bool usable() const {
        return valid && isfinite(heading_rad) && isfinite(variance_rad2) && variance_rad2>0;
    }
    void predict(float gyro_radps, float dt_s, float walk_rad_sqrt_s) {
        if (!usable() || !isfinite(gyro_radps) || !isfinite(dt_s) || dt_s<=0 || dt_s>=0.020f) {
            *this=Prior{}; return;
        }
        heading_rad+=gyro_radps*dt_s;
        variance_rad2+=walk_rad_sqrt_s*walk_rad_sqrt_s*dt_s;
        if (!usable()) *this=Prior{};
    }
};
}

#endif
