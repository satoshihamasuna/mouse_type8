#ifndef SIDE_WALL_STABILITY_H_
#define SIDE_WALL_STABILITY_H_

#include <math.h>

// Resample the side-wall distance every 1 mm of forward encoder travel.
// The 6 mm window approximates the old 20 ms window at 0.32 m/s.
class SideWallStability
{
public:
    static constexpr float kSampleStepMm = 1.0f;
    static constexpr float kDifferenceLimitMm = 5.0f;
    static constexpr float kConfirmDistanceMm = 10.0f;
    static constexpr int kSampleCount = 7;
    static constexpr float kWindowMm = (kSampleCount - 1) * kSampleStepMm;

    void reset()
    {
        count_ = 0;
        newest_ = 0;
        phase_mm_ = 0.0f;
        previous_distance_mm_ = 0.0f;
        difference_mm_ = 0.0f;
        stable_distance_mm_ = 0.0f;
    }

    void update(float distance_mm, float forward_step_mm, bool valid)
    {
        if (!valid || !isfinite(distance_mm) || !isfinite(forward_step_mm)
            || forward_step_mm < 0.0f)
        {
            reset();
            return;
        }

        // A gap larger than the window cannot establish spatial stability.
        if (count_ == 0 || forward_step_mm > kWindowMm)
        {
            reset();
            samples_[0] = distance_mm;
            previous_distance_mm_ = distance_mm;
            count_ = 1;
            return;
        }

        const float missing_mm = count_ < kSampleCount
            ? (kSampleCount - count_) * kSampleStepMm - phase_mm_ : 0.0f;
        if (forward_step_mm > 0.0f)
        {
            float boundary_mm = kSampleStepMm - phase_mm_;
            int crossed = 0;
            while (boundary_mm <= forward_step_mm + kSampleEpsilonMm)
            {
                const float ratio = boundary_mm < forward_step_mm
                    ? boundary_mm / forward_step_mm : 1.0f;
                newest_ = (newest_ + 1) % kSampleCount;
                samples_[newest_] = previous_distance_mm_
                    + ratio * (distance_mm - previous_distance_mm_);
                if (count_ < kSampleCount) ++count_;
                ++crossed;
                boundary_mm += kSampleStepMm;
            }
            phase_mm_ += forward_step_mm - crossed * kSampleStepMm;
            if (phase_mm_ < 0.0f) phase_mm_ = 0.0f;
        }
        previous_distance_mm_ = distance_mm;

        if (count_ < kSampleCount) return;

        float sum = distance_mm;
        const float ratio = phase_mm_ / kSampleStepMm;
        for (int offset = 1; offset < kSampleCount; ++offset)
        {
            const int older = (newest_ + kSampleCount - offset) % kSampleCount;
            const int newer = (older + 1) % kSampleCount;
            sum += samples_[older] + ratio * (samples_[newer] - samples_[older]);
        }
        difference_mm_ = distance_mm - sum / kSampleCount;
        if (fabsf(difference_mm_) < kDifferenceLimitMm)
        {
            // Do not count the part of this step that only filled the window.
            const float confirmed_mm = forward_step_mm > missing_mm
                ? forward_step_mm - missing_mm : 0.0f;
            stable_distance_mm_ += confirmed_mm;
            if (stable_distance_mm_ >= kConfirmDistanceMm - kDistanceEpsilonMm)
                stable_distance_mm_ = kConfirmDistanceMm;
        }
        else
        {
            stable_distance_mm_ = 0.0f;
        }
    }

    bool ready() const { return stable_distance_mm_ >= kConfirmDistanceMm; }
    float difference_mm() const { return difference_mm_; }
    float stable_distance_mm() const { return stable_distance_mm_; }

private:
    // Float roundoff must not add a whole sensor cycle at an exact mm boundary.
    static constexpr float kSampleEpsilonMm = 0.000001f;
    static constexpr float kDistanceEpsilonMm = 0.0001f;
    float samples_[kSampleCount] = {};
    int count_ = 0;
    int newest_ = 0;
    float phase_mm_ = 0.0f;
    float previous_distance_mm_ = 0.0f;
    float difference_mm_ = 0.0f;
    float stable_distance_mm_ = 0.0f;
};

#endif
