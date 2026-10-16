// handles_fixture.hpp -- the scripts the handle tests render: knot times on
// the author's clock, positions and ceilings in window units.
// Constraints:
// - Hand-kept. The smoothness hashes in test_kinetic2_handles.cpp cover every
//   case: a changed case changes them, so a case is only ever appended.
#pragma once

namespace handles_fixture {

struct Case {
    const char* name;
    float vmax, amax, jmax;          // window units
    int n;                           // knots
    const unsigned* t_ms;            // authored times
    const float* p;                  // authored positions
};

static const unsigned kSampleT[] = { 0, 500, 700, 900, 1100, 1300, 1800, 1950, 2200, 2600, 3100, 3300, 3400, 3600, 3800, 4300, 4400, 4700, 4800, 5100, 5200, 5700, 6200 };
static const float kSampleP[] = { 0.1, 0.1, 0.9, 0.1, 0.9, 0.1, 0.1, 0.6, 1, 0, 0, 0.7, 0.5, 1, 0.2, 0.2, 0.8, 0.3, 0.9, 0.4, 0.95, 0.1, 0.1 };

static const unsigned kFigureT[] = { 0, 200, 400, 600, 800 };
static const float kFigureP[] = { 0, 1, 0, 1, 0 };

static const unsigned kSampleTrimT[] = { 0, 500, 700, 900, 1100, 1300, 1800, 1950, 2200, 2600, 3100, 3300, 3400, 3600, 3800, 4300, 4400, 4700, 4800, 5100, 5200, 5700, 6200 };
static const float kSampleTrimP[] = { 0.1, 0.1, 0.9, 0.1, 0.9, 0.1, 0.1, 0.6, 1, 0, 0, 0.7, 0.5, 1, 0.2, 0.2, 0.8, 0.3, 0.9, 0.4, 0.95, 0.1, 0.1 };

static const unsigned kFigureTightT[] = { 0, 200, 400, 600, 800, 1000 };
static const float kFigureTightP[] = { 0, 1, 0, 1, 0, 0 };

static const unsigned kRailCapsT[] = { 0, 123, 145, 225, 316, 593, 653, 689, 716, 751 };
static const float kRailCapsP[] = { 0.99, 0.81, 0.36, 0.83, 0.83, 0.833, 0.12, 0.67, 0.98, 0.48 };

static const unsigned kRailBandT[] = { 0, 462, 827, 931, 955, 1099, 1119, 1157 };
static const float kRailBandP[] = { 0.85, 0.77, 0.7, 0.72, 0.96, 0.04, 0.32, 0.324 };

static const unsigned kPchipMonoT[] = { 0, 140, 456, 557, 678, 710, 769, 838 };
static const float kPchipMonoP[] = { 0.74, 0.85, 0.85, 0.78, 0.02, 0.44, 0.99, 0.88 };

static const Case kCases[] = {
    { "sample", 10, 500, 50000, 23, kSampleT, kSampleP },
    { "figure", 6.66666667, 333.333333, 33333.3333, 5, kFigureT, kFigureP },
    { "sample_trim", 8, 500, 50000, 23, kSampleTrimT, kSampleTrimP },
    { "figure_tight", 4, 133.333333, 33333.3333, 6, kFigureTightT, kFigureTightP },
};

// The rail rules and the monotone judge at work, rendered bare only.
static const Case kRenderCases[] = {
    { "rail_caps", 11.8, 475, 25300, 10, kRailCapsT, kRailCapsP },
    { "rail_band", 9.6, 50, 45700, 8, kRailBandT, kRailBandP },
    { "pchip_mono", 7.486, 143, 68760, 8, kPchipMonoT, kPchipMonoP },
};

}  // namespace handles_fixture
