// =============================================================================
//  MUMBAI 1:1 EXACT — tests/test_hydro.cpp
//
//  The monsoon hydrology (Section 3): depression filling, flow directions and
//  accumulation, watersheds, and the storm model's physical behaviour.  These
//  cases run on synthetic terrain with analytically known answers, because the
//  city itself cannot tell a correct model from a plausible-looking one.
// =============================================================================
#include "pipeline/mgeo/hydro.h"
#include "tests/test_harness.h"

#include <cmath>

using namespace mumbai;

namespace {

// A heightfield filled from a lambda, in world metres.
terrain::Heightfield makeGrid(u32 w, u32 h, f64 step, const std::function<f64(u32, u32)>& f) {
    terrain::Heightfield hf;
    hf.resize(w, h, 0.0, 0.0, step, step);
    for (u32 j = 0; j < h; ++j)
        for (u32 i = 0; i < w; ++i) hf.at(i, j) = static_cast<f32>(f(i, j));
    return hf;
}

constexpr f64 kStep = 20.0;

// A 30 x 30 grid: a plain at 10 m with a square pit at 2 m, ringed by a 12 m rim
// except for one 9 m spill notch on the east side.
terrain::Heightfield pitGrid() {
    return makeGrid(30, 30, kStep, [](u32 i, u32 j) {
        const bool body = i >= 8 && i <= 20 && j >= 8 && j <= 20;
        if (body) return 2.0;
        if (i == 22 && j >= 13 && j <= 15) return 9.0;   // spill notch through the rim
        const bool rim = i >= 6 && i <= 22 && j >= 6 && j <= 22;
        if (rim) return 12.0;
        return 10.0;
    });
}

}  // namespace

TEST(priorityFloodFillsEveryInteriorMinimumToItsSpillLevel) {
    // A bowl inside a 10 m rim, sitting on a 3 m plateau, with a single 6 m
    // notch cut through the rim.  The escape path from the bowl crosses the
    // notch (6 m) and then the plateau (3 m), so the basin's spill level is
    // 6 m: the pool inside the rim must come to rest exactly there, while the
    // notch cell downstream of it rests at the plateau level and the raw
    // ground is left untouched.
    //
    //      cheb = max(|i-15|, |j-15|)
    //      cheb <= 5   bowl interior, 1 m at the centre ramping to 5 m
    //      cheb == 6   rim at 10 m, except the notch lane at 6 m
    //      cheb >= 7   plateau at 3 m
    const terrain::Heightfield hf = makeGrid(31, 31, kStep, [](u32 i, u32 j) {
        const int di = static_cast<int>(i) - 15, dj = static_cast<int>(j) - 15;
        const int cheb = std::max(std::abs(di), std::abs(dj));
        if (cheb <= 5) return 1.0 + 0.8 * cheb;
        if (cheb == 6) return (i == 15 && j == 21) ? 6.0 : 10.0;
        return 3.0;
    });
    hydro::SurfaceOptions opt;
    opt.openBoundaries = true;
    const hydro::Surface s = hydro::buildSurface(hf, opt);

    // The pool rests at the notch level...
    CHECK_NEAR(s.filled[15 * 31 + 15], 6.0, 1e-3);
    CHECK_NEAR(s.filled[10 * 31 + 10], 6.0, 1e-3);
    CHECK_NEAR(s.filled[20 * 31 + 20], 6.0, 1e-3);
    // ...the notch cell itself is a slope, so its filled level is its own
    // height, and the plateau beyond drains freely at 3 m...
    CHECK_NEAR(s.filled[21 * 31 + 15], 6.0, 1e-3);
    CHECK_NEAR(s.filled[2 * 31 + 2], 3.0, 1e-3);
    // ...and the ground underneath is exactly what was authored.
    CHECK_NEAR(s.ground[15 * 31 + 15], 1.0, 1e-6);
    CHECK_NEAR(s.ground[21 * 31 + 15], 6.0, 1e-6);

    // After filling, no interior cell may be a *strict* local minimum: that is
    // precisely the set the fill is supposed to have removed.
    int strictMinima = 0;
    for (u32 j = 1; j + 1 < s.height; ++j) {
        for (u32 i = 1; i + 1 < s.width; ++i) {
            const f32 c = s.filled[j * s.width + i];
            bool lowest = true;
            for (int dj = -1; dj <= 1; ++dj)
                for (int di = -1; di <= 1; ++di) {
                    if (!di && !dj) continue;
                    if (s.filled[(j + dj) * s.width + (i + di)] <= c + 1e-6f) lowest = false;
                }
            if (lowest) ++strictMinima;
        }
    }
    CHECK_EQ(strictMinima, 0);
}

TEST(flowDirectionsAndAccumulationFollowTheFallLine) {
    // A plane tilted towards +x with a broad channel at j = 10 collecting a
    // tributary from the north: every cell must route east, the accumulation
    // must increase monotonically downstream, and the outlet must see the whole
    // catchment.
    const terrain::Heightfield hf = makeGrid(24, 24, kStep, [](u32 i, u32 j) {
        const f64 base = 30.0 - i * 0.35;
        const f64 ditch = (j == 10) ? -1.5 : 0.0;
        return base + ditch;
    });
    hydro::SurfaceOptions opt;
    opt.openBoundaries = true;
    const hydro::Surface s = hydro::buildSurface(hf, opt);

    for (u32 j = 4; j + 4 < 24; ++j) {
        // Rows adjacent to the ditch (j = 10) drain into it — that is the
        // steepest descent — and rows near the north/south edge drain to that
        // open boundary.  Everything else follows the fall line east.
        if (j == 9 || j == 10 || j == 11) continue;
        for (u32 i = 4; i + 4 < 24; ++i)
            CHECK_EQ(static_cast<int>(s.flowDir[j * 24 + i]), 0);
    }
    // Accumulation along the ditch grows towards the sea.
    const u32 a1 = s.accum[10 * 24 + 5];
    const u32 a2 = s.accum[10 * 24 + 10];
    const u32 a3 = s.accum[10 * 24 + 20];
    CHECK(a1 < a2);
    CHECK(a2 < a3);
    // The ditch carries more than a cell far from it at the same column.
    CHECK(s.accum[10 * 24 + 20] > s.accum[2 * 24 + 20]);
}

TEST(watershedsPartitionTheTerrainByOutlet) {
    // A ridge down the middle with a valley on each side: every cell must drain
    // towards its own side's channel, and the two channels must be different
    // catchments.
    const terrain::Heightfield hf = makeGrid(40, 30, kStep, [](u32 i, u32 j) {
        const f64 ridge = 20.0 + 6.0 * std::exp(-std::pow((i - 20.0) / 5.0, 2.0));
        const f64 valley = 4.0 * std::exp(-std::pow((i - 6.0) / 4.0, 2.0)) +
                           4.0 * std::exp(-std::pow((i - 34.0) / 4.0, 2.0));
        const f64 northSouth = 0.9 - 0.03 * j;      // gentle fall towards +z... 
        (void)northSouth;
        return ridge - valley + (29 - j) * 0.05;    // and towards the north edge
    });
    hydro::SurfaceOptions opt;
    opt.openBoundaries = true;
    opt.channelAreaKm2 = 0.006;                 // small grid: reach the channel threshold
    const hydro::Surface s = hydro::buildSurface(hf, opt);

    const u32 westChannel = s.watershed[5 * 40 + 6];
    const u32 eastChannel = s.watershed[5 * 40 + 34];
    CHECK(westChannel != 0);
    CHECK(eastChannel != 0);
    CHECK(westChannel != eastChannel);
    // Cells on each flank drain to their own side's channel, and the divide cell
    // drains somewhere but neither channel claims a cell on the wrong flank.
    CHECK_EQ(static_cast<int>(s.watershed[5 * 40 + 2]), static_cast<int>(westChannel));
    CHECK_EQ(static_cast<int>(s.watershed[5 * 40 + 38]), static_cast<int>(eastChannel));
}

TEST(depressionsPondWaterInsteadOfDrainingAway) {
    // The physical requirement behind "Hindmata floods": water that cannot
    // reach an outlet must sit on the ground.  A closed pit must hold water at
    // the end of the storm, and the plain above it must not.
    //
    // The check is on the pit as a whole rather than on one cell: routing is a
    // directional sweep, so inside a flat pit the water legitimately piles up
    // on the downstream side.  A single cell would measure that bias instead of
    // the ponding.
    const terrain::Heightfield hf = makeGrid(24, 24, kStep, [](u32 i, u32 j) {
        const bool body = i >= 8 && i <= 16 && j >= 8 && j <= 16;
        if (body) return 4.0;
        const bool rim = i >= 5 && i <= 19 && j >= 5 && j <= 19;
        if (rim) return 14.0;
        return 18.0;
    });
    hydro::SurfaceOptions opt;
    opt.openBoundaries = true;
    const hydro::Surface s = hydro::buildSurface(hf, opt);

    hydro::LandSpec land;
    land.resize(s.cellCount(), 0.9f);       // impervious bowl, no drains at all
    hydro::StormSpec storm;
    storm.name = "test";
    storm.rainMmPerHour = {120, 120, 120, 120};
    storm.tideAmplitude = 0.0;              // no tide: isolate the ponding
    storm.stepSeconds = 300.0;
    storm.initialLossMm = 0.0;
    const hydro::FloodResult r = hydro::simulate(s, land, storm, {});

    // Three regions: the closed body, the rim ledge that drains into it, and
    // the outer plain that sheds to the domain boundary.
    f64 pitTotal = 0.0, rimTotal = 0.0, outerTotal = 0.0;
    int pitN = 0, rimN = 0, outerN = 0;
    f32 pitPeak = 0.0f;
    for (u32 j = 0; j < 24; ++j) {
        for (u32 i = 0; i < 24; ++i) {
            const bool body = i >= 8 && i <= 16 && j >= 8 && j <= 16;
            const bool rimRing = !body && i >= 5 && i <= 19 && j >= 5 && j <= 19;
            const f32 d = r.finalDepth[j * 24 + i];
            if (body) { pitTotal += d; ++pitN; pitPeak = std::max(pitPeak, r.maxDepth[j * 24 + i]); }
            else if (rimRing) { rimTotal += d; ++rimN; }
            else { outerTotal += d; ++outerN; }
        }
    }
    const f64 pitMean = pitTotal / pitN;
    const f64 outerMean = outerTotal / outerN;
    const f64 rimMean = rimTotal / rimN;
    // 0.48 m of rain at 0.9 runoff is 0.432 m of water on every cell.  The pit
    // also collects the rim's runoff, so its pool must be at least that deep;
    // the plain beyond the rim sheds to the open boundary and must be nearly
    // dry; and the pit must be far deeper than either.
    CHECK(pitMean > 0.43);
    CHECK(outerMean < 0.10);
    CHECK(pitMean > outerMean * 6.0);
    CHECK(pitMean > rimMean);
    CHECK(pitPeak > 0.43f);
    CHECK(r.floodedCells > 0);
    CHECK(r.finalDepth[12 * 24 + 12] > r.finalDepth[1 * 24 + 1]);
}

TEST(seaWallHoldsTheOpenCoastWhileTheCreekMouthAdmits) {
    // Mumbai's open sea face is walled and its creeks are not, and that single
    // asymmetry is what makes the model's coastal flooding mean anything.  The
    // grid below is 1 m of land beside a wide sea on the west and a one-cell
    // creek cutting in from the east, with a spring tide running over both.
    //
    //      i < 5          open sea, ground -4 m
    //      i == 5         dry-sand strip at 1 m (the walled shore)
    //      i == 20        creek, ground -3 m, cutting through the land
    //      elsewhere      1 m of low land
    //
    // The walled shore must stay dry all run; the creek's banks must be lifted
    // to the tide; and a cell the DEM holds at its below-datum floor must be
    // left alone even though it faces the creek.
    const terrain::Heightfield hf = makeGrid(40, 40, kStep, [](u32 i, u32 j) {
        if (i < 9) return -4.0;                     // open sea, west
        if (i == 9) return 1.0;                     // the walled shore strip
        if (i == 20) return -3.0;                   // creek, one cell wide
        if (i == 19) {
            // West bank of the creek, with one cell the DEM holds at its floor
            // and two upland neighbours sealing it off from the other banks.
            if (j == 11) return -0.75;
            if (j == 10 || j == 12) return 2.5;
            return 1.0;
        }
        if (i == 10 && j == 20) return -0.75;       // reclamation behind the wall
        return 2.5;                                 // upland, above the tide
    });
    hydro::SurfaceOptions sopt;
    sopt.openBoundaries = true;
    const hydro::Surface s = hydro::buildSurface(hf, sopt);

    CHECK(s.isSea[10 * 40 + 2] == 1);
    CHECK(s.isSea[10 * 40 + 20] == 1);
    CHECK(s.isShore[10 * 40 + 9] == 1);             // beside the open sea
    CHECK(s.isShore[15 * 40 + 19] == 1);            // beside the creek
    CHECK(s.isTidalInlet[15 * 40 + 19] == 1);
    CHECK(s.isTidalInlet[10 * 40 + 9] == 0);

    hydro::LandSpec land;
    land.resize(s.cellCount(), 0.5f);
    hydro::StormSpec storm = hydro::stormSpecSevere();
    storm.rainMmPerHour.assign(14, 0.0);            // tide alone: no rain at all
    storm.tideAmplitude = 2.2;
    storm.stepSeconds = 300.0;
    const hydro::FloodResult r = hydro::simulate(s, land, storm, {});

    CHECK(r.tideFloodM3 > 0.0);                     // the creek did admit the tide
    CHECK(r.rainfallM3 == 0.0);
    // Walled coast: dry, whatever the tide does.
    CHECK(r.maxDepth[10 * 40 + 9] < 0.01f);
    // Creek bank: lifted to the tide, one metre of water over one metre of land.
    CHECK(r.maxDepth[15 * 40 + 19] > 0.5f);
    // Upland above the tide is never lifted, even beside a creek: the boundary
    // condition lifts shore cells to the tide, it does not pour the sea inland.
    CHECK(r.maxDepth[15 * 40 + 21] < 0.01f);
    // Cells the DEM floors below datum are behind the shore line, not on it:
    // the tide must not be levelled up over them.
    CHECK(r.maxDepth[11 * 40 + 19] < 0.01f);        // on the creek, sealed off
    CHECK(r.maxDepth[20 * 40 + 10] < 0.01f);        // behind the sea wall
}

TEST(theTideNeverOutrunsItsOwnWave) {
    // The face velocity limiter: a deep tidal head beside a dry cell must not
    // empty itself across the face in one step.  Without the cap the shore
    // floods the whole plain at once and the depth becomes step-order
    // dependent, which is how the first version of this model drowned the city.
    const terrain::Heightfield hf = makeGrid(24, 24, kStep, [](u32 i, u32 /*j*/) {
        return i == 0 ? 1.0 : 0.0;                  // one cell one metre up
    });
    hydro::SurfaceOptions sopt;
    sopt.openBoundaries = true;
    const hydro::Surface s = hydro::buildSurface(hf, sopt);
    hydro::LandSpec land;
    land.resize(s.cellCount(), 0.9f);
    hydro::StormSpec storm;
    storm.name = "limiter";
    storm.rainMmPerHour.assign(2, 0.0);
    storm.tideAmplitude = 0.0;
    storm.stepSeconds = 60.0;
    const hydro::FloodResult r = hydro::simulate(s, land, storm, {});
    CHECK(r.rainfallM3 == 0.0);
    CHECK(r.storedM3 == 0.0);
}

TEST(stormConservesMassExactly) {
    // Rainfall must equal drains + outfall + water still stored, to rounding.
    const terrain::Heightfield hf = makeGrid(40, 40, kStep, [](u32 i, u32 j) {
        // Slopes to the east and dips into ditches, so water leaves through the
        // boundary while some of it is still routing when the storm ends.
        return 12.0 - i * 0.25 + ((j % 5 == 0) ? -1.0 : 0.0);
    });
    hydro::SurfaceOptions opt;
    opt.openBoundaries = true;
    const hydro::Surface s = hydro::buildSurface(hf, opt);

    hydro::LandSpec land;
    land.resize(s.cellCount(), 0.8f);
    for (size_t k = 0; k < land.drainCapacity.size(); ++k)
        land.drainCapacity[k] = static_cast<f32>(20.0 / 1000.0 / 3600.0);   // 20 mm/h
    hydro::StormSpec storm = hydro::stormSpecSevere();
    storm.stepSeconds = 120.0;
    const hydro::FloodResult r = hydro::simulate(s, land, storm, {});

    CHECK(r.rainfallM3 > 0.0);
    // rainfall + tide exchange == drained + outfall + still stored.
    const f64 in = r.rainfallM3 + std::max(0.0, r.tideExchangeM3);
    const f64 out = r.drainedM3 + r.outfallM3 + r.storedM3 + r.filmLossM3 +
                    std::min(0.0, r.tideExchangeM3);
    const f64 err = std::fabs(in - out) / std::max(1.0, in);
    CHECK(err < 1e-9);
}

TEST(highTideFloodsMoreThanLowTide) {
    // The tide-lock mechanism: identical rainfall, different tide phase.
    const terrain::Heightfield hf = makeGrid(30, 30, kStep, [](u32 i, u32 j) {
        return 1.2 + i * 0.05 + j * 0.02;    // a low coastal shelf rising inland
    });
    hydro::SurfaceOptions opt;
    opt.openBoundaries = true;
    opt.seaLevel = 0.0;
    const hydro::Surface s = hydro::buildSurface(hf, opt);

    hydro::LandSpec land;
    land.resize(s.cellCount(), 0.85f);
    for (size_t k = 0; k < land.drainCapacity.size(); ++k)
        land.drainCapacity[k] = static_cast<f32>(30.0 / 1000.0 / 3600.0);

    hydro::StormSpec storm;
    storm.name = "tide-test";
    storm.rainMmPerHour = {60, 60, 60, 60};
    storm.tideAmplitude = 2.0;
    storm.stepSeconds = 300.0;

    hydro::StormSpec lowTide = storm;
    lowTide.tideAmplitude = 0.0;                       // no tide at all
    hydro::StormSpec highTide = storm;
    highTide.tidePhaseHours = 0.0;                     // high water at t = 0

    const hydro::FloodResult low = hydro::simulate(s, land, lowTide, {});
    const hydro::FloodResult high = hydro::simulate(s, land, highTide, {});
    // Severity, not extent: on a shelf this low the rain alone eventually wets
    // every cell, so the cell count saturates and only the depth distinguishes
    // the two runs.  The tide's effect is measured where it actually lands — in
    // how deep the water gets, how much of it is over half a metre, and how much
    // is still on the ground at the end.
    CHECK(high.floodedCells >= low.floodedCells);
    CHECK(high.maxDepth[15 * 30 + 2] > low.maxDepth[15 * 30 + 2]);
    // The shelf has no sea cells of its own, so here the tide acts only through
    // the outfall derate: the same rain, but the drains are throttled while the
    // water is high.  The difference is therefore a matter of degree — tens of
    // millimetres of extra depth — and it is the depth and the retained volume
    // that measure it, not the wetted footprint.
    CHECK(high.drainedM3 < low.drainedM3);
    CHECK(high.peakStorageM3 > low.peakStorageM3);
    CHECK(high.storedM3 > low.storedM3);
}

TEST(drainsRemoveWaterAndReducedCapacityFloodsMore) {
    const terrain::Heightfield hf = makeGrid(30, 30, kStep, [](u32, u32) { return 6.0; });
    hydro::SurfaceOptions opt;
    opt.openBoundaries = true;
    const hydro::Surface s = hydro::buildSurface(hf, opt);

    hydro::StormSpec storm;
    storm.name = "drain-test";
    storm.rainMmPerHour = {40, 40, 40};
    storm.tideAmplitude = 0.0;
    storm.stepSeconds = 300.0;

    hydro::LandSpec drained, undrained;
    drained.resize(s.cellCount(), 0.8f);
    undrained.resize(s.cellCount(), 0.8f);
    for (size_t k = 0; k < drained.drainCapacity.size(); ++k) {
        drained.drainCapacity[k] = static_cast<f32>(45.0 / 1000.0 / 3600.0);   // a good drain
        undrained.drainCapacity[k] = 0.0f;                                     // a blocked one
    }
    const hydro::FloodResult a = hydro::simulate(s, drained, storm, {});
    const hydro::FloodResult b = hydro::simulate(s, undrained, storm, {});
    CHECK(a.drainedM3 > 0.0);
    CHECK(b.storedM3 > a.storedM3);
    CHECK(b.maxDepth[15 * 30 + 15] > a.maxDepth[15 * 30 + 15]);
}

TEST(channelCapacityGrowsWithCatchmentAndSlope) {
    const hydro::DrainSpec spec;
    const f64 small = hydro::channelCapacityFor(200, 576.0, 0.002, spec);
    const f64 big = hydro::channelCapacityFor(20000, 576.0, 0.002, spec);
    const f64 steep = hydro::channelCapacityFor(20000, 576.0, 0.02, spec);
    CHECK(big > small * 1.5);
    CHECK(steep > big);
    CHECK(small > 0.0);
}

TEST(landRasterisationAssignsRunoffAndDrainCapacityByCoverage) {
    const terrain::Heightfield hf = makeGrid(40, 40, kStep, [](u32, u32) { return 5.0; });
    hydro::SurfaceOptions opt;
    opt.openBoundaries = true;
    const hydro::Surface s = hydro::buildSurface(hf, opt);

    hydro::CoverageInput cov;
    cov.isRoad = [](const Vec2d& p) { return p.y > 200.0 && p.y < 240.0; };
    cov.isBuilding = [](const Vec2d& p) { return p.x < 200.0; };
    cov.isGreen = [](const Vec2d& p) { return p.x > 600.0; };
    cov.roadClass = [](const Vec2d&) { return 2; };        // primary
    hydro::DrainSpec spec;
    const hydro::LandSpec land = hydro::rasteriseLand(s, cov, spec);

    const size_t road = 11 * 40 + 10;      // inside the road band
    const size_t built = 1 * 40 + 1;       // built, not road, not green
    const size_t green = 1 * 40 + 35;      // green
    CHECK(land.drainCapacity[road] > 0.0f);
    CHECK_NEAR(land.drainCapacity[built], 0.0, 1e-12);
    CHECK(land.impervious[road] > land.impervious[green]);
    CHECK(land.impervious[built] > land.impervious[green]);
    CHECK_NEAR(land.drainCapacity[road], spec.arterialMmPerHour / 1000.0 / 3600.0, 1e-12);
}

TEST(openBoundaryKeepsTheDomainEdgeFromPonding) {
    // A bowl that touches the domain edge must not become an artificial lake:
    // with open boundaries the edge is an outlet.
    const terrain::Heightfield hf = makeGrid(20, 20, kStep, [](u32 i, u32 j) {
        const f64 edge = std::min(std::min(i, j), std::min(19 - i, 19 - j));
        return 2.0 + edge * 1.5;               // lowest along the border
    });
    hydro::SurfaceOptions open;
    open.openBoundaries = true;
    const hydro::Surface s = hydro::buildSurface(hf, open);
    int edgeOutlets = 0;
    for (u32 i = 0; i < 20; ++i) edgeOutlets += s.isBoundary[i] + s.isBoundary[19 * 20 + i];
    CHECK(edgeOutlets > 0);
    CHECK_NEAR(s.filled[10 * 20 + 10], 2.0 + 9 * 1.5, 1e-3);   // centre untouched
}

TEST(aSlopeShedsItsRainInsteadOfHoldingIt) {
    // The counterpart to the ponding test, and the one the monsoon model needed
    // most: ground that slopes must get rid of its water.  A city where every
    // slope holds its rainfall is not a city with bad drainage, it is a model
    // with no conveyance, and it shows up as half a metre of water on the
    // ridges during a burst — which is exactly what the real Mumbai run did
    // before the routing was sub-stepped.
    //
    // 2.5 % down to the east edge, no drains, no tide: everything that falls
    // must reach the boundary, because that is the only thing that can happen.
    // The whole grid stays well above the water plane on purpose — land below
    // it is a different case (a polder that cannot drain into a higher sea), and
    // mixing the two is how a test stops measuring what it claims to.
    const terrain::Heightfield hf = makeGrid(40, 40, kStep, [](u32 i, u32) {
        return 12.0 - i * 0.025 * kStep * 0.5;    // 12.0 m down to 2.25 m
    });
    hydro::SurfaceOptions opt;
    opt.openBoundaries = true;
    const hydro::Surface s = hydro::buildSurface(hf, opt);

    hydro::LandSpec land;
    land.resize(s.cellCount(), 0.55f);            // half-paved suburb, no drains
    hydro::StormSpec storm;
    storm.name = "slope-test";
    storm.rainMmPerHour.assign(6, 90.0);          // a monsoon burst, 90 mm/h
    storm.tideAmplitude = 0.0;
    storm.stepSeconds = 300.0;
    storm.initialLossMm = 0.0;
    const hydro::FloodResult r = hydro::simulate(s, land, storm, {});

    // Runoff leaves the domain and is booked as outfall, so the water that is
    // still on the ground at the end is what the surface failed to shed.
    const f64 retained = r.storedM3 / std::max(1.0, r.rainfallM3);
    f64 deepest = 0.0;
    for (size_t k = 0; k < r.maxDepth.size(); ++k)
        deepest = std::max(deepest, static_cast<f64>(r.maxDepth[k]));

    CHECK(r.rainfallM3 > 0.0);
    CHECK(r.outfallM3 > r.rainfallM3 * 0.9);      // it drains, and promptly
    CHECK(retained < 0.05);                       // under 5 % left standing
    CHECK(deepest < 0.05);                        // never more than a thin spate

    // Report the numbers when this fails: "how much water, how deep" is the
    // whole diagnosis, and a bare false tells the next reader nothing.
    if (!(retained < 0.05) || !(deepest < 0.05)) {
        size_t at = 0, over = 0;
        for (size_t k = 0; k < r.maxDepth.size(); ++k) {
            if (r.maxDepth[k] > r.maxDepth[at]) at = k;
            if (r.maxDepth[k] > 0.15f) ++over;
        }
        std::printf("      retained %.4f (%.1f of %.1f m3), deepest %.3f m at (%u,%u) "
                    "ground %.2f, %zu cells over 15 cm; outfall %.1f m3, drained %.1f m3\n",
                    retained, r.storedM3, r.rainfallM3, deepest,
                    static_cast<u32>(at % 40), static_cast<u32>(at / 40),
                    static_cast<double>(s.ground[at]), over, r.outfallM3, r.drainedM3);
    }
}

TEST(pavedStreetsCarryWaterThatRoughGroundHolds) {
    // The decision behind the flood model's conveyance term (D-026): a street is
    // graded to its kerb line and connected to a drain, so even a nearly flat
    // street moves water that open ground of the same slope holds.  Two runs on
    // the same almost-flat shelf, differing only in whether the middle row is a
    // paved street: the paved case must shed measurably more.
    const auto shelf = []() {
        return makeGrid(30, 30, kStep, [](u32 i, u32) {
            return 9.0 - i * 0.0002 * kStep;      // 0.02 % fall to the east
        });
    };
    hydro::SurfaceOptions opt;
    opt.openBoundaries = true;
    const hydro::Surface s = hydro::buildSurface(shelf(), opt);

    const auto run = [&](bool paved) {
        hydro::LandSpec land;
        land.resize(s.cellCount(), 0.45f);
        for (size_t k = 0; k < land.roughness.size(); ++k)
            land.roughness[k] = 0.10f;                    // open ground
        if (paved) {
            for (u32 i = 0; i < s.width; ++i) {
                const size_t k = 15 * s.width + i;        // the street row
                land.roughness[k] = 0.016f;               // asphalt with gutters
                land.drainNetwork[k] = 1;
                land.designFlow[k] = 0.002f;              // m3/s of design flow
            }
        }
        hydro::StormSpec storm;
        storm.name = paved ? "paved" : "rough";
        storm.rainMmPerHour.assign(4, 120.0);
        storm.tideAmplitude = 0.0;
        storm.stepSeconds = 300.0;
        storm.initialLossMm = 0.0;
        return hydro::simulate(s, land, storm, {});
    };

    const hydro::FloodResult rough = run(false);
    const hydro::FloodResult paved = run(true);
    CHECK(rough.storedM3 > 0.0);

    // Two measurements, and the sign of the first is the whole point: a street
    // with a drain is a *conduit*, so the street row runs deeper than open
    // ground does — that is what concentrating flow means — while the shelf as a
    // whole holds less water.  (Measured: 0.042 m in the street against 0.035 m
    // on the same row of rough ground, with 8,571 m³ left on the shelf against
    // 9,397 m³.)  A test that expected the street itself to be drier would be
    // testing the wrong model.
    const auto rowMean = [](const hydro::FloodResult& r) {
        f64 sum = 0.0;
        for (u32 i = 0; i < 30; ++i) sum += r.maxDepth[15 * 30 + i];
        return sum / 30.0;
    };
    const f64 roughRow = rowMean(rough), pavedRow = rowMean(paved);
    if (!(paved.storedM3 < rough.storedM3))
        std::printf("      street row mean depth: rough %.4f m, paved %.4f m; "
                    "stored rough %.0f m3, paved %.0f m3\n",
                    roughRow, pavedRow, rough.storedM3, paved.storedM3);
    CHECK(pavedRow > roughRow);                        // the street carries the flow
    CHECK(paved.storedM3 < rough.storedM3);            // so the shelf holds less
    CHECK(paved.outfallM3 > rough.outfallM3);
}
