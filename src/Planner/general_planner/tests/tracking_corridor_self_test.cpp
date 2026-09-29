#include "elastic_tracking_test_utils.hpp"
#include <general_core/tracking/tracking_corridor_builder.hpp>
#include <iostream>

using elastic_test::require;
using general_utils::Vec3f;
using general_utils::vec_Vec3f;
using geometry_utils::Polytope;
using namespace general_planner;

namespace {
TrackingCorridorOps freeOps() {
    TrackingCorridorOps ops;
    ops.point_safe = [](const Vec3f &p) { return p.allFinite(); };
    ops.line_free = [](const Vec3f &, const Vec3f &) { return true; };
    ops.point_poly = [](const Vec3f &p, Polytope &poly) {
        poly = elastic_test::box(p - Vec3f::Constant(.2), p + Vec3f::Constant(.2));
        return true;
    };
    ops.line_poly = [](const Vec3f &a, const Vec3f &b, Polytope &poly) {
        poly = elastic_test::box(a.cwiseMin(b) - Vec3f::Constant(.2),
                                a.cwiseMax(b) + Vec3f::Constant(.2));
        return true;
    };
    return ops;
}
void check(const TrackingCorridorReport &report, const geometry_utils::PolytopeVec &sfcs) {
    require(!sfcs.empty() && sfcs.size() == report.spans.size(), "missing seed provenance");
    for (std::size_t i = 0; i < sfcs.size(); ++i) {
        for (std::size_t k = report.spans[i].begin; k <= report.spans[i].end; ++k)
            require(sfcs[i].PointIsInside(report.path[k], 1.e-6), "uncovered path point");
        if (i) {
            auto previous = sfcs[i-1];
            require(previous.HaveOverlapWith(sfcs[i], 1.e-4), "disconnected corridor accepted");
        }
    }
    require(sfcs.front().PointIsInside(report.path.front(), 1.e-6) &&
            sfcs.back().PointIsInside(report.path.back(), 1.e-6), "missing execution endpoint");
}
void longSparseGuide() {
    TrackingCorridorReport report;
    geometry_utils::PolytopeVec sfcs;
    const vec_Vec3f path{{1.634,-3.782,1.697},{2.965,.684,1.931}};
    require(buildTrackingCorridor(path, {}, freeOps(), sfcs, report), "sparse bag-like catch-up edge failed");
    require(report.path.size() > path.size() && sfcs.size() >= 3, "long edge was skipped instead of subdivided");
    check(report, sfcs);
}
void bridgeUsesSharedVertex() {
    auto ops = freeOps();
    vec_Vec3f path;
    for (int i=0;i<=16;++i) path.push_back({.25*i,0,1});
    for (int i=1;i<=16;++i) path.push_back({4,.25*i,1});
    ops.line_poly = [](const Vec3f &a, const Vec3f &b, Polytope &poly) {
        Vec3f lower = a.cwiseMin(b) - Vec3f::Constant(.2);
        Vec3f upper = a.cwiseMax(b) + Vec3f::Constant(.2);
        if (a.x() < 3.9) upper.x() = b.x();
        else lower.x() = a.x();
        poly = elastic_test::box(lower, upper);
        return true;
    };
    bool bridged = false;
    ops.point_poly = [&](const Vec3f &p, Polytope &poly) {
        require((p-Vec3f(4,0,1)).norm() < 1.e-8, "bridge used SFC index as path index");
        bridged = true;
        poly = elastic_test::box(p-Vec3f::Constant(.2),p+Vec3f::Constant(.2));
        return true;
    };
    TrackingCorridorConfig cfg; cfg.max_seed_length=4;
    TrackingCorridorReport report; geometry_utils::PolytopeVec sfcs;
    require(buildTrackingCorridor(path,cfg,ops,sfcs,report) && bridged,"shared-vertex bridge missing");
    require(report.spans[1].bridge && report.spans[1].begin==16,"wrong bridge provenance");
    check(report,sfcs);
}
void curvedGuideAndRefinement() {
    auto ops=freeOps();
    ops.line_poly=[](const Vec3f &a,const Vec3f &b,Polytope &poly) {
        if ((a-b).norm() > 1.5) return false;
        poly=elastic_test::box(a.cwiseMin(b)-Vec3f::Constant(.15),a.cwiseMax(b)+Vec3f::Constant(.15));
        return true;
    };
    TrackingCorridorConfig cfg;cfg.max_seed_length=5;
    TrackingCorridorReport report;geometry_utils::PolytopeVec sfcs;
    require(buildTrackingCorridor({{0,0,1},{1,1,1},{2,0,1}},cfg,ops,sfcs,report),"curved guide refinement failed");
    check(report,sfcs);
    // A CIRI success flag alone does not guarantee coverage of the bent guide.
    ops=freeOps();
    ops.line_poly=[](const Vec3f &a,const Vec3f &b,Polytope &poly) {
        poly=elastic_test::box(a.cwiseMin(b)-Vec3f::Constant(.15),a.cwiseMax(b)+Vec3f::Constant(.15));
        return true;
    };
    require(buildTrackingCorridor({{0,0,1},{1,1,1},{2,0,1}},cfg,ops,sfcs,report),"uncovered bend was not repaired");
    require(sfcs.size()==2,"shortcut silently dropped the guide apex");check(report,sfcs);
}
void unsafeCasesAndHover() {
    auto ops=freeOps();TrackingCorridorReport report;geometry_utils::PolytopeVec sfcs;
    ops.line_free=[](const Vec3f &,const Vec3f &) {return false;};
    require(!buildTrackingCorridor({{0,0,1},{3,0,1}},{},ops,sfcs,report),"blocked edge skipped");
    ops=freeOps();
    ops.line_poly=[](const Vec3f &a,const Vec3f &b,Polytope &poly) {
        Vec3f upper=a.cwiseMax(b)+Vec3f::Constant(.2);upper.x()=b.x()-.02;
        poly=elastic_test::box(a.cwiseMin(b)-Vec3f::Constant(.2),upper);return true;
    };
    require(!buildTrackingCorridor({{0,0,1},{1,0,1}},{},ops,sfcs,report),"2 cm outside endpoint accepted as 5 cm clearance");
    require(report.attempts<20,"infeasible seed retry unbounded");
    require(trackingCorridorGeometry(report,sfcs).find("candidate_planes=")!=std::string::npos,"failure geometry missing");
    ops=freeOps();TrackingCorridorConfig cfg;cfg.max_attempts=1;
    ops.line_poly=[](const Vec3f &,const Vec3f &,Polytope &) {return false;};
    require(!buildTrackingCorridor({{0,0,1},{1,0,1}},cfg,ops,sfcs,report) && report.attempts==1,"attempt budget ignored");
    require(buildTrackingCorridor({{0,0,1},{0,0,1}},{},freeOps(),sfcs,report),"static hover failed");check(report,sfcs);
}
}
int main() {
    try {
        longSparseGuide();bridgeUsesSharedVertex();curvedGuideAndRefinement();unsafeCasesAndHover();
        std::cout<<"tracking_corridor_self_test PASS: sparse tail, shared bridge, bend coverage, refinement, rejection, bounds, hover\n";
        return 0;
    } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
