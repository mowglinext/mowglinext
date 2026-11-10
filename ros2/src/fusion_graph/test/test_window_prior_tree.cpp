// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Window rebuilds (RebaseISAM2, RigidTransformAll, Load) re-anchor every pose
// with an independent prior. Added one by one, each prior became its own
// Bayes-tree root and iSAM2's back-substitution visited all of them on every
// pose read — the dominant per-node cost once the window is full. They are now
// added through AddPosePriorTree (LinkedPosePriorFactor: the same prior plus an
// exactly-zero Jacobian on a parent pose): plain tail + 1 roots instead of N.
//
// These tests pin that the change is purely structural: the linearisation is
// the plain prior's, and an iSAM2 fed the tree reproduces the old plain-prior
// iSAM2 bit for bit — head pose, marginal covariance and every other pose —
// through chain growth, delayed GNSS on recent AND deep window poses, and
// relinearisation after large corrections.
//
// One bounded exception, pinned by the fault-injection test: a factor on a
// linked (deep) pose makes iSAM2 eliminate that pose last, so it becomes the
// root of the linked tree, merged into one clique with its tree parent (the
// same happens to a pose whose tree parent is relinearised). Once such a pose
// carries three or more non-zero factors, GTSAM may sum them in a different
// order there and the pose itself can differ in the last bits. Deep poses
// touch the live chain only through the exactly-zero links, so the head pose,
// its covariance and everything published from them stay bit-identical.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <random>
#include <string>
#include <typeinfo>
#include <vector>

#include "fusion_graph/factors.hpp"
#include "fusion_graph/graph_manager.hpp"
#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <unistd.h>

namespace fusion_graph
{
struct GraphManagerTestAccess
{
  static constexpr size_t kPlainTail = GraphManager::kWindowPlainPriors;
  static size_t Roots(GraphManager& g)
  {
    std::lock_guard<std::mutex> lock(g.mu_);
    return g.isam_.roots().size();
  }
  static gtsam::ISAM2 CopyIsam(GraphManager& g)
  {
    std::lock_guard<std::mutex> lock(g.mu_);
    return g.isam_;
  }
  static gtsam::Values Estimate(GraphManager& g)
  {
    std::lock_guard<std::mutex> lock(g.mu_);
    return g.isam_.calculateEstimate();
  }
};

// Bayes-tree shape: roots, largest clique (frontal / separator variables) and
// depth. Used to rule out fill or a degenerate ordering from the zero links.
struct TreeShape
{
  size_t roots = 0, max_frontals = 0, max_separator = 0, max_depth = 0;
};
inline TreeShape ShapeOf(const gtsam::ISAM2& isam)
{
  TreeShape t;
  t.roots = isam.roots().size();
  std::vector<std::pair<gtsam::ISAM2::sharedClique, size_t>> stack;
  for (const auto& r : isam.roots())
    stack.emplace_back(r, 1);
  while (!stack.empty())
  {
    const auto [c, depth] = stack.back();
    stack.pop_back();
    t.max_frontals = std::max<size_t>(t.max_frontals, c->conditional()->nrFrontals());
    t.max_separator = std::max<size_t>(t.max_separator, c->conditional()->nrParents());
    t.max_depth = std::max(t.max_depth, depth);
    for (const auto& ch : c->children)
      stack.emplace_back(ch, depth + 1);
  }
  return t;
}
}  // namespace fusion_graph

namespace fg = fusion_graph;

namespace
{

// GraphManager's optimiser settings.
gtsam::ISAM2 MakeIsam()
{
  gtsam::ISAM2Params p;
  p.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.001);
  p.relinearizeThreshold = 0.05;
  p.relinearizeSkip = 5;
  return gtsam::ISAM2(p);
}

gtsam::SharedNoiseModel Iso(double s)
{
  return gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(s, s, s));
}

// Bit for bit, on the stored Rot2 components too, so a -0.0 / +0.0 or a
// c, s difference that theta() rounds away still counts.
void ExpectSamePose(const gtsam::Pose2& a, const gtsam::Pose2& b)
{
  const double pa[] = {a.x(), a.y(), a.rotation().c(), a.rotation().s()};
  const double pb[] = {b.x(), b.y(), b.rotation().c(), b.rotation().s()};
  EXPECT_EQ(std::memcmp(pa, pb, sizeof(pa)), 0)
      << "(" << pa[0] << ", " << pa[1] << ", " << pa[2] << ", " << pa[3] << ") vs (" << pb[0]
      << ", " << pb[1] << ", " << pb[2] << ", " << pb[3] << ")";
}

}  // namespace

TEST(WindowPriorTree, LinkedPriorLinearisesExactlyLikeThePlainPrior)
{
  const gtsam::PriorFactor<gtsam::Pose2> prior(fg::PoseKey(3),
                                               gtsam::Pose2(1.5, -2.0, 0.7),
                                               Iso(0.05));
  const fg::LinkedPosePriorFactor linked(prior, fg::PoseKey(1));
  ASSERT_EQ(linked.keys(), (gtsam::KeyVector{fg::PoseKey(3), fg::PoseKey(1)}));
  EXPECT_TRUE(linked.equals(*linked.clone()));
  EXPECT_FALSE(linked.equals(fg::LinkedPosePriorFactor(
      gtsam::PriorFactor<gtsam::Pose2>(fg::PoseKey(3), gtsam::Pose2(1.5, -2.0, 0.8), Iso(0.05)),
      fg::PoseKey(1))));

  for (const gtsam::Pose2& at : {gtsam::Pose2(1.5, -2.0, 0.7), gtsam::Pose2(1.62, -1.9, 0.81)})
  {
    gtsam::Values v;
    v.insert(fg::PoseKey(3), at);
    v.insert(fg::PoseKey(1), gtsam::Pose2(9.0, 9.0, 2.0));
    EXPECT_EQ(linked.error(v), prior.error(v));
    const auto a = std::dynamic_pointer_cast<gtsam::JacobianFactor>(prior.linearize(v));
    const auto b = std::dynamic_pointer_cast<gtsam::JacobianFactor>(linked.linearize(v));
    ASSERT_TRUE(a && b);
    EXPECT_TRUE(gtsam::Matrix(b->getA(b->begin())) == gtsam::Matrix(a->getA(a->begin())));
    EXPECT_TRUE(gtsam::Matrix(b->getA(b->begin() + 1)) == gtsam::Matrix::Zero(3, 3));
    EXPECT_TRUE(gtsam::Vector(b->getb()) == gtsam::Vector(a->getb()));
    EXPECT_EQ(b->get_model(), nullptr);
  }
}

TEST(WindowPriorTree, TreeSharesOneRootAndReproducesPlainPriorsBitForBit)
{
  constexpr size_t kWindow = 1500;
  constexpr size_t kTail = fg::GraphManagerTestAccess::kPlainTail;
  std::mt19937_64 rng(7);
  std::normal_distribution<double> n01(0.0, 1.0);

  // A rebuilt window: poses along a serpentine, each anchored by its own prior.
  std::vector<gtsam::PriorFactor<gtsam::Pose2>> priors;
  gtsam::Values window;
  for (size_t i = 0; i < kWindow; ++i)
  {
    const gtsam::Pose2 p(0.01 * static_cast<double>(i),
                         std::sin(0.002 * static_cast<double>(i)),
                         0.3 + 0.001 * static_cast<double>(i));
    priors.emplace_back(fg::PoseKey(i), p, Iso(0.05));
    window.insert(fg::PoseKey(i), p);
  }
  gtsam::NonlinearFactorGraph plain;
  for (const auto& p : priors)
    plain.add(p);
  gtsam::NonlinearFactorGraph tree;
  fg::AddPosePriorTree(tree, priors, kTail);

  auto old_isam = MakeIsam();
  auto new_isam = MakeIsam();
  old_isam.update(plain, window);
  new_isam.update(tree, window);
  EXPECT_EQ(old_isam.roots().size(), kWindow);
  EXPECT_EQ(new_isam.roots().size(), kTail + 1);

  // Live graph after the rebuild: chain growth from the window head with
  // odometry, GNSS-like unaries (some delayed onto recent nodes), periodic
  // large corrections that force relinearisation, and occasional factors on
  // deep window poses (a stale fix, an anchor) — the cases where the tree
  // shape differs.
  uint64_t head = kWindow - 1;
  gtsam::Pose2 truth = window.at<gtsam::Pose2>(fg::PoseKey(head));
  for (int step = 0; step < 600; ++step)
  {
    gtsam::NonlinearFactorGraph f;
    gtsam::Values v;
    const gtsam::Pose2 odo(0.02, 0.0, 0.01 * std::sin(0.05 * step));
    truth = truth.compose(odo);
    const auto k = fg::PoseKey(head + 1);
    v.insert(k, old_isam.calculateEstimate<gtsam::Pose2>(fg::PoseKey(head)).compose(odo));
    f.add(gtsam::BetweenFactor<gtsam::Pose2>(
        fg::PoseKey(head),
        k,
        gtsam::Pose2(odo.x() + 0.002 * n01(rng), 0.0, odo.theta() + 0.002 * n01(rng)),
        Iso(0.01)));
    const double jump = (step % 150 == 149) ? 0.4 : 0.0;  // RTK re-fix after drift
    const uint64_t gnss_node = head + 1 - static_cast<uint64_t>(step % 3);
    f.add(fg::GnssLeverArmFactor(fg::PoseKey(gnss_node),
                                 gtsam::Vector2(truth.x() + jump + 0.01 * n01(rng),
                                                truth.y() + 0.01 * n01(rng)),
                                 gtsam::Vector2(0.3, 0.0),
                                 gtsam::noiseModel::Isotropic::Sigma(2, 0.02)));
    if (step % 97 == 13)  // stale fix / anchor deep inside the linked window
      f.add(gtsam::PriorFactor<gtsam::Pose2>(
          fg::PoseKey(static_cast<uint64_t>(step) % (kWindow - kTail)),
          gtsam::Pose2(0.1 * n01(rng), 0.1 * n01(rng), 0.05 * n01(rng)),
          Iso(0.1)));
    old_isam.update(f, v);
    new_isam.update(f, v);
    ++head;

    ExpectSamePose(new_isam.calculateEstimate<gtsam::Pose2>(fg::PoseKey(head)),
                   old_isam.calculateEstimate<gtsam::Pose2>(fg::PoseKey(head)));
    if (step % 10 == 0)
    {
      EXPECT_TRUE(new_isam.marginalCovariance(fg::PoseKey(head)) ==
                  old_isam.marginalCovariance(fg::PoseKey(head)))
          << "step " << step;
    }
  }

  const auto old_all = old_isam.calculateEstimate();
  const auto new_all = new_isam.calculateEstimate();
  ASSERT_EQ(old_all.size(), new_all.size());
  for (const auto& kv : old_all)
    ExpectSamePose(new_all.at<gtsam::Pose2>(kv.key), kv.value.cast<gtsam::Pose2>());
}

TEST(WindowPriorTree, RebaseLinksTheWindowAndLeavesTheEstimateUnchanged)
{
  fg::GraphParams gp;
  gp.node_period_s = 0.1;
  gp.stationary_node_period_s = 0.0;
  gp.stationary_motion_thresh_m = 0.0;
  gp.stationary_motion_thresh_theta = 0.0;
  gp.max_graph_nodes = 1000;
  fg::GraphManager gm(gp);
  gm.Initialize(gtsam::Pose2(), 0.0);
  for (int i = 0; i < 1200; ++i)
  {
    gm.AddWheelTwist(0.3, 0.0, 0.0, 0.1);
    gm.AddGyroDelta(0.0, 0.1);
    gm.QueueGnss(0.03 * (i + 1), 0.0, 0.01, true, std::nullopt, 0);
    gm.Tick(0.101 * (i + 1));  // strictly > node_period_s apart: a node per tick
  }
  const auto before = fg::GraphManagerTestAccess::Estimate(gm);
  ASSERT_GT(before.size(), 1100u);
  gm.RebaseISAM2();
  EXPECT_LE(fg::GraphManagerTestAccess::Roots(gm), fg::GraphManagerTestAccess::kPlainTail + 1);

  // The rebuilt window sits exactly at its prior means: every kept pose is
  // unchanged, and the 1 Hz consumers report exactly the iSAM2 estimate.
  const auto after = fg::GraphManagerTestAccess::Estimate(gm);
  EXPECT_EQ(after.size(), 1000u);
  for (const auto& kv : after)
    ExpectSamePose(kv.value.cast<gtsam::Pose2>(), before.at<gtsam::Pose2>(kv.key));
  const auto poses = gm.GetAllPoses();
  EXPECT_EQ(poses.size(), after.size());
  EXPECT_EQ(gm.LiveNodeCount(), after.size());
  for (const auto& [idx, p] : poses)
    ExpectSamePose(p, after.at<gtsam::Pose2>(fg::PoseKey(idx)));
}

// The production window (6000 poses + a 2000-node live chain): the tree must
// be one shallow root with pose-sized cliques — no fill from the zero links.
TEST(WindowPriorTree, ProductionSizedWindowHasPoseSizedCliquesAndAShallowTree)
{
  constexpr size_t kWindow = 6000;
  std::vector<gtsam::PriorFactor<gtsam::Pose2>> priors;
  gtsam::Values window;
  for (size_t i = 0; i < kWindow; ++i)
  {
    const gtsam::Pose2 p(0.01 * static_cast<double>(i), 0.0, 0.0);
    priors.emplace_back(fg::PoseKey(i), p, Iso(0.05));
    window.insert(fg::PoseKey(i), p);
  }
  gtsam::NonlinearFactorGraph plain;
  for (const auto& p : priors)
    plain.add(p);
  gtsam::NonlinearFactorGraph tree;
  fg::AddPosePriorTree(tree, priors, fg::GraphManagerTestAccess::kPlainTail);
  auto old_isam = MakeIsam();
  auto new_isam = MakeIsam();
  old_isam.update(plain, window);
  new_isam.update(tree, window);
  // The rebuilt window alone: a single shallow linked tree (COLAMD picks its
  // root, so bound the depth by the tree's diameter, 2 * ceil(log2 N)).
  const auto linked = fg::ShapeOf(new_isam);
  std::printf("[shape] rebuilt window: roots=%zu max_frontals=%zu max_separator=%zu depth=%zu\n",
              linked.roots,
              linked.max_frontals,
              linked.max_separator,
              linked.max_depth);
  EXPECT_LE(linked.max_depth, 2u * 13u + 1u);
  EXPECT_LE(linked.max_separator, 1u);
  for (uint64_t i = kWindow; i < kWindow + 2000; ++i)
  {
    gtsam::NonlinearFactorGraph f;
    gtsam::Values v;
    v.insert(fg::PoseKey(i), gtsam::Pose2(0.01 * static_cast<double>(i), 0.0, 0.0));
    f.add(gtsam::BetweenFactor<gtsam::Pose2>(
        fg::PoseKey(i - 1), fg::PoseKey(i), gtsam::Pose2(0.01, 0.0, 0.0), Iso(0.01)));
    f.add(gtsam::PriorFactor<gtsam::Pose2>(fg::PoseKey(i),
                                           gtsam::Pose2(0.01 * static_cast<double>(i), 0.0, 0.0),
                                           Iso(0.02)));
    old_isam.update(f, v);
    new_isam.update(f, v);
  }
  const auto a = fg::ShapeOf(old_isam);
  const auto b = fg::ShapeOf(new_isam);
  std::printf("[shape] plain priors: roots=%zu max_frontals=%zu max_separator=%zu depth=%zu\n",
              a.roots,
              a.max_frontals,
              a.max_separator,
              a.max_depth);
  std::printf("[shape] prior tree  : roots=%zu max_frontals=%zu max_separator=%zu depth=%zu\n",
              b.roots,
              b.max_frontals,
              b.max_separator,
              b.max_depth);
  EXPECT_EQ(b.roots, fg::GraphManagerTestAccess::kPlainTail + 1);
  EXPECT_LE(b.max_separator, std::max<size_t>(a.max_separator, 1));
  EXPECT_LE(b.max_frontals, std::max<size_t>(a.max_frontals, 3));
  EXPECT_EQ(b.max_depth, a.max_depth);  // set by the live chain, not the window
}

// Every way the window is rebuilt — consecutive rebases whose cutoff drops
// tree parents, the rigid dock re-pin, a checkpoint load, clear_graph — builds
// a fresh iSAM2 whose tree only links poses it keeps (nothing in fusion_graph
// removes factors from a live iSAM2). Each must leave plain tail + 1 roots and an exactly
// unchanged (or exactly transformed) estimate.
TEST(WindowPriorTree, EveryRebuildPathLinksTheWindowAndKeepsExactEstimates)
{
  using Access = fg::GraphManagerTestAccess;
  fg::GraphParams gp;
  gp.node_period_s = 0.1;
  gp.stationary_node_period_s = 0.0;
  gp.stationary_motion_thresh_m = 0.0;
  gp.stationary_motion_thresh_theta = 0.0;
  gp.max_graph_nodes = 600;
  fg::GraphManager gm(gp);
  gm.Initialize(gtsam::Pose2(), 0.0);
  int tick = 0;
  auto drive = [&](int n)
  {
    for (int i = 0; i < n; ++i, ++tick)
    {
      gm.AddWheelTwist(0.3, 0.0, 0.02, 0.1);
      gm.AddGyroDelta(0.02, 0.1);
      gm.QueueGnss(0.03 * (tick + 1), 0.001 * tick, 0.01, true, std::nullopt, 0);
      gm.Tick(0.101 * (tick + 1));
    }
  };
  auto expect_rebuilt_from = [&](const gtsam::Values& before, size_t kept)
  {
    EXPECT_EQ(Access::Roots(gm), Access::kPlainTail + 1);
    const auto after = Access::Estimate(gm);
    EXPECT_EQ(after.size(), kept);
    for (const auto& kv : after)
      ExpectSamePose(kv.value.cast<gtsam::Pose2>(), before.at<gtsam::Pose2>(kv.key));
  };

  // 1. Rebase, live chain grows on the tree, rebase again with the window
  //    sliding past the old tree root and half its subtrees.
  drive(800);
  auto before = Access::Estimate(gm);
  gm.RebaseISAM2();
  expect_rebuilt_from(before, 600);
  drive(450);
  EXPECT_EQ(Access::Roots(gm), Access::kPlainTail + 1);  // chain hangs off the plain head
  before = Access::Estimate(gm);
  gm.RebaseISAM2();
  expect_rebuilt_from(before, 600);

  // 2. Rigid dock re-pin: every pose moves by exactly the correction.
  drive(50);
  before = Access::Estimate(gm);
  const gtsam::Pose2 correction(0.4, -0.2, 0.05);
  gm.RigidTransformAll(correction);
  EXPECT_EQ(Access::Roots(gm), Access::kPlainTail + 1);
  {
    const auto after = Access::Estimate(gm);
    ASSERT_EQ(after.size(), before.size());
    for (const auto& kv : after)
      ExpectSamePose(kv.value.cast<gtsam::Pose2>(), correction * before.at<gtsam::Pose2>(kv.key));
  }

  // 3. Checkpoint save + load into a fresh manager.
  const auto prefix =
      (std::filesystem::temp_directory_path() / ("fg_prior_tree_" + std::to_string(::getpid())))
          .string();
  before = Access::Estimate(gm);
  ASSERT_TRUE(gm.Save(prefix));
  {
    fg::GraphManager loaded(gp);
    ASSERT_TRUE(loaded.Load(prefix));
    EXPECT_EQ(Access::Roots(loaded), Access::kPlainTail + 1);
    const auto after = Access::Estimate(loaded);
    EXPECT_EQ(after.size(), 600u);
    for (const auto& kv : after)
    {
      const auto want = before.at<gtsam::Pose2>(kv.key);
      const auto got = kv.value.cast<gtsam::Pose2>();
      EXPECT_NEAR(got.x(), want.x(), 1e-9);  // XML text round trip
      EXPECT_NEAR(got.y(), want.y(), 1e-9);
      EXPECT_NEAR(got.theta(), want.theta(), 1e-9);
    }
  }
  for (const char* ext : {".checkpoint", ".checkpoint.previous"})
    std::filesystem::remove(prefix + ext);

  // 4. clear_graph: Reset discards the tree with the iSAM2; a re-seeded graph
  //    starts from a single prior again.
  gm.Reset();
  EXPECT_EQ(Access::Roots(gm), 0u);
  gm.Initialize(gtsam::Pose2(1.0, 1.0, 0.0), 0.101 * (tick + 1));
  ++tick;
  drive(20);
  EXPECT_EQ(Access::Roots(gm), 1u);
}

namespace
{

// Distance in units in the last place between two finite doubles; 0 only
// when they are bit-identical. Opposite sign bits (including -0.0 vs +0.0)
// count as maximally apart, which also keeps the subtraction below in range.
int64_t Ulps(double a, double b)
{
  if (std::memcmp(&a, &b, sizeof(a)) == 0)
    return 0;
  if (std::signbit(a) != std::signbit(b))
    return std::numeric_limits<int64_t>::max();
  int64_t ia, ib;
  std::memcpy(&ia, &a, sizeof(a));
  std::memcpy(&ib, &b, sizeof(b));
  return ia > ib ? ia - ib : ib - ia;
}

// Outcome of one iSAM2 call: whether it threw and with which exception type.
template <typename F>
std::string Outcome(F&& f)
{
  try
  {
    f();
    return "ok";
  }
  catch (const std::exception& e)
  {
    return typeid(e).name();
  }
}

}  // namespace

// Seeded fault injection on the old (plain priors) and new (prior tree)
// iSAM2 side by side: Huber GNSS outliers up to 3 m, huge and tiny sigmas,
// outages, stale fixes and tight ForceAnchor-style priors on deep window
// poses, and repeated window rebuilds (each from its own estimate) whose
// cutoff slides past tree parents. Every step must give the bit-identical
// head pose, and periodically the identical marginal covariance; the whole
// window must be bit-identical apart from the merged-root-clique pose
// described above (at most one pose, at most 4 ulp). A final NaN measurement
// must fail the same way in both (same exception, or none) on update and on
// the next read.
TEST(WindowPriorTree, TreeMatchesPlainPriorsUnderFaultInjection)
{
  constexpr size_t kWindow = 300;
  constexpr size_t kTail = fg::GraphManagerTestAccess::kPlainTail;
  int64_t max_ulps = 0;
  size_t windows_differing = 0;
  for (unsigned seed = 1; seed <= 12; ++seed)
  {
    SCOPED_TRACE("seed " + std::to_string(seed));
    std::mt19937_64 rng(seed);
    std::normal_distribution<double> n01(0.0, 1.0);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    auto old_isam = MakeIsam();
    auto new_isam = MakeIsam();
    // Rebuild both from the (identical) current estimate, old style vs tree.
    // Rebuild each from its own estimate (as RebaseISAM2 does): old style vs
    // tree, sliding cutoff.
    auto rebuild = [&](uint64_t head)
    {
      for (auto* isam : {&old_isam, &new_isam})
      {
        const auto est = isam->calculateEstimate();
        std::vector<gtsam::PriorFactor<gtsam::Pose2>> priors;
        gtsam::Values kept;
        for (const auto& kv : est)
        {
          if (gtsam::Symbol(kv.key).index() + kWindow <= head)
            continue;
          priors.emplace_back(kv.key, kv.value.cast<gtsam::Pose2>(), Iso(0.05));
          kept.insert(kv.key, kv.value);
        }
        gtsam::NonlinearFactorGraph graph;
        if (isam == &old_isam)
          for (const auto& p : priors)
            graph.add(p);
        else
          fg::AddPosePriorTree(graph, priors, kTail);
        *isam = MakeIsam();
        isam->update(graph, kept);
      }
    };

    gtsam::Values seed_v;
    seed_v.insert(fg::PoseKey(0), gtsam::Pose2());
    gtsam::NonlinearFactorGraph seed_f;
    seed_f.add(gtsam::PriorFactor<gtsam::Pose2>(fg::PoseKey(0), gtsam::Pose2(), Iso(0.02)));
    old_isam.update(seed_f, seed_v);
    new_isam.update(seed_f, seed_v);

    uint64_t head = 0;
    gtsam::Pose2 truth;
    for (int step = 0; step < 900; ++step)
    {
      gtsam::NonlinearFactorGraph f;
      gtsam::Values v;
      const gtsam::Pose2 odo(0.02, 0.0, (step / 50) % 2 ? 0.03 : 0.0);
      truth = truth.compose(odo);
      const auto k = fg::PoseKey(head + 1);
      v.insert(k, old_isam.calculateEstimate<gtsam::Pose2>(fg::PoseKey(head)).compose(odo));
      f.add(gtsam::BetweenFactor<gtsam::Pose2>(
          fg::PoseKey(head),
          k,
          gtsam::Pose2(odo.x() + 0.002 * n01(rng), 0.0, odo.theta() + 0.002 * n01(rng)),
          Iso(0.01)));
      const bool outage = (step % 300) >= 200 && (step % 300) < 260;
      const double r = u01(rng);
      if (!outage)
      {
        double sigma = 0.014, dx = 0.0;
        if (r < 0.05)
          dx = 0.05 + 2.95 * u01(rng);  // wrong fix that got past the gate
        else if (r < 0.08)
          sigma = 0.5 + 4.5 * u01(rng);
        else if (r < 0.10)
          sigma = 0.003;
        const uint64_t lag = r > 0.97 ? std::min<uint64_t>(head, 1 + rng() % (kWindow - 1)) : 0;
        f.add(fg::GnssLeverArmFactor(
            fg::PoseKey(head + 1 - lag),
            gtsam::Vector2(truth.x() + dx + 0.008 * n01(rng), truth.y() + 0.008 * n01(rng)),
            gtsam::Vector2(0.3, 0.0),
            gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345),
                                              gtsam::noiseModel::Isotropic::Sigma(2, sigma))));
      }
      if (step % 37 == 11 && head > 2)  // ForceAnchor / set_pose on a deep pose
      {
        const uint64_t deep = head - 1 - rng() % std::min<uint64_t>(head - 1, kWindow - 1);
        const auto at = old_isam.calculateEstimate<gtsam::Pose2>(fg::PoseKey(deep));
        f.add(gtsam::PriorFactor<gtsam::Pose2>(
            fg::PoseKey(deep),
            gtsam::Pose2(at.x() + 0.05 * n01(rng), at.y() + 0.05 * n01(rng), at.theta()),
            gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.005, 0.005, 0.035))));
      }
      const auto a = Outcome(
          [&]
          {
            old_isam.update(f, v);
          });
      const auto b = Outcome(
          [&]
          {
            new_isam.update(f, v);
          });
      ASSERT_EQ(a, b) << "step " << step;
      ASSERT_EQ(a, "ok") << "step " << step;
      ++head;
      ExpectSamePose(new_isam.calculateEstimate<gtsam::Pose2>(fg::PoseKey(head)),
                     old_isam.calculateEstimate<gtsam::Pose2>(fg::PoseKey(head)));
      if (step % 25 == 0)
      {
        EXPECT_TRUE(new_isam.marginalCovariance(fg::PoseKey(head)) ==
                    old_isam.marginalCovariance(fg::PoseKey(head)))
            << "step " << step;
      }
      if (step % 120 == 119)
      {
        // Whole window: bit-identical, except that the pose merged into the
        // linked tree's root clique (see the header) may differ in the last
        // bits once a factor lands on it.
        const auto old_all = old_isam.calculateEstimate();
        const auto new_all = new_isam.calculateEstimate();
        ASSERT_EQ(old_all.size(), new_all.size());
        size_t differing = 0;
        for (const auto& kv : old_all)
        {
          const auto a = kv.value.cast<gtsam::Pose2>();
          const auto b = new_all.at<gtsam::Pose2>(kv.key);
          const int64_t ulps = std::max({Ulps(a.x(), b.x()),
                                         Ulps(a.y(), b.y()),
                                         Ulps(a.rotation().c(), b.rotation().c()),
                                         Ulps(a.rotation().s(), b.rotation().s())});
          if (ulps == 0)
            continue;
          ++differing;
          max_ulps = std::max(max_ulps, ulps);
          EXPECT_LE(ulps, 4) << "x" << gtsam::Symbol(kv.key).index() << " step " << step;
        }
        EXPECT_LE(differing, 1u) << "step " << step;
        windows_differing += differing > 0;
        // Rebase; every third one twice in a row.
        rebuild(head);
        if (step % 360 == 359)
          rebuild(head);
        EXPECT_EQ(new_isam.roots().size(), std::min(kTail, old_isam.roots().size() - 1) + 1);
      }
    }

    // A non-finite measurement past every input filter.
    gtsam::NonlinearFactorGraph bad;
    bad.add(fg::GnssLeverArmFactor(
        fg::PoseKey(head - 3),
        gtsam::Vector2(std::numeric_limits<double>::quiet_NaN(), 0.0),
        gtsam::Vector2(0.3, 0.0),
        gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345),
                                          gtsam::noiseModel::Isotropic::Sigma(2, 0.014))));
    EXPECT_EQ(Outcome(
                  [&]
                  {
                    old_isam.update(bad, gtsam::Values());
                  }),
              Outcome(
                  [&]
                  {
                    new_isam.update(bad, gtsam::Values());
                  }));
    EXPECT_EQ(Outcome(
                  [&]
                  {
                    old_isam.calculateEstimate();
                  }),
              Outcome(
                  [&]
                  {
                    new_isam.calculateEstimate();
                  }));
  }
  std::printf(
      "[faults] 12 seeds x 7 window checks: %zu with a root-clique pose off by <= %ld ulp\n",
      windows_differing,
      static_cast<long>(max_ulps));
}
