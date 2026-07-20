// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
#include <cmath>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "mowgli_map/obstacle_tracker_node.hpp"
#include <gtest/gtest.h>

// ─────────────────────────────────────────────────────────────────────────────
// Friend-class test fixture to access private algorithm methods
// ─────────────────────────────────────────────────────────────────────────────

class ObstacleTrackerAlgorithmTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override
  {
    node_ = std::make_shared<mowgli_map::ObstacleTrackerNode>();
  }

  void TearDown() override
  {
    node_.reset();
  }

  // Expose private methods via delegation
  std::vector<std::pair<double, double>> convex_hull(
      const std::vector<std::pair<double, double>>& pts)
  {
    return node_->convex_hull(pts);
  }

  std::vector<std::pair<double, double>> boundary_hull(
      const std::vector<std::pair<double, double>>& pts)
  {
    return node_->boundary_hull(pts);
  }

  std::vector<std::pair<double, double>> inflate_hull(
      const std::vector<std::pair<double, double>>& hull, double radius)
  {
    return node_->inflate_hull(hull, radius);
  }

  void merge_overlapping()
  {
    node_->merge_overlapping();
  }

  bool point_in_polygon(double px, double py, const std::vector<std::pair<double, double>>& polygon)
  {
    return node_->point_in_polygon(px, py, polygon);
  }

  std::vector<std::vector<std::pair<double, double>>> dbscan(
      const std::vector<std::pair<double, double>>& points, double eps, int min_pts)
  {
    return node_->dbscan(points, eps, min_pts);
  }

  void associate_clusters(const std::vector<std::vector<std::pair<double, double>>>& clusters,
                          const rclcpp::Time& stamp)
  {
    node_->associate_clusters(clusters, stamp);
  }

  void set_keepout_mask(const nav_msgs::msg::OccupancyGrid& mask)
  {
    std::lock_guard<std::mutex> lock(node_->keepout_mutex_);
    node_->keepout_mask_ = mask;
    node_->have_keepout_mask_ = true;
  }

  // Expose private type for use in test bodies
  using TrackedObstacle = mowgli_map::ObstacleTrackerNode::TrackedObstacle;

  // Access tracked obstacles for merge tests
  std::vector<TrackedObstacle>& tracked()
  {
    return node_->tracked_;
  }

  uint32_t& next_id()
  {
    return node_->next_id_;
  }

  TrackedObstacle make_obstacle(double cx,
                                double cy,
                                double radius,
                                const std::vector<std::pair<double, double>>& hull)
  {
    TrackedObstacle obs;
    obs.id = node_->next_id_++;
    obs.cx = cx;
    obs.cy = cy;
    obs.radius = radius;
    obs.hull_points = hull;
    obs.first_seen = node_->now();
    obs.last_seen = node_->now();
    obs.observation_count = 5;
    obs.persistent = false;
    return obs;
  }

  std::shared_ptr<mowgli_map::ObstacleTrackerNode> node_;
};

// ─────────────────────────────────────────────────────────────────────────────
// boundary_hull tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(ObstacleTrackerAlgorithmTest, BoundaryHull_LShape)
{
  // L-shaped point cloud — boundary_hull should preserve concavity better
  // than convex hull
  std::vector<std::pair<double, double>> pts;
  // Bottom arm of L
  for (double x = 0; x <= 4.0; x += 0.1)
  {
    pts.push_back({x, 0.0});
    pts.push_back({x, 1.0});
  }
  // Vertical arm of L
  for (double y = 1.0; y <= 4.0; y += 0.1)
  {
    pts.push_back({0.0, y});
    pts.push_back({1.0, y});
  }

  auto hull = boundary_hull(pts);
  EXPECT_GE(hull.size(), 4u);

  // The hull should have more vertices than a convex hull of this L-shape
  auto ch = convex_hull(pts);
  // boundary_hull preserves concavity, so it often has more vertices
  EXPECT_GE(hull.size(), ch.size());
}

TEST_F(ObstacleTrackerAlgorithmTest, BoundaryHull_Circle)
{
  // Circular point cloud
  std::vector<std::pair<double, double>> pts;
  for (double a = 0; a < 2 * M_PI; a += 0.05)
  {
    pts.push_back({std::cos(a), std::sin(a)});
  }

  auto hull = boundary_hull(pts);
  EXPECT_GE(hull.size(), 10u);

  // All hull points should be roughly on the unit circle
  for (const auto& [x, y] : hull)
  {
    double r = std::hypot(x, y);
    EXPECT_NEAR(r, 1.0, 0.15);
  }
}

TEST_F(ObstacleTrackerAlgorithmTest, BoundaryHull_SmallCluster)
{
  // Less than 4 points: falls back to convex hull
  std::vector<std::pair<double, double>> pts = {{0, 0}, {1, 0}, {0.5, 1}};

  auto hull = boundary_hull(pts);
  auto ch = convex_hull(pts);

  EXPECT_EQ(hull.size(), ch.size());
}

// ─────────────────────────────────────────────────────────────────────────────
// convex_hull tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(ObstacleTrackerAlgorithmTest, ConvexHull_Triangle)
{
  std::vector<std::pair<double, double>> pts = {{0, 0}, {4, 0}, {2, 3}};
  auto hull = convex_hull(pts);
  EXPECT_EQ(hull.size(), 3u);
}

TEST_F(ObstacleTrackerAlgorithmTest, ConvexHull_WithInterior)
{
  // Square corners + interior points
  std::vector<std::pair<double, double>> pts = {
      {0, 0}, {10, 0}, {10, 10}, {0, 10}, {5, 5}, {3, 3}, {7, 2}};
  auto hull = convex_hull(pts);
  EXPECT_EQ(hull.size(), 4u);
}

TEST_F(ObstacleTrackerAlgorithmTest, ConvexHull_Degenerate)
{
  // 2 points
  std::vector<std::pair<double, double>> pts = {{0, 0}, {1, 1}};
  auto hull = convex_hull(pts);
  EXPECT_EQ(hull.size(), 2u);

  // 1 point
  pts = {{5, 5}};
  hull = convex_hull(pts);
  EXPECT_EQ(hull.size(), 1u);
}

// ─────────────────────────────────────────────────────────────────────────────
// inflate_hull tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(ObstacleTrackerAlgorithmTest, InflateHull_Square)
{
  std::vector<std::pair<double, double>> square = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  double radius = 0.5;

  auto inflated = inflate_hull(square, radius);
  EXPECT_EQ(inflated.size(), square.size());

  // Centroid is (1, 1). Each vertex should be pushed outward.
  // Original vertex (0,0) is at distance sqrt(2) from center.
  // Inflated vertex should be at distance sqrt(2) + 0.5.
  for (size_t i = 0; i < inflated.size(); ++i)
  {
    double orig_dist = std::hypot(square[i].first - 1.0, square[i].second - 1.0);
    double new_dist = std::hypot(inflated[i].first - 1.0, inflated[i].second - 1.0);
    EXPECT_NEAR(new_dist, orig_dist + radius, 1e-6);
  }
}

TEST_F(ObstacleTrackerAlgorithmTest, InflateHull_Empty)
{
  std::vector<std::pair<double, double>> empty;
  auto result = inflate_hull(empty, 1.0);
  EXPECT_TRUE(result.empty());
}

// ─────────────────────────────────────────────────────────────────────────────
// merge_overlapping tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(ObstacleTrackerAlgorithmTest, MergeOverlapping_TwoClose)
{
  // Two obstacles with overlapping radii -> should merge into one
  auto& t = tracked();

  auto obs1 = make_obstacle(1.0, 1.0, 0.5, {{0.5, 0.5}, {1.5, 0.5}, {1.5, 1.5}, {0.5, 1.5}});
  obs1.persistent = true;

  auto obs2 = make_obstacle(1.3, 1.0, 0.5, {{0.8, 0.5}, {1.8, 0.5}, {1.8, 1.5}, {0.8, 1.5}});

  t.push_back(obs1);
  t.push_back(obs2);

  merge_overlapping();

  // Should have merged into 1
  EXPECT_EQ(t.size(), 1u);
}

TEST_F(ObstacleTrackerAlgorithmTest, MergeOverlapping_TwoFar)
{
  // Two obstacles far apart -> should stay separate
  auto& t = tracked();

  auto obs1 = make_obstacle(0.0, 0.0, 0.3, {{-0.3, -0.3}, {0.3, -0.3}, {0.3, 0.3}, {-0.3, 0.3}});

  auto obs2 = make_obstacle(10.0, 10.0, 0.3, {{9.7, 9.7}, {10.3, 9.7}, {10.3, 10.3}, {9.7, 10.3}});

  t.push_back(obs1);
  t.push_back(obs2);

  merge_overlapping();

  // Should remain 2
  EXPECT_EQ(t.size(), 2u);
}

// ─────────────────────────────────────────────────────────────────────────────
// DBSCAN tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(ObstacleTrackerAlgorithmTest, DBSCAN_TwoClusters)
{
  std::vector<std::pair<double, double>> points = {// Cluster 1 around (0,0)
                                                   {0.0, 0.0},
                                                   {0.1, 0.0},
                                                   {0.0, 0.1},
                                                   {0.1, 0.1},
                                                   // Cluster 2 around (5,5)
                                                   {5.0, 5.0},
                                                   {5.1, 5.0},
                                                   {5.0, 5.1},
                                                   {5.1, 5.1}};

  auto clusters = dbscan(points, 0.5, 2);
  EXPECT_EQ(clusters.size(), 2u);
}

TEST_F(ObstacleTrackerAlgorithmTest, DBSCAN_AllNoise)
{
  // Points too far apart to form clusters
  std::vector<std::pair<double, double>> points = {{0.0, 0.0}, {10.0, 10.0}, {20.0, 20.0}};

  auto clusters = dbscan(points, 0.5, 2);
  EXPECT_EQ(clusters.size(), 0u);
}

// ─────────────────────────────────────────────────────────────────────────────
// point_in_polygon tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(ObstacleTrackerAlgorithmTest, PointInPolygon_Inside)
{
  std::vector<std::pair<double, double>> square = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  EXPECT_TRUE(point_in_polygon(5.0, 5.0, square));
}

TEST_F(ObstacleTrackerAlgorithmTest, PointInPolygon_Outside)
{
  std::vector<std::pair<double, double>> square = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  EXPECT_FALSE(point_in_polygon(-1.0, 5.0, square));
  EXPECT_FALSE(point_in_polygon(15.0, 5.0, square));
}

TEST_F(ObstacleTrackerAlgorithmTest, PointInPolygon_Degenerate)
{
  std::vector<std::pair<double, double>> line = {{0, 0}, {1, 0}};
  EXPECT_FALSE(point_in_polygon(0.5, 0.0, line));
}

// ─────────────────────────────────────────────────────────────────────────────
// Keepout-mask suppression (FIX A): a cluster whose centroid lands on a lethal
// keepout cell is a re-detection of an already-promoted obstacle and must be
// dropped; a cluster in free (mask=0) space is a genuine obstacle and tracked.
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(ObstacleTrackerAlgorithmTest, ClusterOnKeepoutCellIsDropped)
{
  // 100×100 cells @ 0.1 m, origin (-5, -5). Default free (0).
  nav_msgs::msg::OccupancyGrid mask;
  mask.info.resolution = 0.1F;
  mask.info.width = 100;
  mask.info.height = 100;
  mask.info.origin.position.x = -5.0;
  mask.info.origin.position.y = -5.0;
  mask.data.assign(static_cast<size_t>(100 * 100), 0);

  auto set_cell = [&](double x, double y, int8_t v)
  {
    const int col = static_cast<int>(std::floor((x - (-5.0)) / 0.1));
    const int row = static_cast<int>(std::floor((y - (-5.0)) / 0.1));
    mask.data[static_cast<size_t>(row) * 100 + col] = v;
  };

  // Mark a lethal patch (promoted keepout) around (2, 2).
  for (double x = 1.7; x <= 2.3; x += 0.05)
  {
    for (double y = 1.7; y <= 2.3; y += 0.05)
    {
      set_cell(x, y, 100);
    }
  }
  set_keepout_mask(mask);

  // Cluster centred on the lethal patch — this is the promoted keepout being
  // re-detected. It must be suppressed.
  const std::vector<std::pair<double, double>> promoted_blob = {
      {1.95, 1.95}, {2.05, 1.95}, {2.05, 2.05}, {1.95, 2.05}, {2.0, 2.0}};
  associate_clusters({promoted_blob}, node_->now());
  EXPECT_TRUE(tracked().empty()) << "cluster on a lethal keepout cell must be suppressed";

  // Cluster in free (mask=0) space — a genuine new obstacle. Must be tracked
  // even though a keepout mask has been received.
  const std::vector<std::pair<double, double>> fresh_blob = {
      {-2.05, -2.05}, {-1.95, -2.05}, {-1.95, -1.95}, {-2.05, -1.95}, {-2.0, -2.0}};
  associate_clusters({fresh_blob}, node_->now());
  EXPECT_EQ(tracked().size(), 1u) << "genuine obstacle in free (mask=0) area must be tracked";
}

TEST_F(ObstacleTrackerAlgorithmTest, NoKeepoutMaskFailsOpen)
{
  // Without a keepout mask the tracker must not suppress anything.
  const std::vector<std::pair<double, double>> blob = {
      {2.0, 2.0}, {2.1, 2.0}, {2.1, 2.1}, {2.0, 2.1}, {2.05, 2.05}};
  associate_clusters({blob}, node_->now());
  EXPECT_EQ(tracked().size(), 1u) << "no mask received → must fail open to detection";
}
