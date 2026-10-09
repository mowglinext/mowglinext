// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

#include "mowgli_interfaces/motion_authorization.hpp"
#include <gtest/gtest.h>

namespace
{
using namespace mowgli_interfaces::motion;
using Message = visualization_msgs::msg::MarkerArray;
const Ring kBody{{0.53, 0.275}, {0.53, -0.275}, {-0.17, -0.275}, {-0.17, 0.275}};

void Rectangle(Message& message,
               double x0,
               double y0,
               double x1,
               double y1,
               const std::string& kind = "area",
               const std::string& purpose = "mowing")
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = kind;
  marker.text = kind == "area" ? purpose : "";
  marker.type = marker.LINE_STRIP;
  marker.action = marker.ADD;
  marker.pose.orientation.w = 1;
  for (const auto& p : Ring{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}})
  {
    geometry_msgs::msg::Point point;
    point.x = p.x;
    point.y = p.y;
    marker.points.push_back(point);
  }
  message.markers.push_back(marker);
}

Message Lawn()
{
  Message message;
  Rectangle(message, -10, -10, 10, 10);
  return message;
}

TEST(MotionSweeps, EdgeOverhangAndFullRotationRemainPermitted)
{
  Message message;
  Rectangle(message, 0, 0, 4, 4);
  const auto geometry = Snapshot::parse(message);
  // Axle on recorded perimeter; large parts of asymmetric body overhang it.
  EXPECT_TRUE(geometry->permits({0, 0, 0}, kBody, 0.2, 0, 1, true));
  EXPECT_TRUE(geometry->permits({0, 0, 0}, kBody, 0, 1, 2 * M_PI, true));
  EXPECT_FALSE(geometry->permits({0, 0, 0}, kBody, -0.1, 0, 1, true));
}

TEST(MotionSweeps, FullChassisCatchesSubmillimetreHoleBetweenEndpointBodies)
{
  auto message = Lawn();
  Rectangle(message, 0.60, 0.20, 0.6001, 0.2001, "obstacle");
  const auto geometry = Snapshot::parse(message);
  EXPECT_TRUE(geometry->permits({0, 0, 0}, kBody, 0.1, 0, 0.5, true));
  EXPECT_FALSE(geometry->permits({0, 0, 0}, kBody, 0.8, 0, 1, true));
  EXPECT_FALSE(geometry->permits({0.8, 0, 0}, kBody, -0.8, 0, 1, true));
}

TEST(MotionSweeps, RotationRejectsHoleAbsentFromBothEndpointBodies)
{
  auto message = Lawn();
  Rectangle(message, 0.30, 0.35, 0.40, 0.45, "obstacle");
  const auto geometry = Snapshot::parse(message);
  EXPECT_TRUE(geometry->permits({0, 0, 0}, kBody, 0, 0, 0, true));
  EXPECT_TRUE(geometry->permits({0, 0, M_PI / 2}, kBody, 0, 0, 0, true));
  EXPECT_FALSE(geometry->permits({0, 0, 0}, kBody, 0, 1, M_PI / 2, true));
  EXPECT_FALSE(geometry->permits({0, 0, M_PI / 2}, kBody, 0, -1, M_PI / 2, true));
}

TEST(MotionSweeps, CurvedAxleCannotCutConcaveCorner)
{
  Message message;
  Rectangle(message, -1, -1, 0.3, 2);
  Rectangle(message, 0.3, 0.7, 2, 2);
  const auto geometry = Snapshot::parse(message);
  EXPECT_TRUE(geometry->permits({0, 0, 0}, kBody, 0, 1, M_PI / 2, true));
  EXPECT_FALSE(geometry->permits({0, 0, 0}, kBody, 1, 1, M_PI / 2, true));
}

TEST(MotionSweeps, TrueGapRejectsButSharedNarrowSeamWorks)
{
  Message message;
  Rectangle(message, -1, -1, 0, 1);
  Rectangle(message, 0, -0.05, 1, 0.05);
  Rectangle(message, 1, -1, 2, 1);
  EXPECT_TRUE(Snapshot::parse(message)->permits({-0.5, 0, 0}, kBody, 1, 0, 2, false));
  message.markers[1].points[0].x = 0.0001;
  message.markers[1].points[3].x = 0.0001;
  EXPECT_FALSE(Snapshot::parse(message)->permits({-0.5, 0, 0}, kBody, 1, 0, 2, false));
}

TEST(MotionSweeps, NavigationAndDockCorridorCannotGrantCoveragePermission)
{
  Message message;
  Rectangle(message, -1, -1, 1, 1, "area", "navigation");
  Rectangle(message, 1, -0.5, 3, 0.5, "dock_corridor");
  const auto geometry = Snapshot::parse(message);
  EXPECT_TRUE(geometry->permits({0, 0, 0}, kBody, 0.2, 0, 1, false));
  EXPECT_FALSE(geometry->permits({0, 0, 0}, kBody, 0.2, 0, 1, true));
  EXPECT_TRUE(geometry->permits({1.5, 0, 0}, kBody, -0.2, 0, 1, false));
}

TEST(MotionSweeps, IdentityIgnoresCachedDeliveryStampButTracksPurposeAndHole)
{
  auto message = Lawn();
  const auto original = Snapshot::parse(message);
  message.markers[0].header.stamp.sec = 12345;
  EXPECT_EQ(original->identity, Snapshot::parse(message)->identity);
  message.markers[0].text = "navigation";
  EXPECT_NE(original->identity, Snapshot::parse(message)->identity);
  Rectangle(message, 1, 1, 2, 2, "obstacle");
  EXPECT_NE(original->identity, Snapshot::parse(message)->identity);
  EXPECT_FALSE(Snapshot::parse(Message{})->valid);
}

TEST(MotionSweeps, MalformedFrameRingPoseAndCommandReject)
{
  auto message = Lawn();
  message.markers[0].header.frame_id = "odom";
  EXPECT_FALSE(Snapshot::parse(message)->valid);
  message = Lawn();
  std::swap(message.markers[0].points[1], message.markers[0].points[2]);
  EXPECT_FALSE(Snapshot::parse(message)->valid);
  const auto geometry = Snapshot::parse(Lawn());
  EXPECT_FALSE(geometry->permits({NAN, 0, 0}, kBody, 0.2, 0, 1, true));
  EXPECT_FALSE(geometry->permits({0, 0, 0}, kBody, NAN, 0, 1, true));
  EXPECT_FALSE(geometry->permits({0, 0, 0}, {}, 0.2, 0, 1, true));
  EXPECT_TRUE(geometry->permits({0, 0, M_PI}, kBody, -0.2, -1e-12, 1, true));
}

TEST(MotionSweeps, RepeatedKinematicCommandsMakeProgressWithLegalOverhang)
{
  Message message;
  Rectangle(message, 0, 0, 20, 20);
  const auto geometry = Snapshot::parse(message);
  const auto start = std::chrono::steady_clock::now();
  Pose pose{0, 0, 0};
  for (int i = 0; i < 1000; ++i)
  {
    ASSERT_TRUE(geometry->permits(pose, kBody, 0.1, 0, 0.1, true));
    pose.x += 0.01;
  }
  EXPECT_NEAR(pose.x, 10, 1e-9);
  const auto elapsed =
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
  std::cout << "kinematic sweep checks=1000 mean_us=" << elapsed / 1000 << '\n';
}
}  // namespace
