// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <atomic>
#include <barrier>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <future>
#include <sstream>

#include "mowgli_interfaces/wgs84_projection.hpp"
#include "mowgli_map/map_server_node.hpp"
#include <dlfcn.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <rcl/error_handling.h>
#include <sys/syscall.h>
#include <sys/uio.h>
#include <unistd.h>

namespace
{
std::atomic<int> fault{0}, hits{0};
std::atomic<int> directory_sync_failures{0}, publication_failures{0};
std::string temporary, directory;
std::mutex gate_mutex, publication_mutex;
std::condition_variable gate;
bool rename_entered = false, release_rename = false;
std::atomic<bool> record_publications{false};
std::vector<std::vector<std::string>> publications;
std::atomic<int> area_publications{0}, generation_publications{0}, mask_publications{0};
std::vector<size_t> area_sizes;  // sizes of the recorded-area lists that went out
std::vector<std::string> sent;  // order of area-list / corridor / generation sends
bool matches(int fd, const std::string& target)
{
  char key[64], path[4096];
  snprintf(key, sizeof(key), "/proc/self/fd/%d", fd);
  auto n = syscall(SYS_readlinkat, AT_FDCWD, key, path, sizeof(path) - 1);
  if (n < 0)
    return false;
  path[n] = 0;
  return path == target;
}
}  // namespace
// Link-local fault injection: only this test executable, only its fixture's .tmp file.
extern "C" ssize_t write(int fd, const void* data, size_t size)
{
  if (fault == 2 && matches(fd, temporary))
  {
    hits++;
    errno = ENOSPC;
    return -1;
  }
  return syscall(SYS_write, fd, data, size);
}
extern "C" ssize_t writev(int fd, const struct iovec* data, int count)
{
  if (fault == 2 && matches(fd, temporary))
  {
    hits++;
    errno = ENOSPC;
    return -1;
  }
  return syscall(SYS_writev, fd, data, count);
}
extern "C" int fsync(int fd)
{
  if ((fault == 3 && matches(fd, temporary)) ||
      ((fault == 5 || fault == 9) && matches(fd, directory)))
  {
    hits++;
    if (fault == 5 || fault == 9)
      directory_sync_failures++;
    errno = EIO;
    return -1;
  }
  return syscall(SYS_fsync, fd);
}
extern "C" int rename(const char* from, const char* to) noexcept
{
  if (fault == 4 && temporary == from)
  {
    hits++;
    errno = EIO;
    return -1;
  }
  if (fault == 8 && temporary == from)
  {
    std::unique_lock<std::mutex> lock(gate_mutex);
    rename_entered = true;
    gate.notify_all();
    gate.wait(lock,
              []
              {
                return release_rename;
              });
  }
  return syscall(SYS_renameat, AT_FDCWD, from, AT_FDCWD, to);
}
extern "C" rcl_ret_t rcl_publish(const rcl_publisher_t* publisher,
                                 const void* msg,
                                 rmw_publisher_allocation_t* allocation)
{
  using Call = rcl_ret_t (*)(const rcl_publisher_t*, const void*, rmw_publisher_allocation_t*);
  static auto actual = reinterpret_cast<Call>(dlsym(RTLD_NEXT, "rcl_publish"));
  const char* topic = rcl_publisher_get_topic_name(publisher);
  if (record_publications && topic)
  {
    if (std::string(topic) == "/mowgli/recorded_area_polygons")
    {
      area_publications++;
      const auto& data = *static_cast<const mowgli_interfaces::msg::RecordedAreaPolygonArray*>(msg);
      if (fault == 12 || (fault == 11 && !data.areas.empty()))
      {
        RCL_SET_ERROR_MSG("injected area list publication failure");
        return RCL_RET_ERROR;
      }
      area_sizes.push_back(data.areas.size());
      sent.push_back("areas:" + std::to_string(data.areas.size()));
    }
    if (std::string(topic) == "/map_server_node/area_list_generation")
    {
      generation_publications++;
      sent.push_back("generation");
    }
    if (std::string(topic) == "/keepout_mask")
    {
      if (fault == 10)
      {
        RCL_SET_ERROR_MSG("injected mask publication failure");
        return RCL_RET_ERROR;
      }
      mask_publications++;
    }
  }
  if (topic && std::string(topic) == "/mowgli/lidar_ignore_corridors")
  {
    if (fault == 6 || fault == 9)
    {
      hits++;
      publication_failures++;
      RCL_SET_ERROR_MSG("injected corridor publication failure");
      return RCL_RET_ERROR;
    }
    if (record_publications)
    {
      const auto& data = *static_cast<const mowgli_interfaces::msg::LidarIgnoreCorridorArray*>(msg);
      std::vector<std::string> names;
      for (const auto& c : data.corridors)
        names.push_back(c.name);
      std::lock_guard<std::mutex> lock(publication_mutex);
      publications.push_back(std::move(names));
      sent.push_back("corridors");
    }
  }
  return actual(publisher, msg, allocation);
}
namespace mowgli_map
{
using Replace = mowgli_interfaces::srv::ReplaceLidarIgnoreCorridors;
using Corridor = mowgli_interfaces::msg::LidarIgnoreCorridor;
using Add = mowgli_interfaces::srv::AddLidarIgnoreCorridor;
using Clear = mowgli_interfaces::srv::ClearLidarIgnoreCorridors;
using Get = mowgli_interfaces::srv::GetLidarIgnoreCorridors;
using namespace std::chrono_literals;
class CorridorTransactionTest : public ::testing::Test
{
protected:
  std::shared_ptr<MapServerNode> node;
  std::string dir, path, old;
  static geometry_msgs::msg::Polygon polygon(float x = 0, float y = 0)
  {
    geometry_msgs::msg::Polygon p;
    for (auto [a, b] :
         std::vector<std::pair<float, float>>{{x, y}, {x + 1, y}, {x + 1, y + 1}, {x, y + 1}})
    {
      geometry_msgs::msg::Point32 q;
      q.x = a;
      q.y = b;
      p.points.push_back(q);
    }
    return p;
  }
  static Corridor corridor(const std::string& name = "new", uint32_t id = 0, double width = .4)
  {
    Corridor c;
    c.name = name;
    c.id = id;
    c.width_m = width;
    c.polyline = polygon();
    c.polyline.points.resize(2);
    return c;
  }
  static std::string read(const std::string& p)
  {
    std::ifstream f(p);
    return {std::istreambuf_iterator<char>(f), {}};
  }
  static std::string non_corridor_data(const std::string& data)
  {
    std::istringstream in(data);
    std::string line, result;
    while (std::getline(in, line))
    {
      if (line.starts_with("lidar_corridor_") || line.starts_with("next_lidar_corridor_id:"))
        continue;
      if (!line.empty())
        result += line + "\n";
    }
    return result;
  }
  void add_area(const std::string& name, uint32_t id)
  {
    auto q = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Request>();
    q->area.name = name;
    q->area.area = polygon();
    q->area.id = id;
    q->area.has_mow_angle = true;
    q->area.mow_angle_deg = 37.;
    q->area.has_ring_direction = true;
    q->area.ring_direction = 1;
    q->area.has_start_point = true;
    q->area.start_x = .5;
    q->area.start_y = .5;
    auto r = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Response>();
    node->on_add_area(q, r);
    ASSERT_TRUE(r->success);
  }
  Replace::Response replace(std::vector<Corridor> list)
  {
    auto q = std::make_shared<Replace::Request>();
    q->corridors = std::move(list);
    q->client_operation_id = "test-operation";
    auto r = std::make_shared<Replace::Response>();
    node->on_replace_lidar_ignore_corridors(q, r);
    return *r;
  }
  std::vector<Corridor> get()
  {
    auto r = std::make_shared<Get::Response>();
    node->on_get_lidar_ignore_corridors(std::make_shared<Get::Request>(), r);
    EXPECT_TRUE(r->success);
    return r->corridors;
  }
  std::string area_name(size_t index)
  {
    return node->areas_.at(index).name;
  }
  uint64_t next()
  {
    return node->next_lidar_corridor_id_;
  }
  void set_path(const std::string& value)
  {
    node->areas_file_path_ = value;
  }
  Add::Response legacy_add(const Corridor& value)
  {
    auto q = std::make_shared<Add::Request>();
    q->corridor = value;
    auto response = std::make_shared<Add::Response>();
    node->on_add_lidar_ignore_corridor(q, response);
    return *response;
  }
  Clear::Response legacy_clear()
  {
    auto response = std::make_shared<Clear::Response>();
    node->on_clear_lidar_ignore_corridors(std::make_shared<Clear::Request>(), response);
    return *response;
  }
  void legacy_postcommit_failure(int mode)
  {
    fault = mode;
    auto added = legacy_add(corridor("legacy", 0));
    fault = 0;
    ASSERT_GT(hits.load(), 0);
    EXPECT_FALSE(added.success);
    EXPECT_EQ(added.id, 42u);
    auto snapshot = get();
    ASSERT_EQ(snapshot.size(), 2u);
    EXPECT_EQ(snapshot.back().id, 42u);
    auto added_file = read(path);
    EXPECT_NE(added_file, old);
    EXPECT_EQ(next(), 43u);
    if (mode == 6)
    {
      EXPECT_TRUE(publications.empty());
      retry_publication();
    }
    ASSERT_FALSE(publications.empty());
    EXPECT_EQ(publications.back(), std::vector<std::string>({"original", "legacy"}));
    EXPECT_EQ(read(path), added_file);  // publication recovery never writes again
    fault = mode;
    auto cleared = legacy_clear();
    fault = 0;
    // Clearing only un-blinds the filter: success once the empty list is live AND
    // published; a pending publication is still reported as not done.
    EXPECT_EQ(cleared.success, mode != 6);
    EXPECT_TRUE(get().empty());
    EXPECT_EQ(next(), 43u);
    auto cleared_file = read(path);
    EXPECT_NE(cleared_file, added_file);
    if (mode == 6)
      retry_publication();
    EXPECT_TRUE(publications.back().empty());
    EXPECT_EQ(read(path), cleared_file);
    reload();
    EXPECT_TRUE(get().empty());
    EXPECT_EQ(next(), 43u);
    EXPECT_EQ(non_corridor_data(read(path)), non_corridor_data(old));
  }
  std::pair<bool, bool> legacy_failure()
  {
    auto q = std::make_shared<Add::Request>();
    q->corridor = corridor();
    auto a = std::make_shared<Add::Response>();
    node->on_add_lidar_ignore_corridor(q, a);
    auto c = std::make_shared<Clear::Response>();
    node->on_clear_lidar_ignore_corridors(std::make_shared<Clear::Request>(), c);
    return {a->success, c->success};
  }
  void edit_other_map_data()
  {
    using Coverage = mowgli_interfaces::srv::SetAreaCoverageLines;
    auto q = std::make_shared<Coverage::Request>();
    q->id = 77;
    q->has_mow_angle = true;
    q->mow_angle_deg = 71.;
    q->has_ring_direction = true;
    q->ring_direction = 1;
    q->has_start_point = true;
    q->start_x = .5;
    q->start_y = .5;
    auto r = std::make_shared<Coverage::Response>();
    node->on_set_area_coverage_lines(q, r);
    ASSERT_TRUE(r->success);
    using Promote = mowgli_interfaces::srv::PromoteObstacle;
    auto obstacle = std::make_shared<Promote::Request>();
    obstacle->area_index = 0;
    obstacle->name = "concurrent-obstacle";
    obstacle->polygon = polygon();
    for (auto& point : obstacle->polygon.points)
    {
      point.x *= .1f;
      point.y *= .1f;
    }
    auto promoted = std::make_shared<Promote::Response>();
    node->on_promote_obstacle(obstacle, promoted);
    ASSERT_TRUE(promoted->success);
  }
  void reload()
  {
    node->load_areas_for_test(path);
  }
  void save()
  {
    node->save_areas_for_test(path);
  }
  void retry_publication()
  {
    node->on_publish_timer();
  }
  void set_next(uint64_t value)
  {
    node->next_lidar_corridor_id_ = value;
    save();
    old = read(path);
  }
  bool try_state_lock()
  {
    if (node->areas_state_mutex_.try_lock())
    {
      node->areas_state_mutex_.unlock();
      return true;
    }
    return false;
  }
  void SetUp() override
  {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    fault = 0;
    hits = 0;
    directory_sync_failures = 0;
    publication_failures = 0;
    record_publications = false;
    publications.clear();
    rename_entered = false;
    release_rename = false;
    auto pattern =
        (std::filesystem::temp_directory_path() / "corridor-transaction-XXXXXX").string();
    dir = mkdtemp(pattern.data());
    path = dir + "/areas.dat";
    temporary = path + ".tmp";
    directory = dir;
    auto opts = rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("map_size_x", 2.),
                                                           rclcpp::Parameter("map_size_y", 2.),
                                                           rclcpp::Parameter("resolution", .1),
                                                           rclcpp::Parameter("areas_file_path", ""),
                                                           rclcpp::Parameter("datum_lat", 48.),
                                                           rclcpp::Parameter("datum_lon", 2.)});
    node = std::make_shared<MapServerNode>(opts);
    add_area("kept-area", 77);
    ASSERT_TRUE(node->apply_promoted_obstacle_for_test(0, polygon(.2, .2), "kept-obstacle"));
    auto q = std::make_shared<Add::Request>();
    q->corridor = corridor("original", 41);
    auto r = std::make_shared<Add::Response>();
    node->on_add_lidar_ignore_corridor(q, r);
    ASSERT_TRUE(r->success);
    node->areas_file_path_ = path;
    save();
    old = read(path);
    ASSERT_NE(old.find("kept-obstacle"), std::string::npos);
    area_publications = 0;
    area_sizes.clear();
    sent.clear();
    generation_publications = 0;
    mask_publications = 0;
    record_publications = true;
  }
  void TearDown() override
  {
    fault = 0;
    record_publications = false;
    node.reset();
    std::filesystem::remove_all(dir);
  }
  void precommit_failure(int mode)
  {
    if (mode == 1)
    {
      ASSERT_TRUE(std::filesystem::create_directory(temporary));
    }
    fault = mode;
    auto r = replace({corridor()});
    fault = 0;
    if (mode != 1)
    {
      ASSERT_GT(hits.load(), 0);
    }
    EXPECT_FALSE(r.success);
    EXPECT_FALSE(r.committed);
    EXPECT_FALSE(r.persisted);
    EXPECT_FALSE(r.published);
    EXPECT_EQ(r.outcome, "PERSISTENCE_FAILED_BEFORE_COMMIT");
    EXPECT_EQ(read(path), old);
    EXPECT_EQ(get(), std::vector<Corridor>{corridor("original", 41)});
    EXPECT_EQ(next(), 42u);
    EXPECT_TRUE(publications.empty());
    reload();
    EXPECT_EQ(get(), std::vector<Corridor>{corridor("original", 41)});
    EXPECT_EQ(non_corridor_data(read(path)), non_corridor_data(old));
  }
};
TEST_F(CorridorTransactionTest, CreateFailure)
{
  precommit_failure(1);
}
TEST_F(CorridorTransactionTest, WriteFailure)
{
  precommit_failure(2);
}
TEST_F(CorridorTransactionTest, FileSyncFailure)
{
  precommit_failure(3);
}
TEST_F(CorridorTransactionTest, RenameFailure)
{
  precommit_failure(4);
}
TEST_F(CorridorTransactionTest, DirectorySyncFailureIsAlreadyCommitted)
{
  fault = 5;
  auto r = replace({corridor()});
  fault = 0;
  ASSERT_GT(hits.load(), 0);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.committed);
  EXPECT_FALSE(r.persisted);
  EXPECT_TRUE(r.published);
  EXPECT_EQ(r.outcome, "COMMITTED_DURABILITY_UNCONFIRMED");
  EXPECT_EQ(get(), r.corridors);
  EXPECT_NE(read(path), old);
  reload();
  EXPECT_EQ(get(), r.corridors);
  EXPECT_EQ(non_corridor_data(read(path)), non_corridor_data(old));
}
TEST_F(CorridorTransactionTest, PublicationFailureRetriesWithoutWritingAgain)
{
  fault = 6;
  auto r = replace({corridor()});
  fault = 0;
  ASSERT_GT(hits.load(), 0);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.committed);
  EXPECT_TRUE(r.persisted);
  EXPECT_FALSE(r.published);
  EXPECT_EQ(r.outcome, "COMMITTED_PUBLICATION_PENDING");
  auto committed = read(path);
  auto counter = next();
  retry_publication();
  ASSERT_EQ(publications.size(), 1u);
  EXPECT_EQ(publications[0], std::vector<std::string>{"new"});
  EXPECT_EQ(read(path), committed);
  EXPECT_EQ(next(), counter);
  reload();
  EXPECT_EQ(get(), r.corridors);
}
TEST_F(CorridorTransactionTest, WholeListValidationLeavesOldState)
{
  std::vector<Corridor> invalid;
  auto c = corridor();
  c.polyline.points.resize(1);
  invalid.push_back(c);
  c = corridor();
  c.polyline.points[1] = c.polyline.points[0];
  invalid.push_back(c);
  c = corridor();
  c.polyline.points[1].z = .001;
  invalid.push_back(c);
  c = corridor();
  c.polyline.points[1].z = std::numeric_limits<float>::epsilon();
  invalid.push_back(c);
  for (int axis = 0; axis < 3; axis++)
  {
    c = corridor();
    auto& p = c.polyline.points[0];
    (axis == 0 ? p.x : axis == 1 ? p.y : p.z) = std::numeric_limits<float>::infinity();
    invalid.push_back(c);
  }
  c = corridor();
  c.polyline.points[0].x = std::numeric_limits<float>::quiet_NaN();
  invalid.push_back(c);
  c = corridor();
  c.width_m = std::numeric_limits<double>::quiet_NaN();
  invalid.push_back(c);
  c = corridor();
  c.width_m = std::numeric_limits<double>::infinity();
  invalid.push_back(c);
  for (auto name :
       {std::string("bad\nname"), std::string("bad\rname"), std::string("bad\0name", 8)})
    invalid.push_back(corridor(name));
  for (const auto& bad : invalid)
  {
    auto r = replace({corridor("valid-first"), bad});
    EXPECT_FALSE(r.success);
    EXPECT_FALSE(r.committed);
    EXPECT_EQ(r.outcome, "VALIDATION_REJECTED");
    EXPECT_EQ(read(path), old);
    EXPECT_EQ(next(), 42u);
  }
  auto duplicate = replace({corridor("a", 77), corridor("b", 77)});
  EXPECT_EQ(duplicate.outcome, "VALIDATION_REJECTED");
  EXPECT_EQ(read(path), old);
  EXPECT_TRUE(publications.empty());
}
TEST_F(CorridorTransactionTest, StagingBudgetIsBoundedWithoutAPerLinePointCap)
{
  auto huge = corridor();
  huge.polyline.points.resize(
      Replace::Request::MAX_PAYLOAD_BYTES / sizeof(geometry_msgs::msg::Point32) + 1);
  auto r = replace({huge});
  EXPECT_EQ(r.outcome, "VALIDATION_REJECTED");
  EXPECT_EQ(read(path), old);
  r = replace(std::vector<Corridor>(Replace::Request::MAX_PAYLOAD_BYTES, corridor()));
  EXPECT_EQ(r.outcome, "VALIDATION_REJECTED");
  EXPECT_EQ(read(path), old);
  huge = corridor(std::string(Replace::Request::MAX_PAYLOAD_BYTES, 'a'));
  r = replace({huge});
  EXPECT_EQ(r.outcome, "VALIDATION_REJECTED");
  EXPECT_EQ(read(path), old);
  auto dense = corridor();
  dense.polyline.points.resize(200, dense.polyline.points.back());
  r = replace({dense});
  EXPECT_TRUE(r.success);
}
TEST_F(CorridorTransactionTest, IdsWidthsOrderAndFullMapSurviveFreshNode)
{
  auto r = replace({corridor("new-first", 0, 0),
                    corridor("existing", 42, 99),
                    corridor("high", UINT32_MAX - 1, .4)});
  ASSERT_TRUE(r.success);
  ASSERT_EQ(r.corridors.size(), 3u);
  EXPECT_EQ(r.corridors[0].id, UINT32_MAX);
  EXPECT_EQ(r.corridors[1].id, 42u);
  EXPECT_EQ(r.corridors[2].id, UINT32_MAX - 1);
  EXPECT_EQ(r.corridors[0].width_m, .05);
  EXPECT_EQ(r.corridors[1].width_m, 1.2);
  EXPECT_EQ(next(), uint64_t{UINT32_MAX} + 1);
  EXPECT_EQ(non_corridor_data(read(path)), non_corridor_data(old));
  auto opts = node->get_node_options();
  node.reset();
  opts.append_parameter_override("areas_file_path", path);
  node = std::make_shared<MapServerNode>(opts);
  EXPECT_EQ(get(), r.corridors);
  EXPECT_EQ(next(), uint64_t{UINT32_MAX} + 1);
  EXPECT_EQ(node->area_obstacle_count_for_test(0), 1u);
  auto failed = replace({corridor("exhausted")});
  EXPECT_EQ(failed.outcome, "VALIDATION_REJECTED");
  EXPECT_EQ(get(), r.corridors);
  auto clear = replace({});
  EXPECT_TRUE(clear.success);
  EXPECT_EQ(next(), uint64_t{UINT32_MAX} + 1);
  EXPECT_TRUE(get().empty());
}
TEST_F(CorridorTransactionTest, ReservedExplicitIdentityIsNeverReminted)
{
  auto r = replace({corridor("new", 0), corridor("existing", 42), corridor("low", 7)});
  ASSERT_TRUE(r.success);
  EXPECT_EQ(r.corridors[0].id, 43u);
  EXPECT_EQ(r.corridors[1].id, 42u);
  EXPECT_EQ(r.corridors[2].id, 7u);
  EXPECT_EQ(next(), 44u);
}
TEST_F(CorridorTransactionTest, MaximumIdAndExhaustionNeverWrap)
{
  auto r = replace({corridor("maximum", UINT32_MAX)});
  ASSERT_TRUE(r.success);
  EXPECT_EQ(next(), uint64_t{UINT32_MAX} + 1);
  reload();
  EXPECT_EQ(get(), r.corridors);
  EXPECT_EQ(next(), uint64_t{UINT32_MAX} + 1);
  auto old_file = read(path);
  auto bad = replace({corridor()});
  EXPECT_FALSE(bad.committed);
  EXPECT_EQ(read(path), old_file);
  EXPECT_EQ(get(), r.corridors);
}
TEST_F(CorridorTransactionTest, Finite2DCoordinatesAndWidthsRoundTripExactly)
{
  auto c = corridor("precision", 123, 0.123456789012345);
  c.polyline.points[0].x = 123456.789f;
  c.polyline.points[0].y = -0.123456789f;
  c.polyline.points[1].x = 123457.125f;
  c.polyline.points[1].y = -0.987654321f;
  auto r = replace({c});
  ASSERT_TRUE(r.success);
  reload();
  EXPECT_EQ(get(), r.corridors);
}
TEST_F(CorridorTransactionTest, NewCommitSupersedesPendingPublication)
{
  fault = 6;
  auto a = replace({corridor("old-pending")});
  fault = 0;
  ASSERT_TRUE(a.committed);
  ASSERT_FALSE(a.published);
  auto b = replace({corridor("latest")});
  ASSERT_TRUE(b.success);
  retry_publication();
  ASSERT_EQ(publications.size(), 1u);
  EXPECT_EQ(publications[0], std::vector<std::string>{"latest"});
}
TEST_F(CorridorTransactionTest, ConcurrentReplacementsNeverMixAndPublishInCommitOrder)
{
  std::barrier start(3);
  auto run = [&](std::string name)
  {
    start.arrive_and_wait();
    return replace({corridor(name)});
  };
  auto a = std::async(std::launch::async, run, "A");
  auto b = std::async(std::launch::async, run, "B");
  start.arrive_and_wait();
  ASSERT_EQ(a.wait_for(5s), std::future_status::ready);
  ASSERT_EQ(b.wait_for(5s), std::future_status::ready);
  auto ar = a.get(), br = b.get();
  ASSERT_TRUE(ar.success);
  ASSERT_TRUE(br.success);
  ASSERT_EQ(publications.size(), 2u);
  EXPECT_EQ(publications[0].size(), 1u);
  EXPECT_EQ(publications[1].size(), 1u);
  auto last = ar.revision > br.revision ? ar : br;
  EXPECT_EQ(get(), last.corridors);
  EXPECT_EQ(publications.back()[0], last.corridors[0].name);
  reload();
  EXPECT_EQ(get(), last.corridors);
}
TEST_F(CorridorTransactionTest, ConcurrentROSClientsUseTheAdditiveService)
{
  auto caller = std::make_shared<rclcpp::Node>("corridor_transaction_client");
  auto a = caller->create_client<Replace>("/map_server_node/replace_lidar_ignore_corridors");
  auto b = caller->create_client<Replace>("/map_server_node/replace_lidar_ignore_corridors");
  ASSERT_TRUE(a->wait_for_service(5s));
  ASSERT_TRUE(b->wait_for_service(5s));
  auto qa = std::make_shared<Replace::Request>();
  qa->corridors = {corridor("rpc-A")};
  qa->client_operation_id = "rpc-A";
  auto qb = std::make_shared<Replace::Request>();
  qb->corridors = {corridor("rpc-B")};
  qb->client_operation_id = "rpc-B";
  auto ar = a->async_send_request(qa);
  auto br = b->async_send_request(qb);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(caller);
  ASSERT_EQ(executor.spin_until_future_complete(ar, 5s), rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_EQ(executor.spin_until_future_complete(br, 5s), rclcpp::FutureReturnCode::SUCCESS);
  auto ra = ar.get(), rb = br.get();
  ASSERT_TRUE(ra->success);
  ASSERT_TRUE(rb->success);
  EXPECT_EQ(ra->client_operation_id, "rpc-A");
  EXPECT_EQ(rb->client_operation_id, "rpc-B");
  EXPECT_EQ(ra->server_instance_id, rb->server_instance_id);
  EXPECT_FALSE(ra->server_instance_id.empty());
  auto last = ra->revision > rb->revision ? ra : rb;
  EXPECT_EQ(get(), last->corridors);
  ASSERT_EQ(publications.size(), 2u);
  EXPECT_EQ(publications[0].size(), 1u);
  EXPECT_EQ(publications[1].size(), 1u);
  reload();
  EXPECT_EQ(get(), last->corridors);
}

TEST_F(CorridorTransactionTest, OtherMapWriterWaitsThenPreservesBothChanges)
{
  fault = 8;
  auto a = std::async(std::launch::async,
                      [&]
                      {
                        return replace({corridor()});
                      });
  {
    std::unique_lock<std::mutex> lock(gate_mutex);
    ASSERT_TRUE(gate.wait_for(lock,
                              5s,
                              []
                              {
                                return rename_entered;
                              }));
  }
  EXPECT_FALSE(try_state_lock());
  std::promise<void> attempted;
  auto ready = attempted.get_future();
  auto b = std::async(std::launch::async,
                      [&]
                      {
                        attempted.set_value();
                        add_area("concurrent-area", 88);
                        save();
                      });
  ready.wait();
  EXPECT_EQ(b.wait_for(0s), std::future_status::timeout);
  EXPECT_EQ(read(path), old);
  {
    std::lock_guard<std::mutex> lock(gate_mutex);
    release_rename = true;
    fault = 0;
  }
  gate.notify_all();
  ASSERT_EQ(a.wait_for(5s), std::future_status::ready);
  EXPECT_TRUE(a.get().success);
  ASSERT_EQ(b.wait_for(5s), std::future_status::ready);
  b.get();
  auto data = read(path);
  EXPECT_NE(data.find("kept-area"), std::string::npos);
  EXPECT_NE(data.find("concurrent-area"), std::string::npos);
  EXPECT_NE(data.find("kept-obstacle"), std::string::npos);
  EXPECT_NE(data.find("lidar_corridor_0_name: new"), std::string::npos);
  reload();
  EXPECT_EQ(get()[0].name, "new");
  EXPECT_EQ(node->area_obstacle_count_for_test(0), 1u);
}
TEST_F(CorridorTransactionTest, CoverageAndObstacleAutosavesPreserveBothTransactions)
{
  std::barrier start(3);
  auto a = std::async(std::launch::async,
                      [&]
                      {
                        start.arrive_and_wait();
                        return replace({corridor()});
                      });
  auto b = std::async(std::launch::async,
                      [&]
                      {
                        start.arrive_and_wait();
                        edit_other_map_data();
                      });
  start.arrive_and_wait();
  ASSERT_EQ(a.wait_for(5s), std::future_status::ready);
  EXPECT_TRUE(a.get().success);
  ASSERT_EQ(b.wait_for(5s), std::future_status::ready);
  b.get();
  auto data = read(path);
  EXPECT_NE(data.find("area_0_mow_angle_deg: 71"), std::string::npos);
  EXPECT_NE(data.find("concurrent-obstacle"), std::string::npos);
  EXPECT_NE(data.find("kept-obstacle"), std::string::npos);
  reload();
  ASSERT_EQ(get().size(), 1u);
  EXPECT_EQ(get()[0].name, "new");
  EXPECT_EQ(node->area_obstacle_count_for_test(0), 2u);
}

TEST_F(CorridorTransactionTest, LoadRestampAndSaveReenterWithoutDeadlock)
{
  auto text = old;
  for (auto key : {"datum_lat:", "datum_lon:"})
  {
    auto start = text.find(key);
    text.erase(start, text.find('\n', start) - start + 1);
  }
  std::ofstream(path) << text;
  auto run = std::async(std::launch::async,
                        [&]
                        {
                          reload();
                          save();
                          return replace({corridor()});
                        });
  ASSERT_EQ(run.wait_for(5s), std::future_status::ready);
  EXPECT_TRUE(run.get().success);
  EXPECT_NE(read(path).find("datum_lat: 48."), std::string::npos);
}
TEST_F(CorridorTransactionTest, EmptyPathAndAntiOverwriteGuardRefuseBeforeCommit)
{
  set_path("");
  auto r = replace({corridor()});
  EXPECT_EQ(r.outcome, "PERSISTENCE_NOT_CONFIGURED");
  EXPECT_FALSE(r.committed);
  EXPECT_EQ(read(path), old);
  set_path(path);
  auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
  node->clear_map_for_test(response);
  r = replace({corridor()});
  EXPECT_EQ(r.outcome, "PERSISTENCE_FAILED_BEFORE_COMMIT");
  EXPECT_EQ(read(path), old);
  EXPECT_EQ(get()[0].name, "original");
}
TEST_F(CorridorTransactionTest, LegacyServicesKeepTypesAndReportDiskFailure)
{
  fault = 4;
  auto [added, cleared] = legacy_failure();
  fault = 0;
  EXPECT_FALSE(added);
  // A clear is a pure removal: it applies in memory even though the write failed.
  EXPECT_TRUE(cleared);
  EXPECT_EQ(read(path), old);
  EXPECT_TRUE(get().empty());
  EXPECT_EQ(next(), 42u);
  // Preserve the old writer's directory-sync policy for unrelated callers.
  fault = 5;
  save();
  fault = 0;
  EXPECT_GT(hits.load(), 0);
}
TEST_F(CorridorTransactionTest, LegacyAddRejectsInvalid2DGeometryAndNamesWithoutEffects)
{
  std::vector<Corridor> invalid;
  for (float z : {.001f,
                  -.001f,
                  std::numeric_limits<float>::epsilon(),
                  std::numeric_limits<float>::infinity(),
                  std::numeric_limits<float>::quiet_NaN()})
  {
    auto c = corridor();
    c.polyline.points[1].z = z;
    invalid.push_back(c);
  }
  for (auto name :
       {std::string("bad\nname"), std::string("bad\rname"), std::string("bad\0name", 8)})
    invalid.push_back(corridor(name));
  auto c = corridor();
  c.polyline.points.resize(1);
  invalid.push_back(c);
  c = corridor();
  c.polyline.points[1] = c.polyline.points[0];
  invalid.push_back(c);
  c = corridor();
  c.polyline.points[0].x = std::numeric_limits<float>::quiet_NaN();
  invalid.push_back(c);
  c = corridor();
  c.width_m = std::numeric_limits<double>::infinity();
  invalid.push_back(c);
  for (size_t i = 0; i < invalid.size(); ++i)
  {
    SCOPED_TRACE(i);
    auto r = legacy_add(invalid[i]);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.id, 0u);
    EXPECT_EQ(get(), std::vector<Corridor>{corridor("original", 41)});
    EXPECT_EQ(next(), 42u);
    EXPECT_EQ(read(path), old);
    EXPECT_TRUE(publications.empty());
  }
}
TEST_F(CorridorTransactionTest, LegacyMemoryOnlyPreservesNamesClampIdentityOrderAndCounter)
{
  set_path("");
  fault = 4;  // no write must be attempted even when Rename would fail
  auto duplicate = legacy_add(corridor("carte: été; [#]", 41, 0));
  ASSERT_TRUE(duplicate.success);
  EXPECT_EQ(duplicate.id, 42u);
  auto new_id = legacy_add(corridor("", 0, 99));
  ASSERT_TRUE(new_id.success);
  EXPECT_EQ(new_id.id, 43u);
  auto low = legacy_add(corridor("legacy-low", 7));
  ASSERT_TRUE(low.success);
  EXPECT_EQ(low.id, 7u);
  auto snapshot = get();
  ASSERT_EQ(snapshot.size(), 4u);
  EXPECT_EQ(snapshot[0], corridor("original", 41));
  EXPECT_EQ(snapshot[1], corridor("carte: été; [#]", 42, .05));
  EXPECT_EQ(snapshot[2], corridor("", 43, 1.2));
  EXPECT_EQ(snapshot[3], corridor("legacy-low", 7));
  EXPECT_EQ(next(), 44u);
  EXPECT_TRUE(legacy_clear().success);
  EXPECT_TRUE(get().empty());
  EXPECT_TRUE(legacy_clear().success);
  EXPECT_EQ(next(), 44u);
  auto after_clear = legacy_add(corridor("after-clear"));
  EXPECT_TRUE(after_clear.success);
  EXPECT_EQ(after_clear.id, 44u);
  EXPECT_EQ(get(), std::vector<Corridor>{corridor("after-clear", 44)});
  EXPECT_EQ(next(), 45u);
  fault = 0;
  EXPECT_EQ(hits.load(), 0);
  EXPECT_EQ(read(path), old);
  ASSERT_EQ(publications.size(), 6u);
  EXPECT_EQ(publications.back(), std::vector<std::string>{"after-clear"});
}
TEST_F(CorridorTransactionTest, LegacyHighIdsDuplicateRemintAndExhaustionRoundTrip)
{
  auto high = legacy_add(corridor("high", UINT32_MAX - 1));
  ASSERT_TRUE(high.success);
  EXPECT_EQ(high.id, UINT32_MAX - 1);
  auto duplicate = legacy_add(corridor("reminted", UINT32_MAX - 1));
  ASSERT_TRUE(duplicate.success);
  EXPECT_EQ(duplicate.id, UINT32_MAX);
  EXPECT_EQ(next(), uint64_t{UINT32_MAX} + 1);
  auto file = read(path);
  auto snapshot = get();
  auto count = publications.size();
  for (auto id : {uint32_t{0}, UINT32_MAX - 1, UINT32_MAX})
  {
    auto failed = legacy_add(corridor("exhausted", id));
    EXPECT_FALSE(failed.success);
    EXPECT_EQ(failed.id, 0u);
    EXPECT_EQ(read(path), file);
    EXPECT_EQ(get(), snapshot);
    EXPECT_EQ(publications.size(), count);
  }
  auto explicit_id = legacy_add(corridor("unused-explicit", 7));
  ASSERT_TRUE(explicit_id.success);
  EXPECT_EQ(explicit_id.id, 7u);
  snapshot = get();
  reload();
  EXPECT_EQ(get(), snapshot);
  EXPECT_EQ(next(), uint64_t{UINT32_MAX} + 1);
  EXPECT_EQ(non_corridor_data(read(path)), non_corridor_data(old));
  ASSERT_TRUE(legacy_clear().success);
  EXPECT_TRUE(get().empty());
  EXPECT_EQ(next(), uint64_t{UINT32_MAX} + 1);
  EXPECT_FALSE(legacy_add(corridor("still-exhausted")).success);
}
TEST_F(CorridorTransactionTest, LegacyPrecommitFailuresPreserveDiskMemoryAndCounter)
{
  for (int mode : {1, 2, 3, 4})
  {
    SCOPED_TRACE(mode);
    hits = 0;
    if (mode == 1)
    {
      ASSERT_TRUE(std::filesystem::create_directory(temporary));
    }
    fault = mode;
    auto added = legacy_add(corridor());
    fault = 0;
    EXPECT_FALSE(added.success);
    EXPECT_EQ(added.id, 0u);
    if (mode != 1)
    {
      EXPECT_GE(hits.load(), 1);
    }
    EXPECT_EQ(read(path), old);
    EXPECT_EQ(get(), std::vector<Corridor>{corridor("original", 41)});
    EXPECT_EQ(next(), 42u);
    EXPECT_TRUE(publications.empty());
    if (mode == 1)
    {
      EXPECT_TRUE(std::filesystem::remove(temporary));
    }
  }
}
TEST_F(CorridorTransactionTest, LegacyDirectorySyncFailureIsNotARollback)
{
  legacy_postcommit_failure(5);
}
TEST_F(CorridorTransactionTest, LegacyPublicationFailureIsNotARollback)
{
  legacy_postcommit_failure(6);
}
TEST_F(CorridorTransactionTest, CombinedDirectorySyncAndPublicationFailuresPreserveBothFacts)
{
  fault = 9;
  auto r = replace({corridor("committed-both-failed")});
  fault = 0;
  ASSERT_EQ(directory_sync_failures.load(), 1);
  ASSERT_EQ(publication_failures.load(), 1);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.committed);
  EXPECT_FALSE(r.persisted);
  EXPECT_FALSE(r.published);
  EXPECT_EQ(r.outcome, "COMMITTED_DURABILITY_UNCONFIRMED");
  EXPECT_NE(r.message.find("replacement committed"), std::string::npos);
  EXPECT_NE(r.message.find("durability unconfirmed"), std::string::npos);
  EXPECT_EQ(r.message.find("rollback"), std::string::npos);
  auto snapshot = get();
  EXPECT_EQ(snapshot, r.corridors);
  ASSERT_EQ(snapshot.size(), 1u);
  EXPECT_EQ(snapshot[0].name, "committed-both-failed");
  EXPECT_EQ(snapshot[0].id, 42u);
  EXPECT_EQ(next(), 43u);
  auto committed_file = read(path);
  EXPECT_NE(committed_file, old);
  EXPECT_TRUE(publications.empty());
  retry_publication();
  ASSERT_EQ(publications.size(), 1u);
  EXPECT_EQ(publications[0], std::vector<std::string>{"committed-both-failed"});
  EXPECT_EQ(read(path), committed_file);
  EXPECT_EQ(next(), 43u);
  EXPECT_EQ(directory_sync_failures.load(), 1);
  EXPECT_EQ(publication_failures.load(), 1);
  EXPECT_FALSE(r.persisted);
  EXPECT_FALSE(r.published);  // original response is not upgraded by retry
  reload();
  EXPECT_EQ(get(), snapshot);
  EXPECT_EQ(read(path), committed_file);
  EXPECT_EQ(non_corridor_data(committed_file), non_corridor_data(old));
}
// ── Removal must survive a failed areas.dat write (full SD card) ────────────
// A corridor blinds collision_monitor along its line, so the disk must never be
// able to keep one alive: clear, or a replace that only drops / narrows lines,
// applies in memory and is published; the file is caught up by the next save.
TEST_F(CorridorTransactionTest, RemovalCommitsInMemoryAndPublishesWhenTheWriteFails)
{
  for (int mode : {1, 2, 3, 4})
  {
    SCOPED_TRACE(mode);
    // Two lines on disk and in memory.
    ASSERT_TRUE(replace({corridor("original", 41), corridor("second", 50)}).success);
    old = read(path);
    publications.clear();
    hits = 0;
    if (mode == 1)
    {
      ASSERT_TRUE(std::filesystem::create_directory(temporary));
    }
    fault = mode;
    auto r = replace({corridor("original", 41)});  // drop "second"
    fault = 0;
    if (mode != 1)
    {
      ASSERT_GT(hits.load(), 0);
    }
    EXPECT_FALSE(r.success);  // not COMMITTED: durability is not confirmed
    EXPECT_TRUE(r.committed);
    EXPECT_FALSE(r.persisted);
    EXPECT_TRUE(r.published);
    EXPECT_EQ(r.outcome, "COMMITTED_DURABILITY_UNCONFIRMED");
    EXPECT_NE(r.message.find("removal committed in memory"), std::string::npos);
    EXPECT_EQ(get(), std::vector<Corridor>{corridor("original", 41)});
    EXPECT_EQ(r.corridors, get());
    ASSERT_EQ(publications.size(), 1u);
    EXPECT_EQ(publications.back(), std::vector<std::string>{"original"});
    EXPECT_EQ(read(path), old);  // the failed write left the file untouched
    EXPECT_EQ(next(), 51u);
    if (mode == 1)
    {
      EXPECT_TRUE(std::filesystem::remove(temporary));
    }
  }
}
TEST_F(CorridorTransactionTest, ClearCommitsInMemoryAndPublishesWhenTheWriteFails)
{
  fault = 2;  // ENOSPC on the temp file
  auto cleared = legacy_clear();
  fault = 0;
  ASSERT_GT(hits.load(), 0);
  EXPECT_TRUE(cleared.success);
  EXPECT_TRUE(get().empty());
  ASSERT_EQ(publications.size(), 1u);
  EXPECT_TRUE(publications.back().empty());
  EXPECT_EQ(read(path), old);
  EXPECT_EQ(next(), 42u);
  fault = 2;
  auto r = replace({});  // the replace service's clear behaves the same
  fault = 0;
  EXPECT_TRUE(r.committed);
  EXPECT_TRUE(r.published);
  EXPECT_FALSE(r.persisted);
  EXPECT_EQ(r.outcome, "COMMITTED_DURABILITY_UNCONFIRMED");
}
TEST_F(CorridorTransactionTest, NarrowingIsARemovalButWideningReshapingAndAddingAreNot)
{
  auto narrower = corridor("original", 41, .2);
  fault = 2;
  auto r = replace({narrower});
  fault = 0;
  EXPECT_TRUE(r.committed);
  EXPECT_EQ(get(), std::vector<Corridor>{narrower});
  EXPECT_EQ(read(path), old);
  publications.clear();
  auto wider = corridor("original", 41, .6);
  auto reshaped = narrower;
  reshaped.polyline.points[1].x = 5.f;
  for (const auto& expanding : std::vector<std::vector<Corridor>>{
           {wider}, {reshaped}, {narrower, corridor("new", 0)}, {corridor("original", 0, .2)}})
  {
    fault = 2;
    auto refused = replace(expanding);
    fault = 0;
    EXPECT_FALSE(refused.committed);
    EXPECT_EQ(refused.outcome, "PERSISTENCE_FAILED_BEFORE_COMMIT");
    EXPECT_EQ(get(), std::vector<Corridor>{narrower});
    EXPECT_TRUE(publications.empty());
  }
  fault = 2;
  EXPECT_FALSE(legacy_add(corridor("added")).success);
  fault = 0;
  EXPECT_EQ(get(), std::vector<Corridor>{narrower});
  EXPECT_TRUE(publications.empty());
}
TEST_F(CorridorTransactionTest, NextSuccessfulWritePersistsAnInMemoryRemoval)
{
  ASSERT_TRUE(replace({corridor("original", 41), corridor("second", 50)}).success);
  fault = 2;
  ASSERT_TRUE(replace({corridor("second", 50)}).committed);
  fault = 0;
  // Any later writer serializes the live list — here an unrelated map save.
  save();
  reload();
  EXPECT_EQ(get(), std::vector<Corridor>{corridor("second", 50)});
  EXPECT_EQ(next(), 51u);
  // ...and the next corridor transaction too.
  fault = 2;
  ASSERT_TRUE(legacy_clear().success);
  fault = 0;
  ASSERT_TRUE(replace({corridor("fresh")}).success);
  reload();
  EXPECT_EQ(get(), std::vector<Corridor>{corridor("fresh", 51)});
}

// ── Names: normalised, not rejected ─────────────────────────────────────────
TEST_F(CorridorTransactionTest, SurroundingWhitespaceInNamesIsTrimmedOnAddAndReplace)
{
  auto added = legacy_add(corridor(" \tpadded add \r\n", 60));
  ASSERT_TRUE(added.success);
  EXPECT_EQ(get().back().name, "padded add");
  auto r = replace({corridor("  original ", 41), corridor("trailing ", 60), corridor(" ", 0)});
  ASSERT_TRUE(r.success) << r.message;
  ASSERT_EQ(r.corridors.size(), 3u);
  EXPECT_EQ(r.corridors[0].name, "original");
  EXPECT_EQ(r.corridors[1].name, "trailing");
  EXPECT_EQ(r.corridors[2].name, "");
  reload();
  EXPECT_EQ(get(), r.corridors);
  // Genuinely invalid names are still rejected, with the reason in the message.
  auto bad = replace({corridor("bad\nname")});
  EXPECT_EQ(bad.outcome, "VALIDATION_REJECTED");
  EXPECT_NE(bad.message.find("line break"), std::string::npos);
}
TEST_F(CorridorTransactionTest, LegacyNameWithTrailingWhitespaceLoadsTrimmedAndReplaces)
{
  auto data = read(path);
  const std::string key = "lidar_corridor_0_name: original\n";
  auto at = data.find(key);
  ASSERT_NE(at, std::string::npos);
  data.replace(at, key.size(), "lidar_corridor_0_name: original  \t\n");
  std::ofstream(path, std::ios::trunc) << data;
  reload();
  EXPECT_EQ(get(), std::vector<Corridor>{corridor("original", 41)});
  // The GUI sends back exactly what it read: the whole-list replace must pass.
  auto r = replace(get());
  EXPECT_TRUE(r.success) << r.message;
}

// ── areas.dat tolerance: CRLF and surrounding blanks ────────────────────────
TEST_F(CorridorTransactionTest, CrlfAndPaddedAreasFileLoadsCompletely)
{
  ASSERT_TRUE(replace({corridor("original", 41), corridor("second", 50)}).success);
  auto lf = read(path);
  std::string crlf;
  std::istringstream in(lf);
  std::string line;
  while (std::getline(in, line))
  {
    // Trailing blanks after the strictly parsed unsigned corridor values
    // (lidar_corridor_N_id, next_lidar_corridor_id), plus Windows line endings.
    if (line.find("corridor_") != std::string::npos && line.find("_id: ") != std::string::npos)
      line += "  ";
    crlf += line + "\r\n";
  }
  std::ofstream(path, std::ios::trunc | std::ios::binary) << crlf;
  reload();
  EXPECT_EQ(get(), std::vector<Corridor>({corridor("original", 41), corridor("second", 50)}));
  EXPECT_EQ(next(), 51u);
  EXPECT_EQ(node->area_obstacle_count_for_test(0), 1u);
  // Round-trips byte-identically to the LF form once re-saved.
  save();
  EXPECT_EQ(read(path), lf);
}
TEST_F(CorridorTransactionTest, MalformedCorridorIdFailsTheLoadBeforeTouchingLiveState)
{
  auto data = read(path);
  const std::string key = "lidar_corridor_0_id: 41\n";
  auto at = data.find(key);
  ASSERT_NE(at, std::string::npos);
  data.replace(at, key.size(), "lidar_corridor_0_id: 41x\n");
  // A different area set, so a half-applied load would be visible.
  const std::string area = "area_0_name: kept-area\n";
  at = data.find(area);
  ASSERT_NE(at, std::string::npos);
  data.replace(at, area.size(), "area_0_name: replaced-area\n");
  std::ofstream(path, std::ios::trunc) << data;
  EXPECT_THROW(reload(), std::runtime_error);
  EXPECT_EQ(get(), std::vector<Corridor>{corridor("original", 41)});
  EXPECT_EQ(next(), 42u);
  EXPECT_EQ(node->area_obstacle_count_for_test(0), 1u);
  EXPECT_EQ(area_name(0), "kept-area");
}

// Restore uses the same link-local write/fsync/rename/publication fault seam.
class RestoreTransactionTest : public CorridorTransactionTest
{
protected:
  using Restore = mowgli_interfaces::srv::RestoreAreas;
  grid_map::GridMap original_grid, original_progress;
  uint32_t original_next_obstacle{0};
  void SetUp() override
  {
    CorridorTransactionTest::SetUp();
    original_grid = node->map_;
    original_progress = node->mow_progress_map_;
    original_next_obstacle = node->next_obstacle_id_;
  }

  Restore::Response restore(const std::string& content)
  {
    auto req = std::make_shared<Restore::Request>();
    req->areas_dat = content;
    auto res = std::make_shared<Restore::Response>();
    node->on_restore_areas(req, res);
    return *res;
  }
  static std::string replacement()
  {
    return "datum_lat: 48.000000000\ndatum_lon: 2.000000000\n"
           "area_count: 2\nnext_area_id: 102\n"
           "area_0_name: navigation\narea_0_id: 100\narea_0_is_navigation: 1\n"
           "area_0_polygon: 4,4;8,4;8,8;4,8\narea_0_obstacle_count: 0\n"
           "area_1_name: replacement lawn\narea_1_id: 101\n"
           "area_1_polygon: 10,0;14,0;14,4;10,4\n"
           "area_1_obstacle_count: 1\narea_1_obstacle_0: 11,1;12,1;12,2;11,2\n"
           "area_1_obstacle_0_name: replacement obstacle\narea_1_obstacle_0_source: 2\n"
           "area_1_mow_angle_deg: 35\narea_1_ring_direction: 2\n"
           "area_1_start_x: 10.5\narea_1_start_y: 0.5\n"
           "lidar_corridor_count: 1\nnext_lidar_corridor_id: 202\n"
           "lidar_corridor_0_name: replacement line\nlidar_corridor_0_id: 201\n"
           "lidar_corridor_0_polyline: 4,4;8,4\nlidar_corridor_0_width_m: 0.5\n";
  }
  void keep_dock()
  {
    node->docking_pose_set_ = true;
    node->docking_pose_.position.x = 2;
    node->docking_pose_.position.y = 3;
    node->docking_pose_.orientation.w = 1;
    node->rebuild_dock_polygons();
  }
  void expect_original(uint64_t generation)
  {
    EXPECT_EQ(read(path), old);
    EXPECT_EQ(node->areas_.size(), 1u);
    EXPECT_EQ(area_name(0), "kept-area");
    EXPECT_EQ(node->areas_[0].id, 77u);
    EXPECT_EQ(node->obstacle_polygons_.size(), 1u);
    EXPECT_EQ(get(), std::vector<Corridor>{corridor("original", 41)});
    EXPECT_EQ(node->area_list_generation_, generation);
    EXPECT_EQ(node->next_area_id_, 78u);
    EXPECT_EQ(node->next_obstacle_id_, original_next_obstacle);
    EXPECT_EQ(node->next_lidar_corridor_id_, 42u);
    EXPECT_TRUE((node->map_.getSize() == original_grid.getSize()).all());
    for (const auto& layer : original_grid.getLayers())
    {
      EXPECT_TRUE(node->map_[layer].isApprox(original_grid[layer]));
    }
    EXPECT_TRUE(node->mow_progress_map_["mowed"].isApprox(original_progress["mowed"]));

    EXPECT_TRUE(publications.empty());
    EXPECT_EQ(area_publications, 0);
    EXPECT_EQ(generation_publications, 0);
    EXPECT_EQ(mask_publications, 0);
  }
  void expect_replaced(uint64_t generation)
  {
    ASSERT_EQ(node->areas_.size(), 2u);
    EXPECT_EQ(area_name(0), "navigation");
    EXPECT_TRUE(node->areas_[0].is_navigation_area);
    EXPECT_EQ(node->areas_[1].id, 101u);
    EXPECT_EQ(node->next_area_id_, 102u);
    EXPECT_EQ(node->next_lidar_corridor_id_, 202u);
    ASSERT_EQ(node->areas_[1].obstacles.size(), 1u);
    EXPECT_EQ(node->areas_[1].obstacles[0].name, "replacement obstacle");
    EXPECT_EQ(node->areas_[1].obstacles[0].source, 2u);
    EXPECT_EQ(node->obstacle_polygons_.size(), 1u);
    EXPECT_EQ(node->areas_[1].coverage_lines.mow_angle_deg, 35);
    EXPECT_EQ(node->areas_[1].coverage_lines.ring_direction, 2);
    EXPECT_TRUE(node->areas_[1].coverage_lines.has_start_point);
    EXPECT_EQ(node->area_list_generation_, generation + 1);
    EXPECT_EQ(get()[0].name, "replacement line");
  }
  void expect_fresh_grids()
  {
    EXPECT_TRUE((node->mow_progress_map_.getSize() == node->map_.getSize()).all());
    EXPECT_TRUE(node->mow_progress_map_["mowed"].isZero());
    EXPECT_FALSE(node->have_last_mow_tool_position_);
    EXPECT_FALSE(node->mow_progress_cache_valid_);
    EXPECT_FALSE(node->classification_dirty_);
    EXPECT_EQ(node->map_.atPosition(std::string(layers::CLASSIFICATION),
                                    grid_map::Position(.5, .5)),
              defaults::CLASSIFICATION);

    EXPECT_EQ(node->map_.atPosition(std::string(layers::CLASSIFICATION),
                                    grid_map::Position(10.5, .5)),
              static_cast<float>(CellType::LAWN));
    EXPECT_EQ(node->map_.atPosition(std::string(layers::CLASSIFICATION),
                                    grid_map::Position(11.5, 1.5)),
              static_cast<float>(CellType::NO_GO_ZONE));
  }

  void run_AllRuntimeLoadsRefuseFreshAutonomousWithoutEffects()
  {
    auto generation = node->area_list_generation_for_test();
    node->high_level_state_for_test(
        mowgli_interfaces::msg::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS);
    auto loaded = node->load_areas_service_for_test();
    EXPECT_FALSE(loaded.success);
    EXPECT_NE(loaded.message.find("mission"), std::string::npos);
    auto binary = std::make_shared<std_srvs::srv::Trigger::Response>();
    node->load_map_for_test(binary);
    EXPECT_FALSE(binary->success);
    EXPECT_NE(binary->message.find("mission"), std::string::npos);
    auto restored = restore(replacement());
    EXPECT_EQ(restored.outcome, "REFUSED_MISSION");
    EXPECT_FALSE(restored.committed);
    EXPECT_FALSE(restored.persisted);
    EXPECT_FALSE(restored.published);
    expect_original(generation);
  }
  void run_AcceptedRestoreReplacesCompleteStateOnceAndKeepsDock()
  {
    keep_dock();
    auto dock = node->docking_pose_;
    auto generation = node->area_list_generation_for_test();
    node->stamp_mow_progress_for_test(.5, .5);
    node->high_level_state_for_test(mowgli_interfaces::msg::HighLevelStatus::HIGH_LEVEL_STATE_IDLE);
    auto r = restore(replacement());
    ASSERT_TRUE(r.success) << r.message;
    EXPECT_TRUE(r.committed);
    EXPECT_TRUE(r.persisted);
    EXPECT_TRUE(r.published);
    EXPECT_EQ(r.outcome, "COMMITTED");
    expect_replaced(generation);
    expect_fresh_grids();
    EXPECT_EQ(node->docking_pose_, dock);
    ASSERT_EQ(publications.size(), 1u);
    EXPECT_EQ(publications[0], std::vector<std::string>{"replacement line"});
    EXPECT_EQ(area_publications, 1);
    EXPECT_EQ(generation_publications, 1);
    EXPECT_TRUE(node->masks_dirty_);
    EXPECT_GT(node->mask_rebuild_not_before_, std::chrono::steady_clock::now());
    EXPECT_NE(read(path).find("replacement lawn"), std::string::npos);
  }
  void run_InvalidContentAndResourceLimitsPreserveStateAndFile()
  {
    auto generation = node->area_list_generation_for_test();
    for (auto text : {std::string("bad"),
                      std::string("area_count: -1\n"),
                      std::string("area_count: 2garbage\n"),
                      std::string("area_count: 1\narea_0_polygon: 0,0;nan,0;0,1\n"),
                      std::string("area_count: 1\narea_0_polygon: 0,0;100000,0;0,100000\n"),
                      replacement() + "datum_lat: 48\n",
                      std::string(4 * 1024 * 1024 + 1, 'x')})
    {
      auto r = restore(text);
      EXPECT_EQ(r.outcome, "INVALID_MAP") << r.message;
      EXPECT_FALSE(r.committed);
      expect_original(generation);
    }
  }
  void run_PreRenameFaultsPreserveStateFileAndPublications()
  {
    auto generation = node->area_list_generation_for_test();
    for (int mode : {1, 2, 3, 4})
    {
      SCOPED_TRACE(mode);
      hits = 0;
      if (mode == 1)
      {
        ASSERT_TRUE(std::filesystem::create_directory(temporary));
      }
      fault = mode;
      auto r = restore(replacement());
      fault = 0;
      EXPECT_EQ(r.outcome, "PERSISTENCE_FAILED_BEFORE_COMMIT") << r.message;
      EXPECT_FALSE(r.committed);
      if (mode != 1)
      {
        EXPECT_GT(hits.load(), 0);
      }
      expect_original(generation);
      if (mode == 1)
      {
        EXPECT_TRUE(std::filesystem::remove(temporary));
      }
    }
  }
  void run_PostRenameDirectorySyncFailureStillInstallsAndPublishes()
  {
    auto generation = node->area_list_generation_for_test();
    fault = 5;
    auto r = restore(replacement());
    fault = 0;
    EXPECT_EQ(r.outcome, "COMMITTED_DURABILITY_UNCONFIRMED");
    EXPECT_TRUE(r.committed);
    EXPECT_FALSE(r.persisted);
    EXPECT_TRUE(r.published);
    EXPECT_FALSE(r.success);
    expect_replaced(generation);
    auto committed = read(path);
    EXPECT_NE(committed, old);
    EXPECT_EQ(publications.size(), 1u);
    reload();
    EXPECT_EQ(read(path), committed);
    EXPECT_EQ(area_name(1), "replacement lawn");
  }
  void run_PostCommitPublicationRetriesWithoutRewriteOrGenerationBump()
  {
    auto generation = node->area_list_generation_for_test();
    node->publish_recorded_area_list();  // the filter knows the old areas
    area_publications = 0;
    fault = 6;
    auto r = restore(replacement());
    fault = 0;
    EXPECT_EQ(r.outcome, "COMMITTED_PUBLICATION_PENDING");
    EXPECT_TRUE(r.committed);
    EXPECT_TRUE(r.persisted);
    EXPECT_FALSE(r.published);
    EXPECT_FALSE(r.success);
    expect_replaced(generation);
    auto committed = read(path);
    EXPECT_TRUE(publications.empty());
    // The restored areas are not filtered with the old corridors: empty list.
    EXPECT_EQ(area_publications, 1);
    EXPECT_TRUE(node->area_list_published_empty_);
    retry_publication();
    EXPECT_EQ(publications.size(), 1u);
    EXPECT_EQ(area_publications, 2);
    EXPECT_FALSE(node->area_list_published_empty_);
    EXPECT_EQ(generation_publications, 1);
    EXPECT_EQ(node->area_list_generation_for_test(), generation + 1);
    EXPECT_EQ(read(path), committed);
  }
  void run_OldDatumReprojectsAllGeometryButKeepsCurrentDock()
  {
    keep_dock();
    auto dock = node->docking_pose_;
    auto content = replacement();
    content.replace(content.find("48.000000000"), 12, "48.000010000");
    auto r = restore(content);
    ASSERT_TRUE(r.success) << r.message;
    double x = 10, y = 0;
    mowgli_interfaces::wgs84::ReprojectEnu(48.00001, 2, 48, 2, x, y);
    EXPECT_NEAR(node->areas_[1].polygon.points[0].x, x, 1e-5);
    EXPECT_NEAR(node->areas_[1].polygon.points[0].y, y, 1e-5);
    x = 11;
    y = 1;
    mowgli_interfaces::wgs84::ReprojectEnu(48.00001, 2, 48, 2, x, y);
    EXPECT_NEAR(node->obstacle_polygons_[0].points[0].y, y, 1e-5);
    x = 4;
    y = 4;
    mowgli_interfaces::wgs84::ReprojectEnu(48.00001, 2, 48, 2, x, y);
    EXPECT_NEAR(get()[0].polyline.points[0].y, y, 1e-5);
    x = 10.5;
    y = .5;
    mowgli_interfaces::wgs84::ReprojectEnu(48.00001, 2, 48, 2, x, y);
    EXPECT_NEAR(node->areas_[1].coverage_lines.start_y, y, 1e-5);
    EXPECT_EQ(node->docking_pose_, dock);
    auto committed = read(path);
    reload();
    EXPECT_EQ(read(path), committed);
    // Start points use the existing millimetre precision on disk.
    EXPECT_DOUBLE_EQ(node->areas_[1].coverage_lines.start_y, std::round(y * 1000) / 1000);
  }
  void run_LatePublicationRetryKeepsTheRebuiltMask()
  {
    fault = 6;
    auto r = restore(replacement());
    ASSERT_EQ(r.outcome, "COMMITTED_PUBLICATION_PENDING");
    node->mask_rebuild_not_before_ = {};
    retry_publication();  // still failing; the settled mask rebuild runs
    ASSERT_GT(node->cached_keepout_mask_.info.width, 0u);
    ASSERT_FALSE(node->masks_dirty_);
    fault = 0;
    retry_publication();
    EXPECT_FALSE(node->restore_publication_pending_);
    EXPECT_GT(node->cached_keepout_mask_.info.width, 0u);
    retry_publication();
    EXPECT_GT(node->cached_keepout_mask_.info.width, 0u);
  }

  void run_PendingRestoreLeavesALaterAreaEditOpen()
  {
    fault = 6;
    auto r = restore(replacement());
    ASSERT_EQ(r.outcome, "COMMITTED_PUBLICATION_PENDING");
    fault = 0;
    auto cleared = std::make_shared<std_srvs::srv::Trigger::Response>();
    node->clear_map_for_test(cleared);
    ASSERT_TRUE(cleared->success);
    add_area("first area of an unfinished edit", 400);
    ASSERT_TRUE(node->area_replace_in_progress_);
    ASSERT_TRUE(node->area_list_published_empty_);
    const auto before = area_publications.load();
    retry_publication();
    EXPECT_FALSE(node->restore_publication_pending_);
    // The edit is not committed yet, so the corridor filter stays off.
    EXPECT_TRUE(node->area_replace_in_progress_);
    EXPECT_TRUE(node->area_list_published_empty_);
    EXPECT_EQ(area_publications.load(), before);
  }

  void run_SaveWhileRestorePendingKeepsTheFilterOff()
  {
    fault = 6;
    ASSERT_EQ(restore(replacement()).outcome, "COMMITTED_PUBLICATION_PENDING");
    ASSERT_TRUE(node->area_list_published_empty_);
    auto saved = std::make_shared<std_srvs::srv::Trigger::Response>();
    node->save_areas_service_for_test(saved);
    ASSERT_TRUE(saved->success);
    // The restored areas must not meet the pre-restore corridors.
    EXPECT_TRUE(node->area_list_published_empty_);
    EXPECT_TRUE(area_sizes.empty() || area_sizes.back() == 0u);
    EXPECT_TRUE(node->restore_publication_pending_);
    fault = 0;
    retry_publication();
    EXPECT_FALSE(node->restore_publication_pending_);
    ASSERT_FALSE(area_sizes.empty());
    EXPECT_EQ(area_sizes.back(), 2u);
  }

  void run_MaskPublicationFailureAfterCommitStaysInside()
  {
    fault = 10;
    Restore::Response r;
    EXPECT_NO_THROW(r = restore(replacement()));
    fault = 0;
    EXPECT_TRUE(r.committed);
    EXPECT_EQ(node->areas_.size(), 2u);
    EXPECT_EQ(publications.size(), 1u);
    ASSERT_FALSE(area_sizes.empty());
    EXPECT_EQ(area_sizes.back(), 2u);
    // The rebuild scheduled at install publishes the new mask.
    node->mask_rebuild_not_before_ = {};
    retry_publication();
    EXPECT_GT(node->cached_keepout_mask_.info.width, 0u);
  }

  void run_AreaListFailureDuringRetryStaysPending()
  {
    fault = 6;
    ASSERT_EQ(restore(replacement()).outcome, "COMMITTED_PUBLICATION_PENDING");
    fault = 11;
    EXPECT_NO_THROW(retry_publication());
    EXPECT_TRUE(node->restore_publication_pending_);
    EXPECT_TRUE(node->recorded_areas_publication_pending_);
    EXPECT_TRUE(node->area_list_published_empty_);
    fault = 0;
    retry_publication();
    EXPECT_FALSE(node->restore_publication_pending_);
    EXPECT_FALSE(node->recorded_areas_publication_pending_);
    ASSERT_FALSE(area_sizes.empty());
    EXPECT_EQ(area_sizes.back(), 2u);
    const auto count = area_publications.load();
    retry_publication();
    EXPECT_EQ(area_publications.load(), count);
  }

  void run_RestoreNeverPairsGeometryFromTwoMaps()
  {
    node->publish_recorded_area_list();  // the filter knows the old map
    sent.clear();
    ASSERT_TRUE(restore(replacement()).success);
    // Filter off, restored corridors, then the restored list.
    EXPECT_EQ(sent, (std::vector<std::string>{"areas:0", "corridors", "generation", "areas:2"}));
  }

  void run_MaskFailureOnTheScheduledRebuildIsContained()
  {
    fault = 10;
    ASSERT_TRUE(restore(replacement()).committed);
    node->mask_rebuild_not_before_ = {};
    EXPECT_NO_THROW(retry_publication());
    EXPECT_TRUE(node->masks_dirty_);
    fault = 0;
    retry_publication();
    EXPECT_FALSE(node->masks_dirty_);
    EXPECT_GT(node->cached_keepout_mask_.info.width, 0u);
  }

  void run_ReloadDuringAPendingRestoreKeepsTheList()
  {
    fault = 6;
    ASSERT_EQ(restore(replacement()).outcome, "COMMITTED_PUBLICATION_PENDING");
    fault = 0;
    auto loaded = node->load_areas_service_for_test();
    ASSERT_TRUE(loaded.success) << loaded.message;
    ASSERT_FALSE(node->recorded_areas_publication_pending_);
    ASSERT_TRUE(node->restore_publication_pending_);
    fault = 6;
    retry_publication();  // blanks the list, corridors fail again
    EXPECT_TRUE(node->recorded_areas_publication_pending_);
    fault = 0;
    retry_publication();
    EXPECT_FALSE(node->restore_publication_pending_);
    EXPECT_FALSE(node->area_list_published_empty_);
    ASSERT_FALSE(area_sizes.empty());
    EXPECT_EQ(area_sizes.back(), 2u);
  }

  void run_PendingRestoreHoldsEveryCorridorPublication()
  {
    node->publish_recorded_area_list();  // the filter knows the old map
    sent.clear();
    fault = 6;
    ASSERT_EQ(replace({corridor("original", 41)}).outcome, "COMMITTED_PUBLICATION_PENDING");
    fault = 12;  // the restore cannot switch the filter off
    ASSERT_EQ(restore(replacement()).outcome, "COMMITTED_PUBLICATION_PENDING");
    ASSERT_FALSE(node->area_list_published_empty_);
    // Neither the older corridor retry nor a new corridor edit may send the
    // restored corridors while the old areas are live.
    EXPECT_NO_THROW(retry_publication());
    ASSERT_TRUE(replace({corridor("latest", 301)}).committed);
    EXPECT_TRUE(publications.empty());
    fault = 0;
    retry_publication();
    EXPECT_FALSE(node->restore_publication_pending_);
    EXPECT_FALSE(node->corridor_publication_pending_);
    ASSERT_EQ(publications.size(), 1u);
    EXPECT_EQ(publications.back(), (std::vector<std::string>{"latest"}));
    EXPECT_EQ(sent, (std::vector<std::string>{"areas:0", "corridors", "generation", "areas:2"}));
  }

  void run_CorridorClearStillGoesOutDuringAPendingRestore()
  {
    node->publish_recorded_area_list();  // the filter knows the old map
    fault = 12;  // the restore cannot switch the filter off
    ASSERT_EQ(restore(replacement()).outcome, "COMMITTED_PUBLICATION_PENDING");
    ASSERT_TRUE(publications.empty());
    // Removing every line suppresses nothing, whatever areas it meets.
    auto r = replace({});
    EXPECT_EQ(r.outcome, "COMMITTED");
    ASSERT_EQ(publications.size(), 1u);
    EXPECT_TRUE(publications.back().empty());
    EXPECT_FALSE(node->corridor_publication_pending_);
    EXPECT_TRUE(node->restore_publication_pending_);
  }

  void run_BootLoadReadsCommittedFileAndKeepsGenerationAtZero()
  {
    ASSERT_TRUE(restore(replacement()).success);
    auto committed = read(path);
    node.reset();
    auto opts =
        rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("map_size_x", 2.),
                                                   rclcpp::Parameter("map_size_y", 2.),
                                                   rclcpp::Parameter("resolution", .1),
                                                   rclcpp::Parameter("areas_file_path", path),
                                                   rclcpp::Parameter("datum_lat", 48.),
                                                   rclcpp::Parameter("datum_lon", 2.)});
    node = std::make_shared<MapServerNode>(opts);
    ASSERT_EQ(node->areas_.size(), 2u);
    EXPECT_EQ(area_name(1), "replacement lawn");
    EXPECT_EQ(get()[0].name, "replacement line");
    EXPECT_EQ(node->area_list_generation_for_test(), 0u);
    EXPECT_EQ(read(path), committed);
  }
};

TEST_F(RestoreTransactionTest, AllRuntimeLoadsRefuseFreshAutonomousWithoutEffects)
{
  run_AllRuntimeLoadsRefuseFreshAutonomousWithoutEffects();
}

TEST_F(RestoreTransactionTest, AcceptedRestoreReplacesCompleteStateOnceAndKeepsDock)
{
  run_AcceptedRestoreReplacesCompleteStateOnceAndKeepsDock();
}

TEST_F(RestoreTransactionTest, InvalidContentAndResourceLimitsPreserveStateAndFile)
{
  run_InvalidContentAndResourceLimitsPreserveStateAndFile();
}

TEST_F(RestoreTransactionTest, PreRenameFaultsPreserveStateFileAndPublications)
{
  run_PreRenameFaultsPreserveStateFileAndPublications();
}

TEST_F(RestoreTransactionTest, PostRenameDirectorySyncFailureStillInstallsAndPublishes)
{
  run_PostRenameDirectorySyncFailureStillInstallsAndPublishes();
}

TEST_F(RestoreTransactionTest, PostCommitPublicationRetriesWithoutRewriteOrGenerationBump)
{
  run_PostCommitPublicationRetriesWithoutRewriteOrGenerationBump();
}

TEST_F(RestoreTransactionTest, OldDatumReprojectsAllGeometryButKeepsCurrentDock)
{
  run_OldDatumReprojectsAllGeometryButKeepsCurrentDock();
}

TEST_F(RestoreTransactionTest, BootLoadReadsCommittedFileAndKeepsGenerationAtZero)
{
  run_BootLoadReadsCommittedFileAndKeepsGenerationAtZero();
}

TEST_F(RestoreTransactionTest, LatePublicationRetryKeepsTheRebuiltMask)
{
  run_LatePublicationRetryKeepsTheRebuiltMask();
}

TEST_F(RestoreTransactionTest, PendingRestoreLeavesALaterAreaEditOpen)
{
  run_PendingRestoreLeavesALaterAreaEditOpen();
}

TEST_F(RestoreTransactionTest, SaveWhileRestorePendingKeepsTheFilterOff)
{
  run_SaveWhileRestorePendingKeepsTheFilterOff();
}

TEST_F(RestoreTransactionTest, MaskPublicationFailureAfterCommitStaysInside)
{
  run_MaskPublicationFailureAfterCommitStaysInside();
}

TEST_F(RestoreTransactionTest, AreaListFailureDuringRetryStaysPending)
{
  run_AreaListFailureDuringRetryStaysPending();
}

TEST_F(RestoreTransactionTest, RestoreNeverPairsGeometryFromTwoMaps)
{
  run_RestoreNeverPairsGeometryFromTwoMaps();
}

TEST_F(RestoreTransactionTest, MaskFailureOnTheScheduledRebuildIsContained)
{
  run_MaskFailureOnTheScheduledRebuildIsContained();
}

TEST_F(RestoreTransactionTest, ReloadDuringAPendingRestoreKeepsTheList)
{
  run_ReloadDuringAPendingRestoreKeepsTheList();
}

TEST_F(RestoreTransactionTest, PendingRestoreHoldsEveryCorridorPublication)
{
  run_PendingRestoreHoldsEveryCorridorPublication();
}

TEST_F(RestoreTransactionTest, CorridorClearStillGoesOutDuringAPendingRestore)
{
  run_CorridorClearStillGoesOutDuringAPendingRestore();
}

}  // namespace mowgli_map
