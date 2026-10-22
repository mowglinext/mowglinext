// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <atomic>
#include <barrier>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <future>
#include <sstream>

#include "mowgli_map/map_server_node.hpp"
#include <dlfcn.h>
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
bool matches(int fd, const std::string& target)
{
  char key[64], path[4096];
  snprintf(key, sizeof(key), "/proc/self/fd/%d", fd);
  auto n = syscall(SYS_readlink, key, path, sizeof(path) - 1);
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
  return syscall(SYS_rename, from, to);
}
extern "C" rcl_ret_t rcl_publish(const rcl_publisher_t* publisher,
                                 const void* msg,
                                 rmw_publisher_allocation_t* allocation)
{
  using Call = rcl_ret_t (*)(const rcl_publisher_t*, const void*, rmw_publisher_allocation_t*);
  static auto actual = reinterpret_cast<Call>(dlsym(RTLD_NEXT, "rcl_publish"));
  const char* topic = rcl_publisher_get_topic_name(publisher);
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
  static Corridor corridor(std::string name = "new", uint32_t id = 0, double width = .4)
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
  void add_area(std::string name, uint32_t id)
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
    EXPECT_FALSE(cleared.success);
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
    char tmp[] = "/tmp/corridor-transaction-XXXXXX";
    dir = mkdtemp(tmp);
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
  for (auto name : {std::string("bad\nname"),
                    std::string("bad\rname"),
                    std::string("bad\0name", 8),
                    std::string(" leading"),
                    std::string("trailing ")})
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
  EXPECT_FALSE(cleared);
  EXPECT_EQ(read(path), old);
  EXPECT_EQ(get()[0].name, "original");
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
  for (auto name : {std::string("bad\nname"),
                    std::string("bad\rname"),
                    std::string("bad\0name", 8),
                    std::string(" leading"),
                    std::string("trailing "),
                    std::string("\ttab")})
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
    auto cleared = legacy_clear();
    fault = 0;
    EXPECT_FALSE(added.success);
    EXPECT_EQ(added.id, 0u);
    EXPECT_FALSE(cleared.success);
    if (mode != 1)
    {
      EXPECT_GE(hits.load(), 2);
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
}  // namespace mowgli_map
