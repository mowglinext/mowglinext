#include <cassert>
#include <cstdlib>
#include <filesystem>
#include <fstream>

#include "mowgli_interfaces/update_maintenance.hpp"

int main()
{
  unsetenv("MOWGLI_UPDATE_MAINTENANCE");
  assert(!mowgli_interfaces::updateMaintenanceActive());
  auto directory = std::filesystem::temp_directory_path() / "mowgli-maintenance-test-XXXXXX";
  auto name = directory.string();
  assert(mkdtemp(name.data()) != nullptr);
  directory = name;
  const auto marker = directory / "maintenance";
  setenv("MOWGLI_UPDATE_MAINTENANCE", marker.c_str(), 1);
  assert(!mowgli_interfaces::updateMaintenanceActive());
  std::ofstream(marker) << "pending";
  assert(mowgli_interfaces::updateMaintenanceActive());
  std::filesystem::remove(marker);
  assert(!mowgli_interfaces::updateMaintenanceActive());
  std::filesystem::remove(directory);
  assert(mowgli_interfaces::updateMaintenanceActive());
  unsetenv("MOWGLI_UPDATE_MAINTENANCE");
}
