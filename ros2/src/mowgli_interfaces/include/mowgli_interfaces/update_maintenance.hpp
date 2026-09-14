#pragma once

#include <cstdlib>
#include <filesystem>

namespace mowgli_interfaces
{
// Operational maintenance inhibition only. Firmware remains the sole blade
// and emergency-stop safety authority. A configured but unreadable gate fails
// closed; hosts which have not installed the updater retain existing behavior.
inline bool updateMaintenanceActive()
{
  const char* path = std::getenv("MOWGLI_UPDATE_MAINTENANCE");
  if (path == nullptr || *path == '\0')
  {
    return false;
  }
  std::error_code error;
  const bool parent =
      std::filesystem::is_directory(std::filesystem::path(path).parent_path(), error);
  if (error || !parent)
  {
    return true;
  }
  const bool present = std::filesystem::exists(path, error);
  return present || static_cast<bool>(error);
}
}  // namespace mowgli_interfaces
