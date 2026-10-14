// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "fusion_graph/checkpoint_file.hpp"

#include <cerrno>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <vector>

#include <boost/crc.hpp>
#include <fcntl.h>
#include <unistd.h>

namespace fusion_graph::checkpoint
{
namespace
{
constexpr const char* kMagic = "MOWGLI_CHECKPOINT_V1";

class TemporaryFile
{
public:
  explicit TemporaryFile(const std::string& pattern)
  {
    std::vector<char> name(pattern.begin(), pattern.end());
    name.push_back('\0');
    fd = ::mkstemp(name.data());
    if (fd >= 0)
      path = name.data();
  }
  ~TemporaryFile()
  {
    if (fd >= 0)
      ::close(fd);
    if (!path.empty())
      ::unlink(path.c_str());
  }
  int fd = -1;
  std::string path;
};

class Directory
{
public:
  explicit Directory(const std::string& path)
      : fd(::open(path.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC))
  {
  }
  ~Directory()
  {
    if (fd >= 0)
      ::close(fd);
  }
  bool Close()
  {
    const int descriptor = fd;
    fd = -1;
    return ::close(descriptor) == 0;
  }
  int fd;
};

unsigned int Checksum(const std::string& metadata, const std::string& graph)
{
  boost::crc_32_type crc;
  crc.process_bytes(metadata.data(), metadata.size());
  crc.process_bytes(graph.data(), graph.size());
  return crc.checksum();
}

std::string Parent(const std::string& prefix)
{
  const auto parent = std::filesystem::path(prefix).parent_path();
  return parent.empty() ? "." : parent.string();
}
}  // namespace

bool Read(const std::string& path, Payload& payload)
try
{
  std::ifstream in(path, std::ios::binary);
  if (!in)
    return false;
  std::string magic, sizes;
  if (!std::getline(in, magic) || magic != kMagic || !std::getline(in, sizes))
    return false;
  size_t metadata_size = 0, graph_size = 0;
  unsigned int checksum = 0;
  std::istringstream header(sizes);
  if (!(header >> metadata_size >> graph_size >> checksum))
    return false;
  header >> std::ws;
  if (!header.eof())
    return false;
  const std::string bytes{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
  if (in.bad() || metadata_size > bytes.size() || graph_size != bytes.size() - metadata_size)
    return false;
  Payload candidate{bytes.substr(metadata_size), bytes.substr(0, metadata_size)};
  if (Checksum(candidate.metadata, candidate.graph) != checksum)
    return false;
  payload = std::move(candidate);
  return true;
}
catch (const std::exception&)
{
  return false;
}

bool Save(const std::string& prefix, const Payload& payload, bool backup_current)
try
{
  const std::string current = prefix + ".checkpoint";
  const std::string previous = current + ".previous";
  std::ostringstream header;
  header << kMagic << '\n'
         << payload.metadata.size() << ' ' << payload.graph.size() << ' '
         << Checksum(payload.metadata, payload.graph) << '\n';
  const auto framing = header.str();
  TemporaryFile temporary(current + ".tmp.XXXXXX");
  if (temporary.fd < 0)
    return false;
  FILE* stream = ::fdopen(temporary.fd, "wb");
  if (!stream)
    return false;
  temporary.fd = -1;  // fclose owns the descriptor, including on failure.
  bool ok = ::fwrite(framing.data(), 1, framing.size(), stream) == framing.size() &&
            ::fwrite(payload.metadata.data(), 1, payload.metadata.size(), stream) ==
                payload.metadata.size() &&
            ::fwrite(payload.graph.data(), 1, payload.graph.size(), stream) == payload.graph.size();
  // Do not rely on stream destruction: flush, file sync AND close can each fail.
  if (::fflush(stream) != 0 || ::ferror(stream))
    ok = false;
  if (ok && ::fsync(::fileno(stream)) != 0)
    ok = false;
  if (::fclose(stream) != 0)
    ok = false;
  if (!ok)
    return false;

  Directory directory(Parent(prefix));
  if (directory.fd < 0)
    return false;

  // Secure the previous inode before the only publication rename. Each name
  // holds a whole generation; these are NOT separate graph/metadata renames.
  // Never overwrite a good recovery file with a corrupt current file.
  Payload old;
  if (backup_current && Read(current, old))
  {
    TemporaryFile backup(previous + ".tmp.XXXXXX");
    ok = backup.fd >= 0;
    if (ok)
    {
      const int fd = backup.fd;
      backup.fd = -1;
      ok = ::close(fd) == 0 && ::unlink(backup.path.c_str()) == 0 &&
           ::link(current.c_str(), backup.path.c_str()) == 0 &&
           ::rename(backup.path.c_str(), previous.c_str()) == 0;
    }
  }
  // Also sync on a first save: failures must be known before publication.
  if (ok && ::fsync(directory.fd) != 0)
    ok = false;
  if (ok && ::rename(temporary.path.c_str(), current.c_str()) != 0)
    ok = false;
  if (ok && ::fsync(directory.fd) != 0)
    ok = false;
  if (!directory.Close())
    ok = false;
  return ok;
}
catch (const std::exception&)
{
  return false;
}

bool Clear(const std::string& prefix)
try
{
  Directory directory(Parent(prefix));
  if (directory.fd < 0)
    return false;
  bool ok = true;
  // Legacy files first: interrupted clearing must not resurrect a stale pair
  // after the modern files have disappeared. Current is removed last.
  for (const char* suffix : {".graph", ".meta", ".scans", ".checkpoint.previous", ".checkpoint"})
  {
    if (::unlink((prefix + suffix).c_str()) != 0 && errno != ENOENT)
    {
      ok = false;
      break;
    }
    if (std::string(suffix) == ".scans" && ::fsync(directory.fd) != 0)
    {
      ok = false;
      break;
    }
  }
  if (::fsync(directory.fd) != 0)
    ok = false;
  if (!directory.Close())
    ok = false;
  return ok;
}
catch (const std::exception&)
{
  return false;
}
}  // namespace fusion_graph::checkpoint
