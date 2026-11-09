// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The checkpoint helper is compiled directly into this target. Linker wrappers
// inject failures at the actual I/O boundary without a production fault seam.
// Child exits test process-interruption recovery, not physical power loss.

#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "fusion_graph/checkpoint_file.hpp"
#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

namespace cp = fusion_graph::checkpoint;
namespace fs = std::filesystem;

namespace
{
enum class Failure
{
  None,
  Write,
  ShortWrite,
  Flush,
  FileSync,
  FileClose,
  DirectoryOpen,
  Link,
  BackupRename,
  BackupDirectorySync,
  CommitRename,
  CommitDirectorySync,
  SecondDirectorySync,
  DirectoryClose,
};

enum class Crash
{
  None,
  PartialWrite,
  BeforeCommit,
  AfterCommit,
};

struct Injection
{
  Failure failure = Failure::None;
  Crash crash = Crash::None;
  bool consumed = false;
  bool committed = false;
  unsigned int directory_syncs = 0;
};

Injection injection;
constexpr int kCrashExit = 73;

bool Fail(Failure wanted)
{
  if (injection.failure != wanted || injection.consumed)
    return false;
  injection.consumed = true;
  errno = wanted == Failure::Write || wanted == Failure::ShortWrite ? ENOSPC : EIO;
  return true;
}

bool DirectoryFd(int fd)
{
  struct stat info
  {
  };
  return ::fstat(fd, &info) == 0 && S_ISDIR(info.st_mode);
}

bool EndsWith(const char* path, const char* suffix)
{
  return std::string(path).ends_with(suffix);
}

class CheckpointFileTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    auto pattern = (fs::temp_directory_path() / "fg_checkpoint_XXXXXX").string();
    char* created = ::mkdtemp(pattern.data());
    ASSERT_NE(created, nullptr);
    directory_ = created;
    prefix_ = directory_ + "/graph";
    injection = {};
  }

  void TearDown() override
  {
    injection = {};
    if (!directory_.empty())
    {
      std::error_code ec;
      fs::remove_all(directory_, ec);
    }
  }

  void ExpectPayload(const std::string& path, const cp::Payload& expected)
  {
    cp::Payload actual;
    ASSERT_TRUE(cp::Read(path, actual));
    EXPECT_EQ(actual.graph, expected.graph);
    EXPECT_EQ(actual.metadata, expected.metadata);
  }

  void InterruptSave(const cp::Payload& payload, Crash crash)
  {
    const pid_t child = ::fork();
    ASSERT_GE(child, 0);
    if (child == 0)
    {
      injection = {};
      injection.crash = crash;
      const bool saved = cp::Save(prefix_, payload);
      ::_exit(saved ? 1 : 2);  // Failure to reach the selected crash boundary.
    }
    int status = 0;
    ASSERT_EQ(::waitpid(child, &status, 0), child);
    ASSERT_TRUE(WIFEXITED(status));
    ASSERT_EQ(WEXITSTATUS(status), kCrashExit);
  }

  std::string directory_;
  std::string prefix_;
  const cp::Payload old_{"old graph bytes", "old metadata bytes"};
  const cp::Payload next_{"new graph bytes", "new metadata bytes"};
};
}  // namespace

extern "C"
{
  size_t __real_fwrite(const void*, size_t, size_t, FILE*);
  int __real_fflush(FILE*);
  int __real_fsync(int);
  int __real_fclose(FILE*);
  int __real_open(const char*, int, ...);
  int __real_close(int);
  int __real_link(const char*, const char*);
  int __real_rename(const char*, const char*);

  size_t __wrap_fwrite(const void* data, size_t size, size_t count, FILE* file)
  {
    if (injection.crash == Crash::PartialWrite)
    {
      __real_fwrite(data, 1, size * count / 2, file);
      __real_fflush(file);
      ::_exit(kCrashExit);
    }
    if (Fail(Failure::Write))
      return 0;
    if (Fail(Failure::ShortWrite))
    {
      if (count > 1)
        return __real_fwrite(data, size, count - 1, file);
      __real_fwrite(data, 1, size / 2, file);
      return 0;
    }
    return __real_fwrite(data, size, count, file);
  }

  int __wrap_fflush(FILE* file)
  {
    if (Fail(Failure::Flush))
      return EOF;
    return __real_fflush(file);
  }

  int __wrap_fsync(int fd)
  {
    const bool directory = DirectoryFd(fd);
    if (directory && ++injection.directory_syncs == 2 && Fail(Failure::SecondDirectorySync))
      return -1;
    const auto failure = directory ? (injection.committed ? Failure::CommitDirectorySync
                                                          : Failure::BackupDirectorySync)
                                   : Failure::FileSync;
    if (Fail(failure))
      return -1;
    return __real_fsync(fd);
  }

  int __wrap_fclose(FILE* file)
  {
    // Read validation also closes FILEs in some implementations. The injected
    // close failure is for the newly written checkpoint, not its predecessor.
    const bool writable = (::fcntl(::fileno(file), F_GETFL) & O_ACCMODE) != O_RDONLY;
    const bool fail = writable && Fail(Failure::FileClose);
    const int result = __real_fclose(file);
    if (fail)
    {
      errno = EIO;
      return EOF;
    }
    return result;
  }

  int __wrap_open(const char* path, int flags, ...)
  {
    if ((flags & O_DIRECTORY) && Fail(Failure::DirectoryOpen))
      return -1;
    if (flags & O_CREAT)
    {
      va_list args;
      va_start(args, flags);
      const auto mode = va_arg(args, mode_t);
      va_end(args);
      return __real_open(path, flags, mode);
    }
    return __real_open(path, flags);
  }

  int __wrap_close(int fd)
  {
    const bool fail = DirectoryFd(fd) && Fail(Failure::DirectoryClose);
    const int result = __real_close(fd);
    if (fail)
    {
      errno = EIO;
      return -1;
    }
    return result;
  }

  int __wrap_link(const char* from, const char* to)
  {
    if (Fail(Failure::Link))
      return -1;
    return __real_link(from, to);
  }

  int __wrap_rename(const char* from, const char* to)
  {
    const bool commit = EndsWith(to, ".checkpoint");
    if (commit && injection.crash == Crash::BeforeCommit)
      ::_exit(kCrashExit);
    if (Fail(commit ? Failure::CommitRename : Failure::BackupRename))
      return -1;
    const int result = __real_rename(from, to);
    if (commit && result == 0)
    {
      injection.committed = true;
      if (injection.crash == Crash::AfterCommit)
        ::_exit(kCrashExit);
    }
    return result;
  }
}  // extern "C"

TEST_F(CheckpointFileTest, CombinedPayloadRoundTripsAndRetainsPrevious)
{
  ASSERT_TRUE(cp::Save(prefix_, old_));
  ExpectPayload(prefix_ + ".checkpoint", old_);
  ASSERT_TRUE(cp::Save(prefix_, next_));
  ExpectPayload(prefix_ + ".checkpoint", next_);
  ExpectPayload(prefix_ + ".checkpoint.previous", old_);
}

TEST_F(CheckpointFileTest, FramingPreservesBinaryPayloads)
{
  const cp::Payload binary{std::string("graph\0bytes", 11), std::string("meta\0bytes", 10)};
  ASSERT_TRUE(cp::Save(prefix_, binary));
  ExpectPayload(prefix_ + ".checkpoint", binary);
}

TEST_F(CheckpointFileTest, EachPreCommitFailureReturnsFalseAndPreservesCurrent)
{
  ASSERT_TRUE(cp::Save(prefix_, old_));
  const Failure failures[] = {Failure::Write,
                              Failure::ShortWrite,
                              Failure::Flush,
                              Failure::FileSync,
                              Failure::FileClose,
                              Failure::DirectoryOpen,
                              Failure::Link,
                              Failure::BackupRename,
                              Failure::BackupDirectorySync,
                              Failure::CommitRename};
  for (const auto failure : failures)
  {
    SCOPED_TRACE(static_cast<int>(failure));
    injection = {};
    injection.failure = failure;
    EXPECT_FALSE(cp::Save(prefix_, next_));
    EXPECT_TRUE(injection.consumed) << "The intended I/O boundary was not exercised";
    injection = {};
    ExpectPayload(prefix_ + ".checkpoint", old_);
  }
}

TEST_F(CheckpointFileTest, PostCommitFailuresKeepPreviousAndNeverReportSuccess)
{
  for (const auto failure : {Failure::CommitDirectorySync, Failure::DirectoryClose})
  {
    SCOPED_TRACE(static_cast<int>(failure));
    injection = {};
    ASSERT_TRUE(cp::Save(prefix_, old_));
    injection = {};
    injection.failure = failure;
    EXPECT_FALSE(cp::Save(prefix_, next_));
    EXPECT_TRUE(injection.consumed);
    injection = {};
    ExpectPayload(prefix_ + ".checkpoint.previous", old_);
    // The rename may already be visible even though durability was not
    // acknowledged. It still contains one complete graph/metadata generation.
    cp::Payload current;
    ASSERT_TRUE(cp::Read(prefix_ + ".checkpoint", current));
    EXPECT_TRUE((current.graph == old_.graph && current.metadata == old_.metadata) ||
                (current.graph == next_.graph && current.metadata == next_.metadata));
  }
}

TEST_F(CheckpointFileTest, CorruptCurrentCannotReplaceTheValidRecoveryCopy)
{
  ASSERT_TRUE(cp::Save(prefix_, old_));
  ASSERT_TRUE(cp::Save(prefix_, next_));
  {
    std::ofstream damaged(prefix_ + ".checkpoint", std::ios::binary);
    damaged << "damaged primary";
  }
  injection = {};
  injection.failure = Failure::CommitRename;
  EXPECT_FALSE(cp::Save(prefix_, {"third graph", "third metadata"}));
  EXPECT_TRUE(injection.consumed);
  injection = {};
  ExpectPayload(prefix_ + ".checkpoint.previous", old_);
}

TEST_F(CheckpointFileTest, SemanticallyInvalidCurrentKeepsRecoveryOnFailedReplacement)
{
  ASSERT_TRUE(cp::Save(prefix_, old_));
  ASSERT_TRUE(cp::Save(prefix_, {"not a decodable graph", "invalid metadata"}));
  injection = {};
  injection.failure = Failure::CommitRename;
  // The graph layer has rejected this CRC-valid generation. Preserve the
  // previous archive instead of rotating a semantically bad primary over it.
  EXPECT_FALSE(cp::Save(prefix_, next_, false));
  EXPECT_TRUE(injection.consumed);
  injection = {};
  ExpectPayload(prefix_ + ".checkpoint.previous", old_);
}

TEST_F(CheckpointFileTest, ReadRejectsTruncationCorruptionAndTrailingData)
{
  ASSERT_TRUE(cp::Save(prefix_, old_));
  std::ifstream stream(prefix_ + ".checkpoint", std::ios::binary);
  const std::string bytes((std::istreambuf_iterator<char>(stream)), {});
  ASSERT_GT(bytes.size(), 8u);
  const std::string damaged_path = directory_ + "/damaged";
  for (const size_t length : {size_t{0}, size_t{8}, bytes.size() / 2, bytes.size() - 1})
  {
    SCOPED_TRACE(length);
    std::ofstream damaged(damaged_path, std::ios::binary);
    damaged.write(bytes.data(), length);
    damaged.close();
    cp::Payload loaded;
    EXPECT_FALSE(cp::Read(damaged_path, loaded));
  }
  auto corrupt = bytes;
  corrupt.back() ^= 1;
  for (const auto& invalid : {corrupt, bytes + "unexpected trailing bytes"})
  {
    std::ofstream damaged(damaged_path, std::ios::binary);
    damaged << invalid;
    damaged.close();
    cp::Payload loaded;
    EXPECT_FALSE(cp::Read(damaged_path, loaded));
  }
}

TEST_F(CheckpointFileTest, InterruptedFirstSaveDoesNotPublishTemporaryFiles)
{
  InterruptSave(next_, Crash::BeforeCommit);
  EXPECT_FALSE(fs::exists(prefix_ + ".checkpoint"));
  EXPECT_FALSE(fs::exists(prefix_ + ".checkpoint.previous"));
  bool temporary_exists = false;
  for (const auto& entry : fs::directory_iterator(directory_))
    temporary_exists |= entry.path().filename().string().starts_with("graph.checkpoint.tmp.");
  EXPECT_TRUE(temporary_exists);
  cp::Payload loaded;
  EXPECT_FALSE(cp::Read(prefix_ + ".checkpoint", loaded));
}

TEST_F(CheckpointFileTest, InterruptedPartialWritePreservesPriorCheckpoint)
{
  ASSERT_TRUE(cp::Save(prefix_, old_));
  InterruptSave(next_, Crash::PartialWrite);
  ExpectPayload(prefix_ + ".checkpoint", old_);
}

TEST_F(CheckpointFileTest, InterruptedBeforeRenameRetainsPriorGeneration)
{
  ASSERT_TRUE(cp::Save(prefix_, old_));
  InterruptSave(next_, Crash::BeforeCommit);
  ExpectPayload(prefix_ + ".checkpoint", old_);
  ExpectPayload(prefix_ + ".checkpoint.previous", old_);
}

TEST_F(CheckpointFileTest, InterruptedAfterRenameHasWholeNewGenerationAndPrevious)
{
  ASSERT_TRUE(cp::Save(prefix_, old_));
  InterruptSave(next_, Crash::AfterCommit);
  ExpectPayload(prefix_ + ".checkpoint", next_);
  ExpectPayload(prefix_ + ".checkpoint.previous", old_);
}

TEST_F(CheckpointFileTest, MissingParentDirectoryFails)
{
  EXPECT_FALSE(cp::Save(directory_ + "/missing/graph", next_));
}

TEST_F(CheckpointFileTest, ClearRemovesModernAndLegacyFilesAndRetainsLidarData)
{
  ASSERT_TRUE(cp::Save(prefix_, old_));
  ASSERT_TRUE(cp::Save(prefix_, next_));
  for (const auto* suffix : {".graph", ".meta", ".scans", ".lidarmap"})
  {
    std::ofstream file(prefix_ + suffix);
    file << "legacy or independent map data";
  }
  ASSERT_TRUE(fs::create_directory(prefix_ + ".lidartiles"));
  injection = {};
  ASSERT_TRUE(cp::Clear(prefix_));
  for (const auto* suffix : {".graph", ".meta", ".scans", ".checkpoint", ".checkpoint.previous"})
    EXPECT_FALSE(fs::exists(prefix_ + suffix)) << suffix;
  EXPECT_TRUE(fs::exists(prefix_ + ".lidarmap"));
  EXPECT_TRUE(fs::exists(prefix_ + ".lidartiles"));
  EXPECT_TRUE(cp::Clear(prefix_));  // Missing checkpoint files are harmless.
}

TEST_F(CheckpointFileTest, ClearLegacyRemovalFailurePreservesModernCheckpoint)
{
  ASSERT_TRUE(cp::Save(prefix_, old_));
  ASSERT_TRUE(fs::create_directory(prefix_ + ".meta"));
  injection = {};
  EXPECT_FALSE(cp::Clear(prefix_));
  ExpectPayload(prefix_ + ".checkpoint", old_);
}

TEST_F(CheckpointFileTest, ClearPreRemovalDirectoryFailuresRetainCurrent)
{
  ASSERT_TRUE(cp::Save(prefix_, old_));
  for (const auto failure : {Failure::DirectoryOpen, Failure::BackupDirectorySync})
  {
    SCOPED_TRACE(static_cast<int>(failure));
    injection = {};
    injection.failure = failure;
    EXPECT_FALSE(cp::Clear(prefix_));
    EXPECT_TRUE(injection.consumed);
    injection = {};
    ExpectPayload(prefix_ + ".checkpoint", old_);
  }
}

TEST_F(CheckpointFileTest, ClearFinalSyncAndCloseFailuresNeverReportSuccess)
{
  for (const auto failure : {Failure::SecondDirectorySync, Failure::DirectoryClose})
  {
    SCOPED_TRACE(static_cast<int>(failure));
    injection = {};
    ASSERT_TRUE(cp::Save(prefix_, old_));
    injection = {};
    injection.failure = failure;
    EXPECT_FALSE(cp::Clear(prefix_));
    EXPECT_TRUE(injection.consumed);
    injection = {};
  }
}
