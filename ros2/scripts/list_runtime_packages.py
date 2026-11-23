#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later
"""Print the apt packages that own the shared libraries the shipped trees load.

Used by ros2/Dockerfile to build the slim `runtime` image: the build stages
need compilers, -dev packages and colcon, the runtime only needs the shared
objects every ELF under the shipped prefixes links against. The list is
derived from the build output itself (ldd + dpkg -S), so it cannot drift from
what was actually compiled.

Usage: list_runtime_packages.py [--check] ROOT [ROOT ...]

  default  print one package name per line (sorted), exit 1 if any ELF has an
           unresolved dependency ("not found")
  --check  only verify that every dependency resolves (used in the runtime
           stage after the packages are installed); print the failures

Libraries that live under one of the ROOTs are part of the shipped trees
themselves and are not mapped to a package.
"""

import os
import subprocess
import sys

ELF_MAGIC = b"\x7fELF"


def iter_elf_files(roots):
    for root in roots:
        for dirpath, _dirnames, filenames in os.walk(root):
            for name in filenames:
                path = os.path.join(dirpath, name)
                if os.path.islink(path) or not os.path.isfile(path):
                    continue
                try:
                    with open(path, "rb") as handle:
                        if handle.read(4) == ELF_MAGIC:
                            yield path
                except OSError:
                    continue


def ldd(path):
    """Return (resolved library paths, unresolved sonames) for one ELF."""
    result = subprocess.run(["ldd", path], capture_output=True, text=True, check=False)
    resolved, missing = set(), set()
    for line in result.stdout.splitlines():
        line = line.strip()
        if "=> not found" in line:
            missing.add(line.split()[0])
        elif "=>" in line:
            target = line.split("=>", 1)[1].split("(")[0].strip()
            if target:
                resolved.add(target)
    return resolved, missing


def path_variants(path):
    """dpkg records the path it unpacked, which may differ from what ldd
    prints on a merged-/usr system (/lib vs /usr/lib) or through symlinks."""
    variants = {path, os.path.realpath(path)}
    for candidate in list(variants):
        if candidate.startswith("/usr/"):
            variants.add(candidate[len("/usr"):])
        else:
            variants.add("/usr" + candidate)
    return variants


def owning_packages(paths):
    owners = {}
    queries = sorted({variant for path in paths for variant in path_variants(path)})
    for start in range(0, len(queries), 200):
        batch = queries[start:start + 200]
        result = subprocess.run(["dpkg", "-S", *batch], capture_output=True, text=True, check=False)
        for line in result.stdout.splitlines():
            if ": " not in line:
                continue
            packages, owned = line.split(": ", 1)
            owners.setdefault(owned.strip(), set()).update(
                p.strip().split(":")[0] for p in packages.split(",")
            )
    unowned = []
    found = set()
    for path in paths:
        hits = set()
        for variant in path_variants(path):
            hits |= owners.get(variant, set())
        if hits:
            found |= hits
        else:
            unowned.append(path)
    return found, unowned


def main(argv):
    check_only = "--check" in argv
    roots = [os.path.realpath(a) for a in argv if a != "--check"]
    if not roots:
        print(__doc__, file=sys.stderr)
        return 2

    libraries, failures = set(), []
    elf_count = 0
    for path in iter_elf_files(roots):
        elf_count += 1
        resolved, missing = ldd(path)
        libraries |= resolved
        failures.extend(f"{path}: {soname} not found" for soname in sorted(missing))

    if failures:
        print(f"{len(failures)} unresolved shared-library dependencies:", file=sys.stderr)
        for failure in failures:
            print(f"  {failure}", file=sys.stderr)
        return 1
    if check_only:
        print(f"all dependencies of {elf_count} ELF files resolve", file=sys.stderr)
        return 0

    external = sorted(
        lib for lib in libraries
        if not any(os.path.realpath(lib).startswith(root + os.sep) for root in roots)
    )
    packages, unowned = owning_packages(external)
    if unowned:
        # A library outside every root that no package owns (hand-copied into
        # /usr/local, say) cannot be reproduced by apt in the runtime stage.
        print("libraries not owned by any package:", file=sys.stderr)
        for lib in unowned:
            print(f"  {lib}", file=sys.stderr)
        return 1
    for package in sorted(packages):
        print(package)
    print(
        f"{elf_count} ELF files, {len(external)} external libraries, {len(packages)} packages",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
