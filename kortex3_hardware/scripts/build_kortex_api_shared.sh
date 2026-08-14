#!/usr/bin/env bash
#
# Regenerate libkortex_api_shared.so from the vendored Kinova SDK static archive.
#
# libkortex_api_shared.so is libKortexApiCpp.a wrapped whole-archive into ONE
# shared library. Both this package's ros2_control plugins AND libkortex3_private.so
# (the gripper driver) link it dynamically, so the process has a SINGLE protobuf
# runtime + descriptor pool (see CMakeLists.txt for why -- otherwise a shared
# Kinova::Api::Frame is freed by the wrong protobuf copy -> "free(): invalid
# pointer" during gripper Modbus init).
#
# Run this whenever third_party/kortex3/libKortexApiCpp.a changes, THEN rebuild
# libkortex3_private.so (in the kortex3_hardware_private repo) from the SAME
# archive. The two .so files carry protobuf descriptor tables that must have an
# identical layout; if they drift, descriptor registration segfaults at load.
# Both .so files are committed to this repo and must always agree.

set -euo pipefail

pkg_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sdk_dir="${pkg_dir}/third_party/kortex3"

arch="$(uname -m)"
if [ "${arch}" = "aarch64" ]; then
  api_lib="${sdk_dir}/lib/release/libKortexApiCpp_ARM.a"
  out_dir="${pkg_dir}/lib/release_arm"
else
  api_lib="${sdk_dir}/lib/release/libKortexApiCpp.a"
  out_dir="${pkg_dir}/lib/release"
fi

if [ ! -f "${api_lib}" ]; then
  echo "error: Kinova SDK archive not found: ${api_lib}" >&2
  exit 1
fi

mkdir -p "${out_dir}"
out="${out_dir}/libkortex_api_shared.so"

echo "Wrapping ${api_lib}"
echo "     ->  ${out}"

# System deps the SDK/protobuf pull in (zlib = protobuf gzip streams) must be
# resolved WITHIN the .so, after the whole-archive group, so no undefined symbols
# leak to downstream links.
g++ -shared -o "${out}" \
  -Wl,-soname,libkortex_api_shared.so \
  -Wl,--whole-archive "${api_lib}" -Wl,--no-whole-archive \
  -lz -ldl -lrt -lm -lpthread

echo "Done."
echo "Now rebuild libkortex3_private.so from the same archive so the two agree."