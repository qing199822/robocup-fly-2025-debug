#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$WORKSPACE_DIR/.." && pwd)"
XTDRONE_DIR="${XTDRONE_DIR:-$PROJECT_ROOT/XTDrone}"
SOURCE_DIR="$XTDRONE_DIR/sitl_config/gazebo_plugin/actor_collisions"
BUILD_DIR="$WORKSPACE_DIR/build/actor_collisions"
OUTPUT_DIR="$WORKSPACE_DIR/devel/lib"
BUILD_ARTIFACT="$BUILD_DIR/libActorCollisionsPlugin.so"
OUTPUT_FILE="$OUTPUT_DIR/libActorCollisionsPlugin.so"

verify_official_source() {
    local file_name="$1"
    local expected_hash="$2"
    local source_file="$SOURCE_DIR/$file_name"
    local actual_hash

    if [ -L "$source_file" ]; then
        echo "错误：官方源码 $source_file 是符号链接，拒绝不安全的源码路径。" >&2
        exit 1
    fi
    if [ ! -e "$source_file" ]; then
        echo "错误：缺少官方源码 $source_file。" >&2
        exit 1
    fi
    if [ ! -f "$source_file" ]; then
        echo "错误：官方源码 $source_file 不是普通文件。" >&2
        exit 1
    fi

    actual_hash="$(sha256sum -- "$source_file" | cut -d ' ' -f 1)"
    if [ "$actual_hash" != "$expected_hash" ]; then
        echo "错误：官方源码 $file_name 的 SHA-256 哈希已变化，拒绝构建。" >&2
        echo "期望：$expected_hash" >&2
        echo "实际：$actual_hash" >&2
        exit 1
    fi
}

verify_official_source \
    "ActorCollisionsPlugin.cc" \
    "e15f07b4a9cc19db1a05dd1aafd1b81557b2badf728cc28d666500034b34e499"
verify_official_source \
    "ActorCollisionsPlugin.hh" \
    "78db47b17157eeb97676fc0ceecc95662dd1a8018c3730c492962ca431b61c29"

mkdir -p "$BUILD_DIR"
rm -f -- "$BUILD_ARTIFACT"
cmake -S "$SOURCE_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --parallel

if [ ! -s "$BUILD_ARTIFACT" ]; then
    echo "错误：构建产物缺失或为空：$BUILD_ARTIFACT" >&2
    exit 1
fi

mkdir -p "$OUTPUT_DIR"
temp_output="$(mktemp "$OUTPUT_DIR/.libActorCollisionsPlugin.so.XXXXXX")"
cleanup() {
    if [ -n "${temp_output:-}" ] && [ -e "$temp_output" ]; then
        rm -f -- "$temp_output"
    fi
}
trap cleanup EXIT

cp -- "$BUILD_ARTIFACT" "$temp_output"
if [ ! -s "$temp_output" ]; then
    echo "错误：复制后的插件产物为空：$temp_output" >&2
    exit 1
fi
chmod 0755 "$temp_output"
mv -f -- "$temp_output" "$OUTPUT_FILE"
temp_output=""
trap - EXIT

echo "Actor collision 插件已生成：$OUTPUT_FILE"
