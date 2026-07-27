#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$WORKSPACE_DIR/.." && pwd)"
XTDRONE_DIR="${XTDRONE_DIR:-$PROJECT_ROOT/XTDrone}"
while [ "$XTDRONE_DIR" != "/" ] && [ "${XTDRONE_DIR%/}" != "$XTDRONE_DIR" ]; do
    XTDRONE_DIR="${XTDRONE_DIR%/}"
done

SOURCE_DIR="$XTDRONE_DIR/sitl_config/gazebo_plugin/actor_collisions"
BUILD_ROOT="$WORKSPACE_DIR/build"
BUILD_DIR="$WORKSPACE_DIR/build/actor_collisions"
DEVEL_DIR="$WORKSPACE_DIR/devel"
OUTPUT_DIR="$WORKSPACE_DIR/devel/lib"
BUILD_ARTIFACT="$BUILD_DIR/libActorCollisionsPlugin.so"
OUTPUT_FILE="$OUTPUT_DIR/libActorCollisionsPlugin.so"
LOCK_FILE="$WORKSPACE_DIR/build/.actor-collisions.lock"

fail() {
    echo "错误：$*" >&2
    exit 1
}

if ! command -v flock >/dev/null 2>&1; then
    fail "缺少 flock，无法安全串行构建 actor collision 插件。请安装 util-linux。"
fi
if ! command -v realpath >/dev/null 2>&1; then
    fail "缺少 realpath，无法验证团队工作区的安全路径。请安装 coreutils。"
fi

WORKSPACE_CANONICAL="$(realpath -e -- "$WORKSPACE_DIR")"

validate_existing_workspace_directory() {
    local directory="$1"
    local canonical

    if [ -L "$directory" ]; then
        fail "团队工作区目录 $directory 是符号链接，拒绝不安全的输出路径。"
    fi
    if [ ! -e "$directory" ]; then
        return
    fi
    if [ ! -d "$directory" ]; then
        fail "团队工作区路径 $directory 不是目录。"
    fi
    canonical="$(realpath -e -- "$directory")"
    case "$canonical" in
        "$WORKSPACE_CANONICAL"/*) ;;
        *) fail "团队工作区目录 $directory 超出工作区边界，拒绝不安全的输出路径。" ;;
    esac
}

ensure_workspace_directory() {
    local directory="$1"

    validate_existing_workspace_directory "$directory"
    if [ ! -e "$directory" ]; then
        if ! mkdir -- "$directory" 2>/dev/null; then
            validate_existing_workspace_directory "$directory"
            if [ ! -d "$directory" ]; then
                fail "无法安全创建团队工作区目录 $directory。"
            fi
        fi
    fi
    validate_existing_workspace_directory "$directory"
}

verify_source_directories() {
    local source_path="$XTDRONE_DIR"
    local component

    for component in "XTDrone" "sitl_config" "gazebo_plugin" "actor_collisions"; do
        if [ "$component" != "XTDrone" ]; then
            source_path="$source_path/$component"
        fi
        if [ -L "$source_path" ]; then
            fail "官方源码目录 $source_path 是符号链接，拒绝不安全的源码路径。"
        fi
        if [ ! -e "$source_path" ]; then
            fail "缺少官方源码目录 $source_path。"
        fi
        if [ ! -d "$source_path" ]; then
            fail "官方源码路径 $source_path 不是目录。"
        fi
    done
}

verify_official_source() {
    local file_name="$1"
    local expected_hash="$2"
    local source_file="$SOURCE_DIR/$file_name"
    local actual_hash

    if [ -L "$source_file" ]; then
        fail "官方源码 $source_file 是符号链接，拒绝不安全的源码路径。"
    fi
    if [ ! -e "$source_file" ]; then
        fail "缺少官方源码 $source_file。"
    fi
    if [ ! -f "$source_file" ]; then
        fail "官方源码 $source_file 不是普通文件。"
    fi

    read -r actual_hash _ < <(sha256sum -- "$source_file")
    if [ "$actual_hash" != "$expected_hash" ]; then
        echo "错误：官方源码 $file_name 的 SHA-256 哈希已变化，拒绝构建。" >&2
        echo "期望：$expected_hash" >&2
        echo "实际：$actual_hash" >&2
        exit 1
    fi
}

verify_source_directories
verify_official_source \
    "ActorCollisionsPlugin.cc" \
    "e15f07b4a9cc19db1a05dd1aafd1b81557b2badf728cc28d666500034b34e499"
verify_official_source \
    "ActorCollisionsPlugin.hh" \
    "78db47b17157eeb97676fc0ceecc95662dd1a8018c3730c492962ca431b61c29"
verify_official_source \
    "CMakeLists.txt" \
    "f38958df562a9f66f435c42e831f2e2606a86b6b7287ad0eb6b8cbb7e4d03b28"

ensure_workspace_directory "$BUILD_ROOT"
BUILD_ROOT_CANONICAL="$(realpath -e -- "$BUILD_ROOT")"
if [ -L "$LOCK_FILE" ]; then
    fail "构建锁 $LOCK_FILE 是符号链接，拒绝不安全的输出路径。"
fi
if [ -e "$LOCK_FILE" ] && [ ! -f "$LOCK_FILE" ]; then
    fail "构建锁 $LOCK_FILE 不是普通文件。"
fi
exec 9>> "$LOCK_FILE"
if [ "$(realpath -e -- "$LOCK_FILE")" != "$BUILD_ROOT_CANONICAL/.actor-collisions.lock" ]; then
    fail "构建锁超出团队工作区边界，拒绝不安全的输出路径。"
fi
if ! flock 9; then
    fail "无法获取 actor collision 构建锁。"
fi

# Check every pre-existing destination before creating or mutating shared paths.
validate_existing_workspace_directory "$BUILD_DIR"
validate_existing_workspace_directory "$DEVEL_DIR"
validate_existing_workspace_directory "$OUTPUT_DIR"
ensure_workspace_directory "$BUILD_DIR"
ensure_workspace_directory "$DEVEL_DIR"
ensure_workspace_directory "$OUTPUT_DIR"

if [ -L "$BUILD_ARTIFACT" ]; then
    fail "构建产物路径 $BUILD_ARTIFACT 是符号链接，拒绝不安全的输出路径。"
fi
if [ -e "$BUILD_ARTIFACT" ] && [ ! -f "$BUILD_ARTIFACT" ]; then
    fail "构建产物路径 $BUILD_ARTIFACT 不是普通文件。"
fi
if [ -L "$OUTPUT_FILE" ]; then
    fail "插件输出路径 $OUTPUT_FILE 是符号链接，拒绝不安全的输出路径。"
fi
if [ -e "$OUTPUT_FILE" ] && [ ! -f "$OUTPUT_FILE" ]; then
    fail "插件输出路径 $OUTPUT_FILE 不是普通文件。"
fi

rm -f -- "$BUILD_ARTIFACT"
cmake -S "$SOURCE_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --parallel

if [ ! -s "$BUILD_ARTIFACT" ]; then
    fail "构建产物缺失或为空：$BUILD_ARTIFACT"
fi

temp_output="$(mktemp "$OUTPUT_DIR/.libActorCollisionsPlugin.so.XXXXXX")"
cleanup() {
    if [ -n "${temp_output:-}" ] && [ -e "$temp_output" ]; then
        rm -f -- "$temp_output"
    fi
}
trap cleanup EXIT

cp -- "$BUILD_ARTIFACT" "$temp_output"
if [ ! -s "$temp_output" ]; then
    fail "复制后的插件产物为空：$temp_output"
fi
chmod 0755 "$temp_output"
mv -f -- "$temp_output" "$OUTPUT_FILE"
temp_output=""
trap - EXIT

echo "Actor collision 插件已生成：$OUTPUT_FILE"
