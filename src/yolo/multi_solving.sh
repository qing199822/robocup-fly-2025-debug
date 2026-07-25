#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
YOLO_PYTHON="${YOLO_PYTHON:-$PROJECT_ROOT/.venv-yolo/bin/python}"

if [ ! -x "$YOLO_PYTHON" ]; then
    echo "错误：找不到 YOLO Python 环境：$YOLO_PYTHON" >&2
    exit 1
fi

"$YOLO_PYTHON" "$SCRIPT_DIR/bbox2coord_node.py" typhoon_h480_0 &
"$YOLO_PYTHON" "$SCRIPT_DIR/bbox2coord_node.py" typhoon_h480_1 &
"$YOLO_PYTHON" "$SCRIPT_DIR/bbox2coord_node.py" typhoon_h480_2 &
"$YOLO_PYTHON" "$SCRIPT_DIR/bbox2coord_node.py" typhoon_h480_3 &
"$YOLO_PYTHON" "$SCRIPT_DIR/bbox2coord_node.py" typhoon_h480_4 &
"$YOLO_PYTHON" "$SCRIPT_DIR/bbox2coord_node.py" typhoon_h480_5
