#!/usr/bin/env bash
set -euo pipefail

xhost +local:root

# 临时复制父目录的文件到当前目录
cp ../dependencies.txt .

HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose up -d --build

echo "Container started. Use ./bashdocker.sh to enter."

