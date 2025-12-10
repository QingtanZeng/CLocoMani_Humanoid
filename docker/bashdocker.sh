#!/usr/bin/env bash
set -euo pipefail

xhost +local:root

docker exec -it --user dev dev_legrobot /bin/bash