#!/usr/bin/env bash
# Heavy generation belongs to the Antigravity verification pass.
set -euo pipefail
project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
utils="$project/micro_ros_stm32cubemx_utils"
if [[ ! -d "$utils" ]]; then
    git clone --depth 1 --branch humble https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git "$utils"
fi
# Configure the finite DDS entity pool for this bare-metal serial application.
python3 - "$utils/microros_static_library/library_generation/colcon.meta" <<'PY'
import json,sys
from pathlib import Path
p=Path(sys.argv[1]); data=json.loads(p.read_text())
data['names']['microxrcedds_client']['cmake-args'] = [
    '-DUCLIENT_PIC=OFF', '-DUCLIENT_PROFILE_UDP=OFF', '-DUCLIENT_PROFILE_TCP=OFF',
    '-DUCLIENT_PROFILE_DISCOVERY=OFF', '-DUCLIENT_PROFILE_SERIAL=OFF',
    '-DUCLIENT_PROFILE_STREAM_FRAMING=ON', '-DUCLIENT_PROFILE_CUSTOM_TRANSPORT=ON']
data['names']['rmw_microxrcedds']['cmake-args'] = [
    '-DRMW_UXRCE_MAX_NODES=1', '-DRMW_UXRCE_MAX_PUBLISHERS=4',
    '-DRMW_UXRCE_MAX_SUBSCRIPTIONS=1', '-DRMW_UXRCE_MAX_SERVICES=0',
    '-DRMW_UXRCE_MAX_CLIENTS=0', '-DRMW_UXRCE_MAX_HISTORY=1',
    '-DRMW_UXRCE_TRANSPORT=custom']
p.write_text(json.dumps(data,indent=2)+'\n')
PY
cd "$project"
# Builder asks to confirm Makefile ABI flags; provide the single affirmative reply.
printf 'y\n' | docker run -i --rm -v "$project:/project" \
    --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library \
    microros/micro_ros_static_library_builder:humble
