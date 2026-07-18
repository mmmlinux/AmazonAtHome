FROM ros:humble-ros-base

# ── System & Python deps ───────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
  && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir "fastapi>=0.100" "uvicorn[standard]>=0.23"

# ── Build workspace ───────────────────────────────────────────────────────────
WORKDIR /ws
COPY . /ws/

RUN . /opt/ros/humble/setup.sh \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# ── Runtime ───────────────────────────────────────────────────────────────────
EXPOSE 8080

# Source both setups, then pass through any arguments to ros2 launch.
# Default: launch the full system with standard settings.
# Override at runtime:
#   docker run ... min_battery_1:=40.0 travel_speed:=3.0
ENTRYPOINT ["/bin/bash", "-c", \
  "mkdir -p /logs; exec 1> >(tee /logs/warehouse_$(date +%Y%m%d_%H%M%S).log); exec 2>&1; . /opt/ros/humble/setup.bash && . /ws/install/setup.bash && exec ros2 launch logistics_server logistics.launch.py \"$@\"", \
  "--"]
CMD []
