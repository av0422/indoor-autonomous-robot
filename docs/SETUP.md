# Setup

## Prerequisites

- Docker Desktop or OrbStack
- 6 CPU / 8 GB RAM allocated to the container
- macOS on Apple Silicon (host has been validated on M2)

## Build and start the container

```bash
docker compose build
docker compose up -d
docker compose exec ros bash
```

## Path note

The repo root is mounted at `/ws` inside the container. On the Mac host it
is the current directory. Only `./src`, `./docs`, `./results`, and `./data`
are mounted — files written anywhere else in the repo root on the host will
not appear inside the container, and vice versa.

## Python virtual environment

Inside the container:

```bash
python3 -m venv /ws/.venv --system-site-packages
source /ws/.venv/bin/activate
pip install -r docs/requirements.txt
```

`--system-site-packages` is required so the venv can see the ROS Python
packages (rclpy, cv_bridge, etc.) installed by apt.

## Build the workspace

```bash
cd /ws
colcon build --symlink-install
source install/setup.bash
```

## Run Gazebo headless

No display is available in the container, so Gazebo runs headless:

```bash
gz sim -s -r
```

## Foxglove

RViz is not used. Visualize via Foxglove instead:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Then connect from the Foxglove desktop/web app on the host to:

```
ws://localhost:8765
```

Port 8765 is published by `docker-compose.yml`.
