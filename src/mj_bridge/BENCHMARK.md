# Panda assembly benchmark

This ROS package exposes a MuJoCo Panda arm, parallel gripper, RGB-D camera and
product-level assembly scoring. A product YAML defines registered parts and their
target poses. Each seed creates a repeatable loose-parts layout.

## Run an assembly

From the repository root, after installing dependencies:

```bash
source enter_sim_env.sh
colcon build --base-paths src/mj_bridge --packages-select mj_bridge --symlink-install
source enter_sim_env.sh
lego-bench run --product examples/products/traffic_light.yaml --seed 42 \
  --headless --executor baseline --output-dir runs/traffic-light-42
```

Omit `--executor baseline` to start the ROS environment for an external controller.
The baseline uses Oracle observations by default. Optional visual inference is
described in `docs/VALIDATION_ZH.md`; GPU inference is not locally validated.

## Episode services

```bash
ros2 service call /mj_bridge/reset std_srvs/srv/Trigger '{}'
ros2 service call /mj_bridge/result std_srvs/srv/Trigger '{}'
ros2 topic echo /mj_bridge/benchmark_state
```

Assembly success requires every part to satisfy the pose tolerances. A failed
executor or expired episode cannot report success. Results include per-part
errors, completion, elapsed time, and observation and connection modes.

The `snap` mode uses contact-qualified grid alignment and simplified connections.
The `physics` mode disables these connections. Snap results do not validate
physical LEGO deformation or real-robot operation.

Run directories retain the product, initial manifest, scene, final state and
result. See `docs/PUBLIC_BENCHMARK_ZH.md` for interfaces and
`docs/validation/REPORT_ZH.md` for measured coverage.
