# Workbenchmark example

This three-part task is imported from `ma-haha-hehe/Workbenchmark`, commit
`213acb19744607ca69ae285f205a851e7a78b7e2`.

The importer preserves the source parts, relative target geometry, layer spacing,
and recorded loose-part XY positions and yaw. It translates the finished layout
to the robot assembly origin and maps the initial Z coordinate to the simulated
table surface. The original task file remains in Workbenchmark.

Run it with the plastic contact profile:

```bash
python -m mj_bridge.benchmark_cli run --product examples/products/workbenchmark/tier2_task_001.yaml --seed 42 --connection-mode physics --contact-profile plastic --executor oracle-baseline --robot-base-x=-.05 --headless --output-dir runs/workbenchmark-example
```

The contact model is a simulation approximation, not measured material calibration.
See `docs/PLASTIC_CONTACT_ZH.md` for parameters and acceptance limits.
