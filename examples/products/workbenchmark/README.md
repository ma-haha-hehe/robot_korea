# Workbenchmark regression inputs

These three-, nine-, and twelve-part tasks are imported from `ma-haha-hehe/Workbenchmark`, commit
`213acb19744607ca69ae285f205a851e7a78b7e2`.

The importer preserves the source parts, relative target geometry, layer spacing,
and recorded loose-part XY positions and yaw. It translates the finished layout
to the robot assembly origin and maps the initial Z coordinate to the simulated
table surface. The original task files remain in Workbenchmark.
`validation_provenance.json` records the upstream and imported hashes for all
three inputs. These are fixed regression cases, not a random or complete suite.

The `regression/` directory preserves eight additional historical-failure/dense
inputs byte for byte, with hashes in its `provenance.json`. The integrated
contact-aware controller passed two and failed six of these complete episodes.
They remain available for reproducing failures, and are not a representative
sample for estimating overall success rate. See the audit report for outcomes.

The integrated tilt-gated unloading revision subsequently passed five of these
eight cases, fixing `tier2_task_037`, `tier3_task_005`, and `tier4_task_051`.
The other three still fail. Current and baseline results are kept separately in
`tilt-extended-results.json` and `extended-current-results.json` under the main
project's `runs/project-audit-20261013/` directory.

Run it with the plastic contact profile:

```bash
bash run_panda_duplo.sh examples/products/workbenchmark/tier2_task_001.yaml runs/workbenchmark-example 42
```

The contact model is a simulation approximation, not measured material calibration.
See `docs/PLASTIC_CONTACT_ZH.md` for parameters and acceptance limits.
See `docs/PANDA_DUPLO_AUDIT_20261014.txt` for the accepted revision and complete
success/failure evidence. A recorded initial layout is retained when changing
the seed; different seeds for these files do not create different source poses.
