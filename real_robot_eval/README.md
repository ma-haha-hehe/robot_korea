# Real-Robot Evaluation Notes

Use this directory to record the small real-robot validation added to the thesis.
The goal is not to replace the simulation benchmark, but to show that the
perception-to-execution pipeline can run on the physical UR5 setup.

## Recommended protocol

1. Start the UR driver terminal.
2. Start the MoveIt terminal.
3. Keep the Docker terminal for vision debugging only. The host pipeline starts
   and restarts the bridge process automatically.
4. Run dry-run trials first. Execute robot motion only after the generated
   `pick_place_task.yaml` looks reasonable.

## Minimum experiment set

- Perception-only dry-run: 5 trials for each target brick.
- Single pick-and-place: 5 trials for `yellow 2x4 brick.` and 5 trials for
  `blue 2x4 brick.`.
- Two-step assembly: 3 trials for `yellow 2x4 brick.,blue 2x4 brick.` with
  placement offsets `0,0,0;0.08,0,0`.

## Files to save per trial

- `FoundationPose/root/FoundationPose/vision_output.yaml`
- `src/panda_pick/config/pick_place_task.yaml`
- `src/panda_pick/src/plan_perception_test.yaml`
- `/shared_data/vision_pose_debug.jpg` from inside the vision Docker, or copy it
  out after a run if the OpenCV tracking window is hard to inspect
- the relevant terminal output from the pipeline run
- one before image and one after image of the workspace
- optional: final assembled structure photo for thesis figures

## Metrics

- vision success: whether a non-empty `vision_output.yaml` was produced
- detected target and pose: `T_cam_obj`, transformed `T_base_obj`, yaw
- perception time: from target trigger to vision output ready
- execution success: whether the robot completed without ROS or URScript error
- grasp success: whether the gripper lifted the correct brick
- placement success: whether the brick was placed at the intended grid location
- insertion quality: fully seated, partially seated, or failed insertion
- total pipeline time
- failure stage and short note
