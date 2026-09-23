# On-demand RGB-D for Codex

The bridge now offers two additional MCP tools. The ROS camera may continue
publishing, but Codex receives an image only when it calls `observe_rgbd`.
Images are not continuously sent to the model and no SAM3 inference is required.

| Task | Tools |
| --- | --- |
| Move left slowly for 3 seconds | `execute_velocity(0, 0.1, 0, 3)`; no image or target |
| Describe what is ahead | `observe_rgbd`; no movement |
| Measure an image region | `observe_rgbd`, then `get_depth_region` |
| Move based on the camera view | RGB-D tools, then a short `execute_velocity` with `rgbd_snapshot_id` |
| Use a precise SAM3 object mask | Existing `set_target`, `observe_scene`, `execute_steering` |

## Start using the new tools

The module is installed into the existing `/tmp/g1-codex-install` package and
the Docker MCP launcher loads it directly from the workspace. Reload the
`g1_ros` MCP connection to load the new tool definitions. The controller does
not need restarting. Keep the camera simulator and camera interface running.
SAM3 can remain running or be disabled at the next launch with `sam3:=false`.

First ask Codex:

> Use g1_ros observe_rgbd to inspect the camera view without moving. Describe
> the scene. Choose an interior region of the table, query its metric depth
> with get_depth_region, and report the result and observation age.

For a movement task:

> Approach the table using RGB-D snapshots; do not use SAM3. Inspect the RGB
> image, select an interior table region, and query its depth. Before each
> short movement, use a fresh RGB-D snapshot and pass its rgbd_snapshot_id to
> execute_velocity. Use at most 0.1 m/s and 0.5-second actions. Stop when the
> detected table point is about 0.8 m away, or if observations become unusable.

RGB-D-guided actions allow up to 60 seconds to reason about a retained image.
Before starting, the bridge compares a current synchronized RGB-D pair with the
decision image, both over the full view and each region queried with
`get_depth_region`. It rejects changed camera calibration, insufficient valid
depth, or changes over the configured comparison thresholds: more than 5% of
pixels changing by over 0.10 m depth or 30 RGB intensity levels in any region.
This is a scene-change heuristic, not semantic tracking or collision avoidance;
it can reject harmless camera sway and can miss small changes below the thresholds.

Camera freshness remains 1.5 seconds. Visual actions last at most one second,
and expire to zero if the revalidated image becomes stale during execution.
All pending images are invalidated when an action starts or stop is requested;
observe again after each action. An old image alone never authorizes movement.

For a nonvisual task:

> Move left slowly for 3 seconds using execute_velocity. Do not request images
> or set a target. Use vx=0, vy=0.1, yaw_rate=0, duration=3.

Omitting `rgbd_snapshot_id` retains the tracking-free command behavior. All
movement still obeys execution/preview mode, controller readiness, velocity
limits, cancellation, joystick takeover, and the controller's 200 ms watchdog.

## observe_rgbd

`observe_rgbd(include_depth_preview=false)` returns MCP image content plus
metadata. The first image is RGB, encoded as JPEG at its original dimensions.
There is no resizing, so pixel regions refer directly to the returned image.
An optional second image is a depth visualization, with a fixed TURBO colour
scale from 0.1 to 10 metres and invalid depth shown as black. For numerical
distances, query depth rather than estimating metres from the colours.

Metadata includes `rgbd_snapshot_id`, image width/height, optical frame,
capture timestamp, age, freshness, and RGB/depth timestamp offset. Capture
requires aligned RGB, depth, and calibration within 30 ms of each other, all
no older than 1.5 seconds. It validates dimensions, optical frames and intrinsics.

Default ROS inputs:

- `/camera/color/image_raw`
- `/camera/aligned_depth_to_color/image_raw`
- `/camera/aligned_depth_to_color/camera_info`

Bridge parameters `rgbd_rgb_topic`, `rgbd_depth_topic`, and `rgbd_info_topic`
can override the topics. Only small bounded frame caches are retained. Image
encoding and depth processing happen on tool calls, outside the control timer.

## get_depth_region

Arguments: `rgbd_snapshot_id`, `x_min`, `y_min`, `x_max`, `y_max`.

- Origin: top-left of the original RGB image; x rightward, y downward.
- Minimum bounds inclusive; maximum bounds exclusive.
- Uses the depth captured with that exact RGB image, even if newer frames arrive.
- Supports `16UC1` millimetres and `32FC1` metres; returns metres.
- Excludes zero, nonfinite, and out-of-range depth (outside 0.1–10 m).
- Reports valid-pixel fraction, median depth, 10th/90th percentiles, and a
  median 3D optical-frame point using calibrated rays and lens distortion.
- When TF is available at capture time, also reports a torso-heading-relative
  point and horizontal distance. RGB images and optical depth do not require TF.

Depth is optical Z, not radial distance. Regions may contain multiple surfaces;
statistics do not establish that all pixels belong to the intended object.
Choose an interior region, inspect the depth spread, or use optional SAM3 masks.
Distance to a sampled point is not clearance from the object's nearest edge.

At most four snapshots are retained for 60 seconds. Older depth results expose
`fresh: false`; expired or evicted IDs require another `observe_rgbd` call.
These RGB-D IDs are separate from SAM3's single-use `snapshot_id`.

This is an observe/reason/act loop, not continuous obstacle avoidance. Returning
an image does not make every visible path traversable. Fast reactions belong
in a local ROS controller.

## Validation

Tests cover synchronization, staleness, future timestamps, depth units, invalid
pixels, image colour/dimensions, calibration, retained-frame identity, eviction,
MCP image transport, operation without SAM3/TF, visual-action expiry, reasoning delays, scene-change rejection, and consumed-image rejection. The
relative-motion tests continue to run without any camera fixture.

```bash
# Inside the container, from the repository directory:
source /opt/ros/jazzy/setup.bash
/opt/g1-codex-venv/bin/python -m unittest discover -s tests -p test_codex_rgbd.py -v
ROS_DOMAIN_ID=173 /opt/g1-codex-venv/bin/python tests/test_codex_ros_bridge.py -v
```
