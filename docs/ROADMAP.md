# Roadmap

Features planned after the first release (the first version in git, once inside-out tracking works). Not ordered
by priority yet. Where omarchyvr already has a version, look at it first (it may be ported to C++ like the rest).

1. **Passthrough**: the headset cameras' view in the headset.
2. **Video recording of the OpenXR scene**: record what the headset shows, to a video file.
3. **VR home**: omarchyvr has a home window in the headset like ours; look at it and compare with `pibridge-home`.
4. **Display and rendering settings**, as in Pimax Play / EVO (we have the Windows screenshots):
   - rendering settings (render resolution and quality);
   - foveated rendering (the centre region);
   - the current rendering mode shown on the Dashboard;
   - IPD;
   - refresh rate.
5. **Controller pairing and unpairing, and a vibration test**: omarchyvr has parts of this; look there first.
6. **Microphone and sound tests**.
7. **Backgrounds**: omarchyvr has backgrounds (environments) for the headset; see what they are.
8. **Device serial numbers on the Dashboard**, like EVO: headset, controllers, base stations (read at run time and
   shown only in the app; never written to the repo or logs that get shared).

## Now (before the first release): room and controllers

Spatial scanning, step by step (each step ends with a live check in the headset):

1. **Floor, centre, forward** from the room map: fix the floor choice (no table 0.76 m below the head), the offset
   sign/frame, the in-headset recenter with a controller on the floor, the controller pose in `pibridge-roomsetup`.
1b. **Head tracking on all four cameras** (lower "tracking" pair + upper side "Pi-controller" pair): Basalt
   calibration export for 4 cameras, `PIMAX_SLAM_CAMS=0,1,2,3`, one live take 4 vs 2 cameras compared on drift,
   losses and creep; the default if better. More wall and ceiling points also help the room map.
   *Ready for the take (2026-10-19):* Tracking page "Head tracking cameras: 2 / 4" (`io_slam_cams`, default 2),
   the four-camera calibration from `pibridge-slam export`, the driver's frame time and camera-count guard
   ([TRACKING.md](TRACKING.md)); the take and the comparison: research/head-tracking-improvements.md, live take 4.
2. **Planes: floor, walls, tables** from the map points (horizontal and vertical planes, RANSAC), the boundary from
   the walls, shown on the Room page and in the headset.
3. **Room mesh** from the headset's stereo cameras: depth per frame (stereo matching on the GPU), fused with the SLAM
   poses into a mesh; saved with the room.
4. **What is what** (table, sofa, door): segmentation of the camera frames by a neural network on the PC GPU,
   labels on the mesh and the planes.

Controllers: the IMU-led tracker is the default; next live take decides the remaining overshoot and the
out-of-view behaviour.
