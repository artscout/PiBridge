# Room scanning: model choice (research, 2026-10-19)

Not measured yet; licences marked (m) still need a check of the LICENSE file before use.

Camera facts: four OV7251 grey 640x480 fisheye (KB4, f ~234 px). Only the lower pair (75 mm baseline) does stereo:
disparity ~17.5 px at 1 m, 5.8 px at 3 m, 3.5 px at 5 m (~13 cm error at 3 m with 0.25 px matching error), so
stereo depth is good to about 3-4 m; the upper side cameras give monocular depth only. Rectify the lower pair to
pinhole first (OpenCV fisheye::stereoRectify + remap). Networks expect RGB: grey copied into 3 channels.

## First version
1. Depth: rectified lower pair -> RT-IGEV++ (MIT) or RAFT-Stereo fast (MIT (m)), ONNX on ONNX Runtime CUDA (as
   Monado's hand tracking). Upper cameras: Depth Anything 3 Small/Base (Apache-2.0), scaled per frame to stereo and
   SLAM points, only outside the stereo overlap.
2. Segmentation: EoMT (MIT) with a DINOv2 backbone (Apache-2.0 (m)), ADE20K classes (floor, wall, ceiling, table,
   chair, sofa, door). ADE20K terms (research/education) make the trained weights a grey area: download on first
   run, do not ship. Later: SAM 2 (Apache-2.0 (m)) + OWLv2 (Apache-2.0 (m)) for open-vocabulary labels.
3. Fusion: nvblox (Apache-2.0), CUDA TSDF from depth + Basalt poses, per-voxel label histogram, mesh. Planes
   (floor, walls, ceiling, tables) by RANSAC on the labelled mesh.

## Do not use (licences)
FoundationStereo and Fast-FoundationStereo (non-commercial / conflicting terms), UniDepth (CC-BY-NC), Depth Anything
V2 Base/Large/Giant and Depth Anything 3 Large/Giant/Nested (CC-BY-NC), YOLO-World (GPL-3.0), Ultralytics (AGPL-3.0),
PlaneRCNN (NVIDIA non-commercial), ScanNet/Matterport-trained checkpoints (ODIN, Mask3D, OpenScene, PlaneRecTR) for
shipping. Check BGNet, PlaneTR, SAM 3, DINOv3 before use.

## Other candidates
Selective-Stereo (MIT), CREStereo (Apache-2.0 (m)), HITNet (Apache-2.0 (m), fast, weaker zero-shot), Metric3D v2
(code BSD-2, weights licence not stated), Depth Pro (Apple licence, too slow live), OneFormer / Mask2Former (MIT,
heavy), Open3D (MIT (m), large dependency), Voxblox (BSD-3 (m), CPU, unmaintained).

## Sources
github.com/gangweix/IGEV-plusplus, github.com/ByteDance-Seed/Depth-Anything-3,
huggingface.co/SpatialHub/depth-anything-3-onnx, github.com/tue-mps/eomt, github.com/nvidia-isaac/nvblox,
github.com/NVlabs/FoundationStereo, huggingface.co/nvidia/fast-foundationstereo, github.com/Windsrain/Selective-Stereo,
github.com/YvanYin/Metric3D, github.com/lpiccinelli-eth/UniDepth, github.com/apple/ml-depth-pro,
github.com/SHI-Labs/OneFormer, github.com/AILab-CVC/YOLO-World, github.com/pengsongyou/openscene,
github.com/ayushjain1144/odin, github.com/SJingjia/PlaneRecTR, github.com/cherubicXN/PlaneTR3D
