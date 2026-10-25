# Project Context: 3D Drone Detection and Tracking from LiDAR with Radar Fusion (MMAUD)

This file gives you the full background for this project. Read it first, then follow the "Status and next steps" section at the bottom.

## 1. About me (the user)

- Strong in computer vision model training and experiment design (PyTorch, detection, segmentation, ablations, metrics). My thesis used voxel-based and slice-by-slice 3D segmentation of volumetric data (not point clouds).
- Already experienced with ONNX export, TensorRT and quantization, so those are not learning goals.
- Comfortable in Python. Beginner in C++, ROS 2, Kalman filters, point cloud geometry and sensor fusion.
- Goal: build a portfolio project that shows I can work with LiDAR point cloud data and hardware-related perception systems, to apply for perception engineering roles in drones, autonomous vehicles and robotics.
- I am learning as I build. Please explain concepts briefly when they first appear (for example what a Kalman filter update step does), but do not lecture. Keep explanations short and tied to the code.
- Hindi and English both work for chat. Documents and code are written in English.

## 2. Project goal

Build a 3D perception pipeline that detects and tracks a flying drone from LiDAR point cloud data, then improves it with a second sensor via sensor fusion. The finished project should show a clear, measured improvement from fusion over a LiDAR-only baseline.

The point of the project is to learn how to work with LiDAR data and sensor hardware, not to use machine learning for its own sake. Do not force ML into a step that does not need it.

Core deliverables:

1. A LiDAR-only baseline: point cloud preprocessing, detection of the drone by clustering, and a Kalman filter tracker.
2. Sensor fusion: add a second modality (radar first, then a second LiDAR or camera if time allows) and show the improvement against the baseline, including a test where one sensor drops out.
3. Proper evaluation: quantitative metrics, ablations, and a 3D or bird's-eye-view visualization of tracks vs ground truth.
4. A ROS 2 deployment: the tracker running as a ROS 2 node that replays a recorded bag, with tracks shown in RViz2.
5. A clean README with results, a short write-up, and a demo video or GIF.

Optional, gated component:

- **Learned detector** (phase 3). Only add it if the baseline results show a problem a learned model would fix (for example missed detections at long range or false clusters). If clustering is already near-perfect on the chosen sequence, either build a very small version (such as a compact classifier that rejects false clusters) or skip it. Decide with me after the baseline numbers.

Optional extras, only if time allows, in this order of priority:

- A C++ port of the tracker (Kalman filter and track management), tested against the Python outputs.
- A GitHub Actions workflow that runs the tests.
- A second MMAUD sequence (a different drone) to check that results generalize.

Explicitly out of scope for now: GTSAM, Ceres, SLAM or odometry, physical sensor calibration (use the calibration files that ship with the dataset), Jetson deployment, ONNX/TensorRT export (I already have this skill), and drone simulators (Gazebo, PX4). Do not add these unless I ask.

Later, separate project (not part of this one): LiDAR-inertial odometry and mapping on NTU VIRAL, to learn odometry and SLAM. Do not mix it into this repository.

## 3. Datasets

Chosen dataset: **MMAUD** (multi-modal anti-UAV dataset, NTU ARIS lab). Source: https://github.com/ntu-aris/MMAUD, paper: Yuan et al., ICRA 2024, arXiv 2402.03706.

What has been verified from the repo page and paper abstract:

| Item | Finding |
|---|---|
| Sensors | Two 3D LiDARs (a conic one and a peripheral one), two time-synchronized cameras, one mmWave radar, four audio array nodes |
| LiDAR models | Conic is a Livox Avia (70 degree field of view, up to 300 m, facing upward). Peripheral is a Livox Mid-360. Model details come from a secondary review of the paper. |
| Layout | Sensors are fixed on the ground and look up at the sky, so there is probably no ground plane (to be confirmed on real frames) |
| Format | ROS 1 bags (.bag), compressed. `rosbag decompress` restores full playback rate. ROS topic names and message types are NOT documented, inspect the bag. |
| Ground truth | Leica total station, stored in a separate .bag per sequence. Accuracy not stated. Whether it shares a coordinate frame with the LiDAR is unknown, check early. |
| Sequences | V1 "Rooftop Simple": five drones (Mavic2, Mavic3, Phantom4, Avata, M300), mostly below 30 m. V2 and V3 are harder (up to 100 m), have unknown sizes and some placeholder ground truth links, avoid them. |
| V1 sizes | Mavic3 11.1 GB (321 s), Phantom4 13.2 GB (181 s), Mavic2 14.1 GB (198 s), M300 14.4 GB (429 s), Avata 19.7 GB (396 s) |
| License | CC BY-NC-SA 4.0 (non-commercial academic use). Cite the paper, do not redistribute the raw data. |
| Hosting | OneDrive (V1), Google Drive (V2, V3, calibration) |
| Calibration | fisheye_calibration.zip on Google Drive, plus sensor_calibration.md and sensors_and_usage.md in the repo (not yet read) |

Sequence to use first: **V1 DJI Mavic3** (smallest V1 file). Files to download: the ROSBag Data .bag and the Ground truth .bag, into `data/mmaud/v1_mavic3/`. The "Folder Data" .zip is skipped until we know what it contains. The mapping of links to columns on the page is inferred from position, so confirm by file size.

Fallback and later options:

- **nuScenes mini split**: driving data with LiDAR, radar and cameras, official devkit, tracking metrics (AMOTA, AMOTP) and pretrained models. Details are from memory and not verified (about 10 scenes, about 4 GB, free account and Terms of Use required). Use it only if MMAUD turns out to be unworkable. Tell me and propose it, do not switch silently.
- **NTU VIRAL**: drone-mounted LiDAR, camera, IMU and UWB for odometry. Reserved for the later separate project.
- **KITTI Tracking**: small and well documented, only for a quick test.

Datasets rejected after review: NFL Big Data Bowl (2D tabular player positions, no sensor data), UAV tree point clouds and industrial point clouds on Kaggle (static scenes, no moving target or time sequence).

Read ROS bags in Python with the `rosbags` library so that a ROS installation is not required. Livox sensors may use custom message types that need to be registered by hand, check this first.

## 4. Technical approach

Suggested pipeline (propose changes if you see better options):

1. **Data loading**: read point clouds, radar and ground truth, synchronize timestamps across sensors, and write a small loader with a consistent internal format.
2. **Preprocessing** (Open3D, NumPy): voxel downsampling and region of interest cropping. Background subtraction for a fixed sensor. RANSAC ground removal only if real frames show a ground plane worth removing.
3. **Classical detection baseline**: DBSCAN or similar clustering to find the small, sparse, moving target.
4. **Tracking**: Kalman filter, constant velocity first, then constant acceleration. Hungarian algorithm (SciPy) for association only if there is more than one object. Track birth and death logic.
5. **Fusion**: add radar as an extra measurement update in the filter (late fusion). Then, if time allows, a second LiDAR or camera. Always compare against the LiDAR-only baseline.
6. **Learned detector (optional, gated)**: see section 2. If built, use a small PointNet-style model or a light voxel detector, and feed its output to the same Kalman filter.
7. **Evaluation**: position error (RMSE, ATE style) against ground truth, detection precision and recall, track continuity (ID switches, track fragmentation), and robustness with a sensor dropout test.
8. **Visualization**: 3D plot or bird's-eye-view animation of predicted track vs ground truth, saved as a GIF or video.
9. **ROS 2 deployment**: a Python rclpy node that subscribes to point clouds and publishes tracks, fed by `ros2 bag play` and viewed in RViz2. Then the optional C++ port.

## 5. Constraints

- **Everything must be free.** Open source libraries only (Python, NumPy, SciPy, Open3D, PyTorch, rosbags, filterpy or a hand-written Kalman filter). No paid services.
- **Machine**: Windows 11 Home, Intel Core Ultra 7 258V, 31.5 GB RAM, no NVIDIA GPU. The classical parts run on the laptop CPU.
- **GPU training** (only if the optional learned detector is built): use free Kaggle notebooks (about 30 GPU hours per week, third-party figure) or free Colab. DeepInfra is an inference service and is not used for training. Keep training code separate from the main pipeline, save checkpoints often, and preprocess the large bags on the laptop into small files before uploading anything.
- **Disk space**: D: has about 361 GB free. Ask me before downloading anything large, and prefer small sequences first.
- **Python environment**: conda, created by path in `D:\my_envs\lidar-drone` with Python 3.10 (`conda create -p D:\my_envs\lidar-drone python=3.10`, activate with the full path). Conda does not search `D:\my_envs` by default, so do not change my global conda config unless I ask. Existing env there: `sam-edge`, do not touch it.
- **ROS 2**: native Windows is a poor fit. Use WSL2 with Ubuntu 22.04 (Humble) or 24.04 (Jazzy) when we reach the ROS 2 phase. Check whether WSL is installed before suggesting setup.
- Pinned `requirements.txt` or `environment.yml` and a fixed random seed for reproducibility.

## 6. How I want you to work

- Start simple and get an end-to-end baseline working before adding complexity. A finished, measured project beats a half-done ambitious one.
- Work in small steps. After each milestone, run it, show me the result or metric, and then ask before moving on to the next stage.
- Write readable code with short comments and docstrings. I want to understand every file, so avoid clever abstractions.
- Write tests for the Kalman filter and the data loaders (for example, a filter fed a perfect constant velocity track should converge).
- Keep experiments configurable (YAML or argparse) and log results in a simple, comparable format so I can build ablation tables.
- Ask me before making large decisions (dataset choice, model choice, switching away from this plan). Do not guess silently.
- If something is uncertain or you cannot verify it (dataset format, library API, license), say so and check the docs instead of assuming.
- Do not use em dashes in any text or documents you write (code comments, README, docs, commit messages). Use commas, colons, parentheses or hyphens instead.
- Stay in one lane (multi-sensor 3D perception for moving platforms) and avoid learning something new for each job posting. Add a new topic only when this project needs it.

## 7. Build in public

I plan to share progress on LinkedIn and GitHub, with a longer write-up on a blog and a demo video at the end. When a milestone produces a good visual or result, suggest it as a post topic and offer a draft. Only report numbers that were actually measured, and say how they were measured. Cite the MMAUD paper and do not re-upload raw data.

Planned post milestones: project announcement, first rendered frame, lessons from the data, clustering without ML, Kalman filter explained on a drone track, first baseline numbers, fusion ablation, ROS 2 node in RViz2, optional C++ port, final write-up.

## 8. Suggested repository layout

```
project-root/
  CLAUDE.md
  README.md
  requirements.txt
  configs/
  data/               (gitignored, raw and processed data, e.g. data/mmaud/v1_mavic3/)
  src/
    io/               (dataset loaders, rosbag reading, time sync)
    preprocessing/    (voxel, cropping, background subtraction, optional RANSAC)
    detection/        (clustering baseline, optional learned model wrappers)
    tracking/         (Kalman filter, association, track management)
    fusion/           (late fusion, optional feature-level fusion)
    evaluation/       (metrics, ablation scripts)
    visualization/    (3D and BEV plots, video export)
  ros2_ws/src/drone_tracker/   (ROS 2 node, Python first, C++ later)
  cpp/                (standalone C++ tracker and tests, if useful)
  notebooks/          (exploration only)
  tests/
  results/            (metrics tables, figures, videos)
```

## 9. Rough phases

1. **Setup and data exploration**: environment, dataset access, open the bag, list topics and message types, check the ground truth frame, visualize a point cloud frame with the ground truth position.
2. **LiDAR-only baseline**: preprocessing, clustering, Kalman filter, first error numbers against ground truth.
3. **Fusion**: add radar, run ablations (LiDAR only, radar only, fused, sensor dropout).
4. **Optional learned detector**: only if the baseline shows a problem it would fix (decision gate after phase 2).
5. **ROS 2 deployment**: WSL2 setup, Python node, bag replay, RViz2 demo. Then the optional C++ port.
6. **Write-up**: metrics tables, plots, visualization video, README, blog post.
7. **Optional extras**: second sequence, CI.

## 10. Status and next steps

Done so far:

1. Environment checked: Windows 11, Python 3.12 system install, conda at `C:\Users\adima\anaconda3` (mamba available), no NVIDIA GPU, D: has about 361 GB free.
2. Datasets researched and MMAUD chosen (section 3). nuScenes mini details are still unverified.

Next:

1. Create the `data/mmaud/v1_mavic3/` folder. I download the Mavic3 ROSBag Data and Ground truth .bag files by hand from the MMAUD GitHub page, and tell you when they are in place.
2. Propose the pinned dependency list and the environment file. Wait for my confirmation, then create the conda env at `D:\my_envs\lidar-drone`.
3. Inspect the bags: topics, message types, rates, radar data, ground truth frame. Report what you find before writing the main pipeline.
4. Write a script that loads one sequence and renders one point cloud frame with the ground truth position overlaid.
