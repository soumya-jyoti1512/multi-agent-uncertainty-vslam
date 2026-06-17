# Collaborative Multi-Agent Visual SLAM for Uncertainty-Aware Navigation

>A ROS 2-based collaborative SLAM framework integrating ORB-SLAM3 for visual odometry, COVINS-G for multi-robot map fusion, and SCOPE for uncertainty-aware pose estimation and coordination in distributed 3D mapping., validated within the Gazebo simulator and deployed on TurtleBot4 hardware.
---

Lab Member webpage(Check alumni section): https://tasl.ucr.edu/people/

# Note
This project was conducted as part of TASL Lab at university of California, Riverside. The codebase, robot hardware, and computing infrastructure were owned by the lab. Accordingly, no source code or experimental data are published in this repository. This README documents the system design, technical architecture, and implementation approach for reference and portfolio purposes.

# Personal Reimplementation & Planned Extensions
A personal reimplementation of this system in a simulated environment is planned. Since the original codebase belongs to the university research lab, this reimplementation is being developed independently from scratch. Three extensions beyond the original architecture are being incorporated, each targeting a distinct layer of the system.

---

# Overview

Traditional multi-robot SLAM systems operate independently: each robot maintains its own local map and localization estimate without a globally shared spatial representation. This leads to:

- Redundant mapping of shared environments
- Drift accumulation across robots
- Inconsistent global reference frames
- Poor coordination during collaborative navigation

This project addresses those limitations by building a collaborative multi-agent visual SLAM system in which three TurtleBot4 robots jointly construct and maintain a single globally consistent map in real time.

Each robot runs ORB-SLAM3 locally as the visual front-end, extracting ORB features from RGB-D camera streams and generating keyframes. These keyframes are transmitted through a ROS2 + DDS communication layer to a centralized COVINS-G back-end, which performs:

- Global pose graph optimization
- Cross-agent loop closure detection
- Drift correction
- Shared map fusion

A major limitation of conventional SLAM-driven navigation is that planners assume localization is perfect. This project explicitly addresses that by integrating SCOPE, which propagates covariance estimates from the SLAM back-end and produces uncertainty-aware localization confidence scores.

The resulting system enables:

- Collaborative multi-robot mapping
- Shared global localization
- Cross-robot loop closure
- Uncertainty-aware trajectory planning
- Confidence-modulated navigation behavior

The complete pipeline was validated in both:

- **Gazebo simulation**
- **Real-world deployment with 3 TurtleBot4 robots**

---

# Table of Contents

- [System Architecture](#system-architecture)
- [Technical Approach](#technical-approach)
  - [ORB-SLAM3 Front-End](#1-orb-slam3--per-robot-front-end)
  - [COVINS-G Back-End](#2-covins-g--multi-agent-global-back-end)
  - [SCOPE Uncertainty Estimation](#3-scope--uncertainty-estimation)
  - [Uncertainty-Aware Nav2 Planning](#4-uncertainty-aware-navigation-with-nav2)
- [Mathematical Formulation](#mathematical-formulation)
- [Software Stack](#software-stack)
- [Hardware Platform](#hardware-platform)
- [Simulation & Validation](#simulation--validation)
- [Personal Reimplementation & Planned Extensions](#personal-reimplementation--planned-extensions)

---

# System Architecture

The system is organized into four functional layers:

1. Per-robot SLAM front-ends
2. Shared ROS2 middleware
3. Centralized multi-agent SLAM back-end
4. Uncertainty-aware navigation

## System Architecture

<img width="2916" height="2286" alt="Image" src="https://github.com/user-attachments/assets/1e6b6ef1-fa7a-4880-9d4a-a23a2eea3440" />

# Technical Approach

---

## 1. ORB-SLAM3 - Per-Robot Front-End

ORB-SLAM3 provides the per-robot visual SLAM front-end due to its robustness across monocular, stereo, and RGB-D input modes and its structured keyframe representation, which is directly compatible with COVINS-G's map fusion protocol. On each robot, ORB-SLAM3 performs real-time feature extraction using ORB (Oriented FAST and Rotated BRIEF) keypoints from the RGB-D stream, computes visual odometry through feature matching and PnP pose estimation, and maintains a local map of 3D map points. Keyframes are selected based on tracking quality and motion thresholds, and their associated poses, ORB descriptors, and co-visible map points are forwarded to the back-end.

Each TurtleBot4 runs an independent ORB-SLAM3 front-end on its onboard Raspberry Pi 4.

### Responsibilities

- ORB feature extraction
- Feature matching
- RGB-D visual odometry
- Local mapping
- Keyframe generation
- Pose estimation

### Sensor Input

- Intel RealSense D435i RGB-D stream
- Depth + RGB synchronized frames
- 640×480 @ 30 FPS

### ORB Feature Pipeline
<img width="1385" height="2015" alt="Image" src="https://github.com/user-attachments/assets/8db2a692-c7a1-4180-bac3-04e6a9814dba" />

### ORB Descriptor

ORB combines:

- FAST corners
- BRIEF descriptors
- Orientation compensation

Feature matching is performed using Hamming distance over binary descriptors.

### Pose Estimation

Camera pose is estimated via Perspective-n-Point (PnP):

```math
\mathbf{x}_i \sim \mathbf{K} [\mathbf{R} | \mathbf{t}] \mathbf{X}_i
```

Where:

- `X_i` → 3D landmark point
- `x_i` → projected image point
- `K` → camera intrinsic matrix
- `R,t` → camera pose

---

## 2. COVINS-G - Multi-Agent Global Back-End

COVINS-G maintains a global pose graph in which nodes represent shared keyframes from all agents and edges encode relative pose constraints derived from visual co-visibility and loop closure detections. Place recognition is performed using DBoW2 (Bag of Binary Words) on incoming ORB descriptors, enabling loop closure detection both within a single agent's trajectory and across different agents the mechanism by which the system achieves cross-robot map alignment. When a loop is detected, a new edge is added to the graph, and a joint non-linear pose graph optimization is solved to minimize accumulated drift across all robots simultaneously, maintaining global consistency.

COVINS-G acts as the centralized collaborative SLAM back-end.

Its responsibilities include:

- Global pose graph construction
- Cross-agent map fusion
- Drift correction
- Loop closure optimization
- Shared localization

---

## Pose Graph Representation

The global SLAM graph is represented as:

```math
G = (V, E)
```

Where:

- `V` → robot keyframe poses
- `E` → relative pose constraints

Each edge encodes a transformation:

```math
T_{ij} \in SE(3)
```

representing the relative transformation between keyframes `i` and `j`.

---

## Loop Closure Optimization

The back-end minimizes the pose graph objective:

```math
\mathbf{x}^* =
\arg \min_{\mathbf{x}}
\sum_{(i,j)\in E}
\left\|
\mathbf{z}_{ij} -
h(\mathbf{x}_i,\mathbf{x}_j)
\right\|_{\Omega_{ij}}^2
```

Where:

- `z_ij` → observed relative pose
- `h(x_i,x_j)` → predicted transformation
- `Ω_ij` → information matrix

This optimization jointly corrects drift across all robots simultaneously.

---

## Cross-Agent Loop Closure

Place recognition is performed using DBoW2 bag-of-words matching over ORB descriptors.

<img width="2376" height="1835" alt="Image" src="https://github.com/user-attachments/assets/aee0bc26-b065-40d1-8714-f416d78c83f5" />

Cross-agent loop closures allow robots to align maps even when exploring independently.

---


### 3. SCOPE - Stochastic Occupancy Prediction (Dynamic-Environment Forecasting)

While the collaborative visual SLAM stack (ORB-SLAM3 + COVINS-G) handles localization and static mapping, SCOPE adds awareness of the dynamic environment. It is a family of deep-neural-network occupancy predictors (`scope++`, `scope`, `so-scope`), implemented in PyTorch, that forecast the future occupancy grid map of the scene about 0.5 s (5 steps) ahead together with the uncertainty of that forecast.

**Pipeline:**

- **Input** - the robot's **2D LiDAR** stream is converted into a short history of local occupancy grid maps, together with the robot's ego-motion (so SCOPE can separate its own movement from the scene's).
- **Model** - a stochastic deep generative network rolls the occupancy grid forward in time. Because it is stochastic, a single inference draws several plausible future occupancy samples.
- **Output** - from those samples, a prediction (mean) occupancy map (where dynamic obstacles are likely to be) and an uncertainty map (how confident the forecast is). The lightweight `so-scope` variant reads from a precomputed uncertainty-statistics lookup table for fast inference on resource-limited robots.

Prediction mean and uncertainty over $K$ stochastic occupancy samples $\hat{M}^{(k)}$ predicted $\Delta$ steps into the future:

$$\bar{M}_{t+\Delta} = \frac{1}{K}\sum_{k=1}^{K}\hat{M}^{(k)}_{t+\Delta}, \qquad U_{t+\Delta} = \frac{1}{K}\sum_{k=1}^{K}\left(\hat{M}^{(k)}_{t+\Delta} - \bar{M}_{t+\Delta}\right)^{2}$$

The mean map $\bar{M}$ and uncertainty map $U$ become the prediction and uncertainty costmap layers, with each occupied cell mapped to a soft (Gaussian) cost rather than a lethal one.

> Role of the LiDAR: although localization and mapping are camera-based (RGB-D → ORB-SLAM3 / COVINS-G), the RPLIDAR A1M8 is the sensor that feeds SCOPE. The camera drives collaborative SLAM; the LiDAR drives dynamic occupancy prediction.

### 4. Nav2 - Predictive Uncertainty-Aware Navigation

SCOPE's outputs are fused into the ROS 2 navigation stack (Nav2) as **two additional costmap layers** merged into the master costmap:

1. Prediction costmap layer - adds cost where dynamic obstacles are forecast to be, so the planner routes around where people are *heading*, not just where they currently are.
2. Uncertainty costmap layer - adds cost in proportion to the forecast uncertainty, so the robot keeps a wider margin where the prediction is less reliable.

Predicted and uncertain cells are mapped to a soft Gaussian cost rather than a lethal obstacle value a predicted obstacle or an uncertain region is not a confirmed obstacle, so a soft cost gradient steers the robot toward safer routes without freezing it. The result is a predictive, uncertainty-aware nominal path. Because the layers use the standard layered-costmap interface, the framework works on top of either a classical controller (e.g., DWA) or a learned policy (e.g., DRL-VO).


# Mathematical Formulation

---

## Collaborative Pose Graph Optimization

The full optimization objective:

```math
\mathbf{x}^* =
\arg \min_{\mathbf{x}}
\sum_{(i,j)\in E}
e_{ij}^T \Omega_{ij} e_{ij}
```

with residual:

```math
e_{ij} =
z_{ij}^{-1}
(x_i^{-1}x_j)
```

---

## Visual Feature Matching

Feature correspondence:

```math
m^* =
\arg \min_m
d_H(f_i, f_j)
```

Where:

- `d_H` → Hamming distance
- `f_i, f_j` → ORB descriptors

---


---

# Software Stack

| Component | Framework | Role |
|---|---|---|
| SLAM Front-End | ORB-SLAM3 | Local visual SLAM |
| SLAM Back-End | COVINS-G | Global map fusion |
| Uncertainty Module | SCOPE | Pose covariance estimation |
| Navigation | Nav2 | Uncertainty-aware planning |
| Middleware | ROS2 + DDS | Communication |
| Simulation | Gazebo | Multi-robot simulation |
| Place Recognition | DBoW2 | Loop closure detection |
| Language | C++ , Python | core and coordination |

---

# Hardware Platform

| Component | Specification |
|---|---|
| Mobile Robots | 3× TurtleBot4 |
| RGB-D Cameras | 3× Intel RealSense D435i |
| Onboard Compute | Raspberry Pi 5 (16GB RAM) |
| Ground Station | Ubuntu 24.04 Workstation |
| Network | Dedicated 5 GHz Wi-Fi |
| LiDAR | RPLIDAR A1M8 |

---

# Quantitative System Parameters

| Parameter | Value |
|---|---|
| Number of Robots | 3 |
| RGB-D Resolution | 640×480 |
| Camera Frame Rate | 30 FPS |
| LiDAR Type | RPLIDAR A1M8 |
| Communication | ROS2 DDS |
| SLAM Mode | RGB-D |
| Pose Space | SE(3) |
| Optimization | Nonlinear Pose Graph |
| Map Representation | Keyframe Graph |

---

# Simulation & Validation

The complete system was validated in:

- Gazebo simulation
- Real TurtleBot4 deployment

### Simulation Objectives

- Multi-agent loop closure testing
- DDS discovery validation
- Shared map consistency
- Uncertainty-aware trajectory adaptation

### Real-World Deployment

Three physical TurtleBot4 robots operated simultaneously in the same environment using:

- Shared SLAM back-end
- Collaborative mapping
- Centralized optimization
- Uncertainty-aware navigation

---

# Personal Reimplementation & Planned Extensions

>  A full independent reimplementation of the system is currently planned.

Since the original research codebase belongs to the university lab, the reimplementation is being developed independently from scratch.

---

## 1. Decentralized Multi-Agent Back-End

The original system uses a centralized COVINS-G server.

The reimplementation replaces this with:

- Peer-to-peer DDS communication
- Distributed pose graph optimization
- Consensus-based synchronization

### Distributed Optimization Objective

```math
x_i^{k+1} =
x_i^k +
\alpha
\sum_{j \in \mathcal{N}(i)}
(x_j^k - x_i^k)
```

This removes the single point of failure.

---

## 2. Learning-Based Feature Extraction

ORB features are replaced with:

- SuperPoint
- LightGlue

### Motivation

ORB struggles in:

- Low-texture environments
- Motion blur
- Dynamic scenes
- Poor lighting

### Updated Pipeline

<img width="1385" height="1565" alt="Image" src="https://github.com/user-attachments/assets/2e25efae-9e71-446c-b187-b663858312c1" />

Expected improvements:

- More robust tracking
- Fewer outlier matches
- Improved loop closure reliability

---

## 3. Semantic Uncertainty for Dynamic Scenes (YOLOv8n)

SCOPE predicts *where* occupancy will be and how uncertain that is, but it is **class-agnostic** — it does not know *what* is moving. This extension adds a lightweight **YOLOv8n** detector and folds a semantic term into SCOPE's uncertainty layer, so the robot is extra cautious specifically around recognized humans:

$$D = \sum_i A_i\, c_i, \qquad U_{\text{final}} = \alpha\, U_{\text{occ}} + \beta\, U_{\text{semantic}}$$

where $A_i$ is a normalized bounding-box area and $c_i$ the detection confidence. This enriches the occupancy-prediction uncertainty; it does **not** alter the SLAM localization.
