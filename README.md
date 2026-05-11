# Collaborative Multi-Agent Visual SLAM for Uncertainty-Aware Navigation


---
```text
# Note
This project was conducted as part of TASL Lab at university of California, Riverside. The codebase, robot hardware, and computing infrastructure were owned by the lab. Accordingly, no source code or experimental data are published in this repository. This README documents the system design, technical architecture, and implementation approach for reference and portfolio purposes.

# Personal Reimplementation & Planned Extensions
In Progress: A personal reimplementation of this system in a simulated environment is currently in progress. Since the original codebase belongs to the university research lab, this reimplementation is being developed independently from scratch. Three extensions beyond the original architecture are being incorporated, each targeting a distinct layer of the system.
```
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

```text
+------------------+  +------------------+  +------------------+
|  TurtleBot4 #1   |  |  TurtleBot4 #2   |  |  TurtleBot4 #3   |
|  ORB-SLAM3       |  |  ORB-SLAM3       |  |  ORB-SLAM3       |
|  Front-End       |  |  Front-End       |  |  Front-End       |
|  RGB-D Stream    |  |  RGB-D Stream    |  |  RGB-D Stream    |
+--------+---------+  +--------+---------+  +--------+---------+
         |                     |                     |
         +---------------------+---------------------+
                               |
                    +----------+----------+
                    |   ROS2 + DDS Layer  |
                    | (Cyclone DDS / LAN) |
                    +----------+----------+
                               |
                    +----------+----------+
                    |      COVINS-G       |
                    | Global Pose Graph   |
                    | Optimization +      |
                    | Cross-Agent Loops   |
                    +----------+----------+
                               |
                    +----------+----------+
                    |      SCOPE          |
                    | Covariance /        |
                    | Uncertainty         |
                    +----------+----------+
                               |
                    +----------+----------+
                    |      Nav2           |
                    | Uncertainty-Aware   |
                    | Trajectory Planner  |
                    +---------------------+
```

---

# Technical Approach

---

## 1. ORB-SLAM3 - Per-Robot Front-End

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

```text
RGB-D Frame
      │
      ▼
FAST Keypoint Detection
      │
      ▼
ORB Descriptor Extraction
      │
      ▼
Feature Matching
      │
      ▼
PnP Pose Estimation
      │
      ▼
Keyframe Selection
```

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

```text
Robot A Keyframe ──► Visual Vocabulary
                                   │
Robot B Keyframe ──► Similarity Score
                                   │
                            Loop Closure?
                                   │
                                   ▼
                        Add Pose Constraint Edge
```

Cross-agent loop closures allow robots to align maps even when exploring independently.

---

## 3. SCOPE - Uncertainty Estimation

Traditional SLAM pipelines output only a pose estimate:

```math
\hat{x}
```

However, navigation requires knowledge of confidence as well.

SCOPE augments the pipeline by estimating pose covariance:

```math
\Sigma =
\mathbb{E}
[(x-\hat{x})(x-\hat{x})^T]
```

Where:

- `x̂` → estimated pose
- `Σ` → localization uncertainty covariance matrix

---

## Confidence Metric

Localization confidence is derived from covariance magnitude:

```math
c = \frac{1}{\mathrm{trace}(\Sigma)}
```

Higher covariance → lower confidence.

---

## Uncertainty Propagation

The uncertainty estimate is published through ROS2 as:

```text
PoseWithCovarianceStamped
```

which Nav2 consumes during planning.

---

## 4. Uncertainty-Aware Navigation with Nav2

Nav2 integrates SCOPE uncertainty estimates into trajectory planning.

### Behavior Policy

| Confidence | Navigation Behavior |
|---|---|
| High | Normal velocity and aggressive planning |
| Medium | Conservative trajectory generation |
| Low | Replanning + reduced speed |

---

## Cost Function Modulation

Trajectory cost is modified as:

```math
J_{total} =
J_{path}
+
\lambda_u \cdot U(x)
```

Where:

- `J_path` → standard path cost
- `U(x)` → uncertainty penalty
- `λ_u` → uncertainty weighting coefficient

---

## Adaptive Velocity Control

Velocity is reduced in uncertain regions:

```math
v =
v_{max} \cdot e^{-k \cdot \mathrm{trace}(\Sigma)}
```

Where:

- `v_max` → maximum robot velocity
- `k` → decay coefficient

This creates confidence-aware navigation behavior.

---

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

## Covariance Fusion

Composite uncertainty:

```math
\Sigma_{total} =
\Sigma_{slam}
+
\lambda_d \Sigma_{dynamic}
```

This enables integration of semantic uncertainty from dynamic objects.

---

# Software Stack

| Component | Framework | Role |
|---|---|---|
| SLAM Front-End | ORB-SLAM3 | Local visual SLAM |
| SLAM Back-End | COVINS-G | Global map fusion |
| Uncertainty Module | SCOPE | Pose covariance estimation |
| Navigation | Nav2 | Uncertainty-aware planning |
| Middleware | ROS2 + Cyclone DDS | Communication |
| Simulation | Gazebo Classic | Multi-robot simulation |
| Place Recognition | DBoW2 | Loop closure detection |

---

# Hardware Platform

| Component | Specification |
|---|---|
| Mobile Robots | 3× TurtleBot4 |
| RGB-D Cameras | 3× Intel RealSense D435i |
| Onboard Compute | Raspberry Pi 4 (8GB RAM) |
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

- Gazebo Classic simulation
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

> **In Progress:** A full independent reimplementation of the system is currently under development in Gazebo + ROS2.

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

```text
RGB Image
    │
    ▼
SuperPoint Keypoints
    │
    ▼
LightGlue Matching
    │
    ▼
PnP Pose Estimation
```

Expected improvements:

- More robust tracking
- Fewer outlier matches
- Improved loop closure reliability

---

## 3. Semantic Uncertainty for Dynamic Environments

SCOPE estimates geometric uncertainty but not scene dynamics.

The reimplementation adds semantic uncertainty using:

- YOLOv8n object detection
- Dynamic occupancy scoring

### Dynamic Occupancy Score

```math
D =
\sum_i
A_i \cdot c_i
```

Where:

- `A_i` → normalized bounding box area
- `c_i` → detection confidence

---

## Composite Uncertainty

Final uncertainty:

```math
U_{final} =
\alpha \cdot U_{geometric}
+
\beta \cdot U_{semantic}
```

This allows Nav2 to reason about both:

- Localization uncertainty
- Scene instability

---

# Key Contributions

- Collaborative multi-agent visual SLAM
- Shared globally optimized pose graph
- Cross-agent loop closure
- Uncertainty-aware navigation
- Real-world TurtleBot4 deployment
- Semantic uncertainty extension
- Decentralized SLAM architecture proposal

---

