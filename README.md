# 🚗 Landmark-Based Lane Detection Using Traffic Cones (YOLOv8 + RealSense)
This project was initially designed for the model city, however it is continued with the aim of getting a pipeline for the autonomous driving disciplin of Formula Student. Information about model city colloquium are here directly below, information about continued implementation are starting from [here](#formula-student-problem-switch).

This project implements a cone-based **landmark lane detection system** designed for autonomous navigation in a **model city lab environment**. It uses an **Intel RealSense RGB-D camera** and a **custom-trained YOLOv8 model** to detect cones, estimate depth, and form a drivable centerline path in real time.

# 🧪 Research Question
How effectively can a landmark-based lane detection system using a RealSense camera and YOLOv8 cone detection generate a reliable drivable path for small autonomous vehicles in a model city environment without relying on traditional lane markings?

# 📖 Introduction
Accurate and robust lane detection is a critical component of autonomous vehicle navigation, especially in environments where traditional road markings are unavailable, inconsistent, or purposely excluded. In small-scale research setups like model cities-lab environments built for developing and testing autonomous driving algorithms-traditional lane lines are often replaced with simpler landmarks such as colored traffic cones.

This project presents a landmark-based lane detection system that leverages YOLOv8 object detection and Intel RealSense RGB-D sensing to identify traffic cones and generate a drivable path. The cones act as visual proxies for lane boundaries. By combining RGB detection and depth-based 3D localization, the system dynamically pairs cones based on spatial proximity and infers a centerline path in real time.

Unlike conventional vision-based approaches that rely on white lane markings or digital maps, this system operates independently of structured roads and instead focuses on environment-derived landmarks, making it ideal for both educational platforms and constrained robotic environments.

The goal is to evaluate how reliably such a system can estimate a lane path using only visual and depth cues, without relying on external infrastructure like GPS or SLAM.

<div align="center">
    <img src="Test_results/image_intro.jpeg">
   <br>
<em>Figure: Model city environment.</em>
</div>

---



## Literature Review Summary
The literature strongly supports using traffic cones as robust lane landmarks in autonomous driving, especially where lane paint is unavailable. Studies validate cone detection with RGB-D sensors and advanced object detectors like YOLO, emphasizing benefits of sensor fusion for accurate 3D localization.

Our approach builds on these findings by integrating an Intel RealSense camera with YOLOv8 for real-time, robust lane path estimation, suitable for small autonomous vehicles operating in experimental or indoor environments.

👉 For detailed insights and full references, view the [Literature_Review.md](https://github.com/Scientific-Colloquiom/Landmark_Based_Lane_Detection/blob/main/Referense%20papers/readme.md).
 

---


# Pipeline Overview
## 📷 1. Sensor Input - RealSense Camera

**Streams Captured:**
- **RGB Frame:** 640x480 @ 30 FPS  
- **Depth Frame:** 640x480 @ 30 FPS

**Tech Stack:**
- Captured using `pyrealsense2`  
- `depth_scale` used to convert depth values to meters

## 🧠 2. Object Detection - YOLOv8
**Model:**
- YOLOv8 (Ultralytics), custom-trained on [Roboflow](https://universe.roboflow.com/transfer-learning-h9tt8/scientific-colloquim/dataset/4)  
- **Classes:**
  - `Class 0`: **Cone-red** (left lane boundary)
  - `Class 1`: **Cones-yellow** (right lane boundary)

**Output:**
- Bounding boxes  
- Depth in meters  
- Confidence scores

<div align="center">
    <img src="Test_results/test_image_obj.png">
   <br>
<em>Figure: Object Detection.</em>
</div>

## 📐 3. Depth Estimation (3D Localization)
**Steps:**
- Extract lower-central ROI from each bounding box  
- Filter:
  - Invalid (`= 0`) depth values  
  - Outliers using 10th-90th percentile  
- Compute **median depth**  
- Convert to meters using RealSense `depth_scale`

## 🧭 4. Landmark Categorization
- Cones classified as:
  - `left_cones` - Red cones
  - `right_cones` - Yellow cones

Each cone includes:
- 2D image position  
- Estimated depth  
- 3D coordinates  
- Bounding box and confidence

## 🧮 5. Cone Pairing and Path Formation
**Pairing Rule:** Match each **left cone with its nearest right cone** using **3D Euclidean distance**

For each valid pair:
- Compute centerpoint:  
  `(x, y) = ((x₁ + x₂)/2, (y₁ + y₂)/2)` 
   - where `(cx, cy)` are the 2D centers of the matched left and right cones 
- Average average depth
- Store for centerline sorting and smoothing

## 📏 6. Centerline Path Generation
- Path points are **sorted by increasing average depth**  
- Temporal smoothing applied using a sliding window of recent frames  
- Smoothed midpoints are connected with a **polyline**, and the first point is labeled `"START"`

### 🔧 Parameters Used:
| Parameter             | Value / Logic                            |
|----------------------|------------------------------------------|
| ROI Sampling Area     | Bottom 20% of bounding box               |
| Depth Filtering       | 10th-90th percentile, median computed    |
| Pairing Rule          | Nearest in `y` direction (vertical axis) |
| Sorting Criterion     | By average depth (ascending)             |
| Visualization         | OpenCV (bounding boxes, lines, markers)  |

## 🖼️ 7. Output Visualization
- **Left Panel:** Annotated RGB frame  
  - Cone bounding boxes, class labels, depth (in meters)  
  - Centerline polyline and `"START"` marker  
- **Right Panel (optional):** Depth image (JET color map)  
- Designed for real-time visual feedback

---

# 📦 Dataset Details
The dataset was **custom-collected in "Model City" environment** specifically designed for small autonomous vehicle experiments. It includes labeled images of **traffic cones** used as visual lane markers, captured from a RealSense RGB-D camera.

### 📁 Dataset Split:
- **Training**: 123 images  
- **Validation**: 20 images  
- **Test**: 15 images  
- **Classes**:  
  - `Class 0`: Cone-red (left lane marker)  
  - `Class 1`: Cones-yellow (right lane marker)

### 📝 YOLOv8 Dataset YAML (`data.yaml`)
```yaml
train: ../train/images
val: ../valid/images
test: ../test/images

nc: 2
names: ['Cone-red', 'Cones-yellow']

roboflow:
  workspace: transfer-learning-h9tt8
  project: scientific-colloquim
  version: 4
  license: CC BY 4.0
  url: https://universe.roboflow.com/transfer-learning-h9tt8/scientific-colloquim/dataset/4
```

## 📊 Model Performance Summary
### **Validation Set Results:**
| Class          | Images | Instances | Precision | Recall | mAP@0.5 | mAP@0.5:0.95 |
|----------------|--------|-----------|-----------|--------|---------|--------------|
| All Classes    | 20     | 130       | **0.929** | **0.909** | **0.884** | **0.728** |
| Cone-red       | 19     | 53        | 0.940     | 0.884  | 0.864   | 0.718        |
| Cones-yellow   | 19     | 77        | 0.919     | 0.935  | 0.905   | 0.737        |

### **Speed Per Image:**
- **Preprocessing:** 11.3 ms  
- **Inference:** 178.6 ms  
- **Postprocessing:** 26.1 ms  

# Confusion Matrix

<div align="center">
    <img src="Test_results/confusion_matrix.png">
   <br>
<em>Figure: Confusion Matrix.</em>
</div>

---

# 📈 Evaluation and Results
To assess the performance of our cone-based landmark lane detection system, we use four key evaluation metrics:

### 🔹 Detection Accuracy (YOLOv8)
We evaluate the cone detection capability of our YOLOv8 model using standard object detection metrics. These values reflect how accurately the model identifies and classifies red and yellow cones in the scene.

- **Precision:** 92.9%  
- **Recall:** 90.9%  
- **mAP@0.5:** 88.4%  
- **mAP@0.5:0.95:** 72.8%

The model demonstrates strong overall detection performance, with high precision and recall across both classes. Confusion matrix results further confirm effective separation between red and yellow cones with minimal false positives.

### 🔹 Depth Accuracy (RMSE)
To validate the system's depth estimation, we compared the predicted depth values from the RealSense camera with ground-truth distances measured manually. Two sets of frames were evaluated with five measurement and Root Mean Square error was used.

- **Frame 1:** RMSE = 0.035 m 
- **Frame 2:** RMSE = 0.030 m 
  
<div align="center">
    <img src="Test_results/tets_image4.png">
    <br>
<em>Figure: Frame 1.</em>
</div>

<br>
<br>
<br>

<div align="center">
    <img src="Test_results/test_image5.png">
    <br>
<em>Figure: Frame 2.</em>
</div>

The RMSE values indicate reliable depth prediction, with average errors under 4 cm. These results confirm that our lower-central ROI sampling method yields accurate depth even under varied ranges.

### 🔹 Lane Estimation Error (Midpoint Deviation)
To assess how well the system can form the lane centerline, we measured three deviation between the predicted midpoints (between left and right cones) and manually marked ground-truth midpoints on the track.

- **Mean Error:** 0.071 m  
- **RMSE:** 0.071 m

This low deviation confirms that the system can accurately estimate the drivable centerline using 3D cone pairing. The consistent sub-centimeter error across points supports the robustness of the approach.

*📋 For detailed tables and raw measurements, refer to the evaluation script outputs and screenshots.*

<div align="center">
    <img src="Test_results/test_image_result.png">
  <br>
<em>Figure: Real-time lane detection terminal result.</em>
</div>

---

## ✅ Conclusion
We developed a real-time lane detection system using **traffic cones**, an **Intel RealSense RGB-D camera**, and a **YOLOv8 object detector** for use in model city lab environments. By pairing detected cones using depth data, the system generates an accurate drivable centerline without relying on traditional lane markings.

Through careful cone pairing and depth-based spatial reasoning, the system achieves:

- **High object detection performance** (mAP@0.5: **88.4%**)  
- **Accurate depth estimation** (RMSE < **0.04 m**)  
- **Reliable centerline prediction** (lane deviation RMSE ≈ **0.071 m**)

These results validate the feasibility and robustness of using **simple, color-coded landmarks** for real-time lane inference in constrained robotic or educational environments. While this implementation does not include vehicle control or motion planning, it lays a strong foundation for future integration with autonomous navigation pipelines.

Ultimately, this approach offers a **cost-effective, vision-based alternative** to conventional lane detection, ideal for small-scale autonomous systems in structured or semi-structured environments.

---

## 📌 Notes
- This project does **not integrate path-following or control**. It is focused solely on **real-time lane detection** using traffic cones.
- Designed for use in controlled indoor "model city" environments.



---
# Formula student problem switch
- The aim is to get something like [this](https://www.youtube.com/watch?v=ZPVMYiw5ucc) or [this](https://www.youtube.com/watch?v=FbKLE7uar9Y).
  - [Rules](https://www.formulastudent.de/fsg/rules)
  - Maybe the most interesting points:
    - Usage of GPS is allowed, however maybe not reliable
    - Cones representing the track outlines
- Literature of approaches to do that can be found [here](https://arxiv.org/pdf/2210.10933), [here](https://arxiv.org/pdf/1909.00119) and [here](https://arxiv.org/pdf/2408.06113).
- A thorough report about the trajectory/ path planning can be found [here](https://static1.squarespace.com/static/5e2a78aea2dc434ac475b5a4/t/615ee7754fb3ae56ba011134/1731809144631/Adam+Slomoi+-+Path+Planning+and+Control+in+an+Autonomous+Formula+Student+Vehicle.pdf).

# Developed pipeline & Problem division
## Problem division
The whole Cone detection and track driving problem can be divided into the following sub-problems:
- Pylon/ Cone detection
- Mapping + Localization
- Trajectory planning
- Route creation and optimization
- (Path following, Vehicle state determination, etc.)

## Pipeline
The pylon/ cone detection is done in the [pylon detection node](./landmark_based_lane_detection_pkg/landmark_based_lane_detection_pkg/pylon_detection.py) by using a deep learning model, described [here](#model-performance-summary). Since this was only designed for the model city, it was tried to use instead of SLAM as Formula Student teams are using the OptiTrack system to create directly the map from the detections. Therefore this node does already transform with OptiTrack's information the positions into map frame. With the [map creation node](./landmark_based_lane_detection_pkg/landmark_based_lane_detection_pkg/map_creation.py) it was tried to use simple Kalman filters to map the detections into map frame. This does however work not that good, since small heading changes are sometimes not detected by OptiTrack as well as the used localization node does introduce a lag. This causes that a very simple translation of cone positions into map frame is not possible and SLAM is also needed here (probably).\
The data flow is as following:\
((Camera image --> pylon detection node) + (OptiTrack pose --> robus_localization)) --> map creation node

The further parts of the full problem are therefor only developed in a simulated environment, where a [map creation simulation node](./landmark_based_lane_detection_pkg/landmark_based_lane_detection_pkg/simulate_map_creation.py) simulates the mapping process by reading in a predefined map (cone positions and cone type (left/ right)). The map is then produced by adding seen cones into the output map which is then sent via a ros topic to another nodes.\
The trajectory to drive is then created by the [trajectory creation node](./landmark_based_lane_detection_pkg/landmark_based_lane_detection_pkg/trajectory_creation.py). It's using the Dynamic Window Approach (DWA) to find drivable trajectories. The previous "find the centerpoint for every cone-pair" was removed, since sometimes only one cone of a pair is seen and therefore this algorithm can produce wrong output.\
The trajectory will then be driven by the [driving simulation node](./landmark_based_lane_detection_pkg/landmark_based_lane_detection_pkg/simulate_driving.py), which simply follows the trajectory. The ouput of that is a pose where the ego vehicle is currently - this output is then again used in the map creation node, which checks, which cones are visibile and adds them to the output map, if they were not visible before.\
The [route optimization node](./landmark_based_lane_detection_pkg/landmark_based_lane_detection_pkg/route_optimization.py) will detect a loop closure and then create a full optimized (here only smoothed) path and output it.\
In a later stage, when every node is further developed and works fine, the optimized path can be used instead of the trajectory planning node for further laps after the first one.  
The pipeline can be seen here:
<div align="center">
    <img src="readme_utils/simulated_pipeline_rqt.png">
</div>
The output of it can be seen here:
<div align="center">
    <img src="readme_utils/simulated_pipeline_output.png">
</div>

Note: The modelcity setput can be launched with [this](./landmark_based_lane_detection_pkg/launch/pylone_detection_launch.py) launch file.
The simulation setup can be launched with [this](./landmark_based_lane_detection_pkg/launch/simulate_trajectory_finding_launch.py) launch file.


# Behaviour/ Problems
- The here stated problems are arising in the modelcity setup and will be probably replaced by other problems in a real-world application
- RGB+Depth image causes high data traffic (or computational need) --> Low fps of only up to 7 fps [in yellow car at least (measured IN the yellow car)]. With full resolution (needed for wider field of view) the fps will further decrease

- The accuracy of the object detection is not that high in the situation where the car stands on its tires (training data mostely recorded while holding the car in the air)

- The heading of the ego vehicle is slow and is lacking behind (it updates only when the car changes also its position strong enough), so a simulation of localization (SLAM part) with custom mapping with OptiTrack information cannot be made --> Problem comes from OptiTrack + localization node [Used the [robus](https://git.hs-coburg.de/ROBUS/robus_localization) version] partly. Using the [commented out version](https://git.hs-coburg.de/ROBUS/robus_localization/src/commit/60e374683cf754f8d5f47832bc68bb4cc463731c/robus_localization_pkg/robus_localization.py#L32) with lower covariances solves this at least a partly.


# Next steps
- Solving the modelcity related problems:
  - Find way to have higher frame rates while having the full field of view

  - Train model with data which was recorded with the camera of the car in situation where the car stands *on* the ground/ drives around by its own. Use more data to increase the performance of the model

  - Make the ego heading more responsive/ precise
  - Tune the map creation script in a way, so that it works with this behaviour of the localization output
    - Or use SLAM, because currently we assume that we know the position of the ego vehicle with OptiTrack + localization node and only want to do mapping. However apparently the position (more precise the heading) is not correct/ precise enough, so we have to localize the ego pose and do mapping of the pylones
    - If you try to use the approach of using the pose which comes from the robus-localization node, then the map-creation script must also be changed in a way, so that it does not forget the already detected pylones which are no longer detecable (because they are e.g. behind the car) --> But I guess this approach of using OptiTrack for position and heading is anyway not that good, because e.g. GPS does also not provide any heading but only a position and the gps-signals can be occluded, so another method must be used in real world applications like Formula Student

- Switching to solving the Formula Student problem:
  - Bring the simulated pipeline into real world application
    - Using for that changed problem state suitable new algorithms and interfaces
