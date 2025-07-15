# Literature Review: Landmark-Based Lane Detection Using Traffic Cones (YOLOv8 + RealSense)

## Introduction

Lane detection is a core task in autonomous navigation, enabling vehicles to follow predefined paths safely and efficiently. Traditional methods rely on painted lane markings, but many experimental or indoor environments—such as model city labs—lack these markings.

To overcome this, landmark-based lane detection uses visually distinct objects like traffic cones as movable, high-contrast proxies for lane boundaries. This approach enables dynamic path computation without relying on GPS or HD maps.

Our project develops a real-time cone-based lane detection system using an Intel RealSense RGB-D camera and a custom-trained YOLOv8 model to detect cones and estimate their 3D positions. Paired cones define left and right boundaries, generating a smoothed centerline for autonomous navigation.

## Literature Highlights

- **Dhall et al. (2019)**: Proposed a real-time 3D traffic cone detection pipeline using monocular camera and keypoint regression, showing cones as robust landmarks without lane paint [1].
- **Paleczny et al. (2024)**: Combined ZED stereo camera and LiDAR with YOLO detection in a Formula Student driverless car, demonstrating high accuracy in cone localization through RGB + depth fusion [2].
- **Gong et al. (2022)**: Enhanced cone detection by integrating YOLOv3 with attention modules and LiDAR clustering, improving range and accuracy [3].
- **Watanabe et al. (2009)**: Demonstrated lane detection using roadside structures as alternatives to markings, validating landmark-based approaches [4].
- **Jhong et al. (2023)**: Developed density-aware semantic fusion frameworks for LiDAR-camera 3D object detection, confirming benefits of sensor fusion for robust spatial understanding [5].
- **Peng et al. (2023)**: Showed 3D object detection and tracking via solid-state LiDAR and RGB camera integration, relevant for landmark-reliant navigation systems [6].

These studies collectively support visual landmark detection as a reliable alternative to lane markings, especially in unstructured or indoor environments. They highlight advantages of combining vision with depth sensing for spatial accuracy and real-time performance.

## Method Choice & Justification

- **Cone-based landmark lane detection** using Intel RealSense RGB-D camera and custom-trained YOLOv8.
- **Advantages:**
  - Proven cone-based navigation success [1][2].
  - Affordable and lightweight RGB-D sensor providing color + depth data.
  - YOLOv8 ensures fast, accurate real-time detection suitable for embedded systems.
  - Independence from traditional lane markings matches model city lab conditions [4].

## Limitations

- Depth sensing range limited (~8m) and affected by bright outdoor light.
- Environmental noise and occlusions may reduce detection confidence.
- Single-sensor dependency (no LiDAR fusion) may reduce robustness in adverse conditions.

## Future Work

- Integrate LiDAR or radar for improved 3D localization.
- Apply temporal filtering and SLAM for centerline stability.
- Extend to closed-loop path following control.
- Train on larger, diverse datasets to generalize detection.

## References

[1] Dhall et al., "Real-time 3D Traffic Cone Detection for Autonomous Driving," IEEE IV Symposium, 2019.  
[2] Paleczny et al., "Cone Detection System with Camera and LiDAR," IEEE URC, 2024.  
[3] Gong et al., "Perception System of a Formula Student Driverless Car," ICRCA, 2022.  
[4] Watanabe et al., "Lane Detection with Roadside Structure," IEEE IV Symposium, 2009.  
[5] Jhong et al., "Density-Aware and Semantic-Guided Fusion for 3D Object Detection," IEEE Sensors Journal, 2023.  
[6] Peng et al., "3D Object Detection Using Solid-State LiDAR and RGB Camera," IEEE Sensors Journal, 2023.

---

*All reference papers are uploaded to the GitHub repository for full access.*
 
