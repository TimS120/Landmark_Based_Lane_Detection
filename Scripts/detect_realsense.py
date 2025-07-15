import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# Load YOLO model
model = YOLO("runs/detect/train2/weights/best.pt")

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipeline.start(config)

# Depth scale
profile = pipeline.get_active_profile()
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Camera intrinsics for 3D calculation
color_stream = profile.get_stream(rs.stream.color)
intr = color_stream.as_video_stream_profile().get_intrinsics()
fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy

def get_cone_depth(depth_image, x1, y1, x2, y2):
    """Sample a lower region of the bounding box, exclude zeros and outliers."""
    bbox_height = y2 - y1
    roi_y_start = int(y2 - 0.2 * bbox_height)
    roi_y_end = y2
    roi_x_start = int(x1 + 0.25 * (x2 - x1))
    roi_x_end = int(x2 - 0.25 * (x2 - x1))
    depth_roi = depth_image[roi_y_start:roi_y_end, roi_x_start:roi_x_end]

    depth_vals = depth_roi.flatten()
    depth_vals = depth_vals[depth_vals > 0]
    if depth_vals.size == 0:
        return None
    lower = np.percentile(depth_vals, 10)
    upper = np.percentile(depth_vals, 90)
    filtered = depth_vals[(depth_vals >= lower) & (depth_vals <= upper)]
    if filtered.size == 0:
        return None
    return np.median(filtered) * depth_scale

def compute_3d(x, y, depth):
    # Convert pixel (x, y) and depth to metric 3D coordinates
    X = (x - cx) * depth / fx
    Y = (y - cy) * depth / fy
    Z = depth
    return np.array([X, Y, Z])

# Buffer for temporal smoothing
N_FRAMES = 5
history_buffer = []

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        height, width = color_image.shape[:2]

        results = model.predict(color_image, conf=0.5, verbose=False)
        annotated_frame = color_image.copy()
        left_cones, right_cones = [], []

        # Collect all cones with position and depth
        all_cones = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                cls_id = int(box.cls[0])
                confidence = float(box.conf[0])  # Get confidence for this detection
                depth_val = get_cone_depth(depth_image, x1, y1, x2, y2)
                if depth_val is None:
                    continue
                cone = {
                    'pos': (cx, cy),
                    'depth': depth_val,
                    'cls': cls_id,
                    'rect': (x1, y1, x2, y2),
                    'conf': confidence
                }
                all_cones.append(cone)

        # Assign cones to left/right by x-position (not fixed color)
        mid_x = width // 2
        left_cones = [c for c in all_cones if c['pos'][0] < mid_x]
        right_cones = [c for c in all_cones if c['pos'][0] >= mid_x]

        # Annotation: keep color annotation by class for visualization
        for cone in all_cones:
            x1, y1, x2, y2 = cone['rect']
            cx, cy = cone['pos']
            depth_val = cone['depth']
            conf = cone['conf']
            cls_id = cone['cls']
            if cls_id == 0:  # Red
                color = (0, 255, 0)
            else:  # Yellow
                color = (0, 0, 255)
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(annotated_frame, f"{depth_val:.2f}m | Conf: {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # 3D Euclidean pairing for path generation
        cone_pairs = []
        used_right_indices = set()
        for l in left_cones:
            l_3d = compute_3d(l['pos'][0], l['pos'][1], l['depth'])
            min_dist, matched_r_idx = float('inf'), None
            for idx, r in enumerate(right_cones):
                if idx in used_right_indices:
                    continue
                r_3d = compute_3d(r['pos'][0], r['pos'][1], r['depth'])
                dist = np.linalg.norm(l_3d - r_3d)
                if dist < min_dist:
                    min_dist, matched_r_idx = dist, idx
            if matched_r_idx is not None:
                used_right_indices.add(matched_r_idx)
                r = right_cones[matched_r_idx]
                center_x = int((l['pos'][0] + r['pos'][0]) / 2)
                center_y = int((l['pos'][1] + r['pos'][1]) / 2)
                avg_depth = (l['depth'] + r['depth']) / 2
                cone_pairs.append({'center': (center_x, center_y), 'y': center_y, 'depth': avg_depth})

        # Sort by increasing depth for true path "forward"
        cone_pairs.sort(key=lambda pair: pair['depth'])
        centerline_points = [pair['center'] for pair in cone_pairs]

        # Temporal smoothing for path
        if centerline_points:
            history_buffer.append(centerline_points)
            if len(history_buffer) > N_FRAMES:
                history_buffer.pop(0)
            # Pad if needed for averaging
            max_len = max(len(pts) for pts in history_buffer)
            padded = []
            for pts in history_buffer:
                if len(pts) < max_len:
                    # Pad with last point
                    pts = pts + [pts[-1]] * (max_len - len(pts))
                padded.append(pts)
            smoothed_points = np.mean(np.array(padded), axis=0).astype(int)
            smoothed_points = [tuple(pt) for pt in smoothed_points]
        else:
            smoothed_points = []

        # Draw path and "START" at first pair
        if len(smoothed_points) >= 2:
            cv2.polylines(annotated_frame, [np.array(smoothed_points)], False, (0, 255, 0), 3)
            # Draw start marker at the first pair
            cv2.circle(annotated_frame, smoothed_points[0], 8, (0, 255, 0), -1)
            cv2.putText(annotated_frame, "START", (smoothed_points[0][0] + 10, smoothed_points[0][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Depth colormap visualization
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
        )

        combined_view = np.hstack((annotated_frame, depth_colormap))
        cv2.imshow("Detection + Depth Map", combined_view)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()