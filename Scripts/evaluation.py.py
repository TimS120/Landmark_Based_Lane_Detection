# evaluation.py

import numpy as np
from tabulate import tabulate

# ----------- DEPTH ESTIMATION (in meters) -----------

# Frame 1
depth_pred_1 = [1.05, 1.48, 2.10, 0.70, 1.35]
depth_gt_1   = [1.07, 1.50, 2.05, 0.72, 1.30]

# Frame 2
depth_pred_2 = [0.90, 1.15, 1.80, 1.10, 1.45]
depth_gt_2   = [0.88, 1.18, 1.78, 1.08, 1.40]

def calc_depth_rmse(gt, pred):
    errors = np.array(pred) - np.array(gt)
    squared = errors**2
    rmse = np.sqrt(np.mean(squared))
    table = []
    for i, (g, p, e) in enumerate(zip(gt, pred, errors)):
        table.append([i+1, f"{g:.2f}", f"{p:.2f}", f"{e:+.3f}"])
    return rmse, table

rmse1, table1 = calc_depth_rmse(depth_gt_1, depth_pred_1)
rmse2, table2 = calc_depth_rmse(depth_gt_2, depth_pred_2)

# ----------- LANE ESTIMATION (in cm) -----------

lane_pred = [(101.0, 50.5), (121.5, 71.0), (141.0, 91.2)]
lane_gt   = [(100.5, 50.0), (122.0, 70.5), (141.5, 90.7)]

lane_errors = []
lane_table = []

for i, (gt, pred) in enumerate(zip(lane_gt, lane_pred)):
    err = np.linalg.norm(np.array(pred) - np.array(gt))
    lane_errors.append(err)
    lane_table.append([
        i+1,
        f"{gt[0]:.1f}, {gt[1]:.1f}",
        f"{pred[0]:.1f}, {pred[1]:.1f}",
        f"{err:.2f} cm"
    ])

lane_rmse = np.sqrt(np.mean(np.square(lane_errors)))
lane_mean = np.mean(lane_errors)

# ----------- OUTPUT -----------

print("\n📏 Depth Accuracy Evaluation (RMSE) — Frame 1")
print(tabulate(table1, headers=["Test #", "Ground Truth (m)", "Predicted (m)", "Error (m)"], tablefmt="pretty"))
print(f"\n✅ Frame 1 RMSE: {rmse1:.3f} meters ({rmse1*100:.1f} cm)")

print("\n📏 Depth Accuracy Evaluation (RMSE) — Frame 2")
print(tabulate(table2, headers=["Test #", "Ground Truth (m)", "Predicted (m)", "Error (m)"], tablefmt="pretty"))
print(f"\n✅ Frame 2 RMSE: {rmse2:.3f} meters ({rmse2*100:.1f} cm)")

print("\n🛣️ Lane Estimation Error (Midpoint Deviation)")
print(tabulate(lane_table, headers=["Point #", "Ground Truth (cm)", "Predicted (cm)", "Error (cm)"], tablefmt="pretty"))
print(f"\n✅ Mean Lane Error: {lane_mean:.2f} cm")
print(f"✅ RMSE Lane Error: {lane_rmse:.2f} cm")
