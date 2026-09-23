#!/usr/bin/env python3
"""RGB-D特徴から旧・新source frame間の回転を推定し、odom姿勢差と比較する。

これは姿勢推定を検証する独立な真値ではない。RGB特徴対応、depth/RGB画素整列、fallback
intrinsicsの誤差を含むため、odom回転と大きく矛盾するかをみる補助診断として使う。
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


FX, FY = 574.28826904, 574.28826904
CX, CY = 354.75085449, 215.23262024
CAMERA_MATRIX = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]])


def rotation_from_rpy(rpy_deg):
    """ROS固定軸roll/pitch/yawからworld/body回転行列を作る。"""
    roll, pitch, yaw = np.radians(rpy_deg)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def angle_axis_degrees(rotation):
    """回転行列を角度軸ベクトルのdegree表現に変換する。"""
    vector, _ = cv2.Rodrigues(rotation)
    return np.degrees(vector.reshape(3))


def rotation_error_degrees(first, second):
    """2つの回転行列間の最小角距離を返す。"""
    relative = first.T @ second
    cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def odom_reprojection_errors(old_rgb_path, new_rgb_path, old_depth_path,
                             new_depth_path, old_source, new_source):
    """TF姿勢から予測したRGB feature位置と実画像対応のずれを調べる。"""
    old_rgb = cv2.imread(str(old_rgb_path), cv2.IMREAD_COLOR)
    new_rgb = cv2.imread(str(new_rgb_path), cv2.IMREAD_COLOR)
    old_depth = cv2.imread(str(old_depth_path), cv2.IMREAD_UNCHANGED)
    new_depth = cv2.imread(str(new_depth_path), cv2.IMREAD_UNCHANGED)
    if any(image is None for image in (old_rgb, new_rgb, old_depth, new_depth)):
        return {"status": "image missing"}
    if not all("x_m" in source["base_pose"] and "y_m" in source["base_pose"]
               for source in (old_source, new_source)):
        return {"status": "source-time x/y pose missing"}

    old_gray = cv2.cvtColor(old_rgb, cv2.COLOR_BGR2GRAY)
    new_gray = cv2.cvtColor(new_rgb, cv2.COLOR_BGR2GRAY)
    mask_old = ((old_depth >= 500) & (old_depth <= 5000)).astype(np.uint8) * 255
    mask_new = ((new_depth >= 500) & (new_depth <= 5000)).astype(np.uint8) * 255
    # 画像へ描いたタイトルと旧・新画素markerは特徴抽出から除く。
    mask_old[:45, :] = 0
    mask_new[:45, :] = 0
    for mask, source in ((mask_old, old_source), (mask_new, new_source)):
        cv2.circle(mask, tuple(source["pixel_uv"]), 35, 0, -1)
    orb = cv2.ORB_create(nfeatures=3000, scaleFactor=1.2, nlevels=8)
    key_old, desc_old = orb.detectAndCompute(old_gray, mask_old)
    key_new, desc_new = orb.detectAndCompute(new_gray, mask_new)
    if desc_old is None or desc_new is None:
        return {"status": "feature descriptors missing"}
    matches = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(desc_old, desc_new, k=2)
    ratio_matches = [a for a, b in matches if a.distance < 0.78 * b.distance]
    old_pixels = np.float32([key_old[m.queryIdx].pt for m in ratio_matches])
    new_pixels = np.float32([key_new[m.trainIdx].pt for m in ratio_matches])
    homography_inliers = np.zeros(len(ratio_matches), dtype=np.bool_)
    if len(ratio_matches) >= 4:
        _, homography_mask = cv2.findHomography(
            old_pixels, new_pixels, cv2.RANSAC, 5.0
        )
        if homography_mask is not None:
            homography_inliers = homography_mask.ravel().astype(np.bool_)

    def camera_pose_world(source):
        """base poseとURDF OAK RGB optical mountからodom->cameraを作る。"""
        pose = source["base_pose"]
        world_base_rotation = rotation_from_rpy(
            [pose["roll_deg"], pose["pitch_deg"], pose["yaw_deg"]]
        )
        base_camera_rotation = rotation_from_rpy([-90.0, 0.0, -90.0])
        world_camera_rotation = world_base_rotation @ base_camera_rotation
        world_base_translation = np.array([pose["x_m"], pose["y_m"], pose["z_m"]])
        world_camera_translation = world_base_translation + world_base_rotation @ np.array(
            [0.0, 0.0, 0.28]
        )
        return world_camera_rotation, world_camera_translation

    old_rotation, old_translation = camera_pose_world(old_source)
    new_rotation, new_translation = camera_pose_world(new_source)
    residuals, vectors = [], []
    for match_index, match in enumerate(ratio_matches):
        # RGB画像上で支配的な幾何変換に合う特徴だけを使う。これも実景の単一平面を
        # 仮定するため、odomの正否を単独で証明するものではない。
        if not homography_inliers[match_index]:
            continue
        u0, v0 = np.rint(key_old[match.queryIdx].pt).astype(int)
        u1, v1 = np.rint(key_new[match.trainIdx].pt).astype(int)
        depth0 = float(old_depth[v0, u0]) * 0.001
        depth1 = float(new_depth[v1, u1]) * 0.001
        if not (0.5 <= depth0 <= 5.0 and 0.5 <= depth1 <= 5.0):
            continue
        point_old = np.array([(u0 - CX) * depth0 / FX,
                              (v0 - CY) * depth0 / FY, depth0])
        point_world = old_rotation @ point_old + old_translation
        point_new = new_rotation.T @ (point_world - new_translation)
        if point_new[2] <= 0.0:
            continue
        predicted = np.array([FX * point_new[0] / point_new[2] + CX,
                              FY * point_new[1] / point_new[2] + CY])
        residual = np.array([u1, v1], dtype=np.float64) - predicted
        if np.all(np.isfinite(residual)):
            residuals.append(float(np.linalg.norm(residual)))
            vectors.append(residual)
    if not residuals:
        return {"status": "no valid RGB-D feature correspondences",
                "ratio_matches": len(ratio_matches)}
    residuals = np.asarray(residuals)
    vectors = np.asarray(vectors)
    return {
        "status": "computed; correspondence/calibration errors remain",
        "ratio_matches": len(ratio_matches),
        "rgb_homography_inlier_count": int(homography_inliers.sum()),
        "depth_valid_correspondences": int(residuals.size),
        "median_reprojection_error_px": float(np.median(residuals)),
        "p90_reprojection_error_px": float(np.percentile(residuals, 90)),
        "fraction_under_10px": float(np.mean(residuals < 10.0)),
        "median_dx_px": float(np.median(vectors[:, 0])),
        "median_dy_px": float(np.median(vectors[:, 1])),
        "warning": "予測は旧候補depth、EEPROM fallback intrinsics、URDF外部姿勢、近傍時刻RGBに依存。誤対応やシーンdepth不整合を除けない。",
    }


def estimate_pair(old_rgb_path, new_rgb_path, old_depth_path, new_depth_path,
                  old_source, new_source):
    """ORB対応点のうち両frameでdepthが得られる点からPnP回転を推定する。"""
    old_rgb = cv2.imread(str(old_rgb_path), cv2.IMREAD_COLOR)
    new_rgb = cv2.imread(str(new_rgb_path), cv2.IMREAD_COLOR)
    old_depth = cv2.imread(str(old_depth_path), cv2.IMREAD_UNCHANGED)
    new_depth = cv2.imread(str(new_depth_path), cv2.IMREAD_UNCHANGED)
    if any(image is None for image in (old_rgb, new_rgb, old_depth, new_depth)):
        raise FileNotFoundError("RGB/depth input image missing")
    if old_rgb.shape[:2] != old_depth.shape or new_rgb.shape[:2] != new_depth.shape:
        raise ValueError("RGBとdepthの解像度が一致しない")

    old_gray = cv2.cvtColor(old_rgb, cv2.COLOR_BGR2GRAY)
    new_gray = cv2.cvtColor(new_rgb, cv2.COLOR_BGR2GRAY)
    old_mask = ((old_depth >= 500) & (old_depth <= 5000)).astype(np.uint8) * 255
    new_mask = ((new_depth >= 500) & (new_depth <= 5000)).astype(np.uint8) * 255
    # 保存画像上の説明文字・pixel markerは特徴抽出から除外する。
    old_mask[:45, :] = 0
    new_mask[:45, :] = 0
    for mask, source in ((old_mask, old_source), (new_mask, new_source)):
        u, v = source["pixel_uv"]
        cv2.circle(mask, (u, v), 35, 0, -1)

    orb = cv2.ORB_create(nfeatures=3000, scaleFactor=1.2, nlevels=8)
    old_keypoints, old_descriptors = orb.detectAndCompute(old_gray, old_mask)
    new_keypoints, new_descriptors = orb.detectAndCompute(new_gray, new_mask)
    result = {"old_features": len(old_keypoints), "new_features": len(new_keypoints)}
    if old_descriptors is None or new_descriptors is None:
        result["status"] = "descriptor不足"
        return result

    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    pairs = matcher.knnMatch(old_descriptors, new_descriptors, k=2)
    # 画像は屋外舗装面が多く特徴が少ないため、ratio testの後にPnP RANSACで幾何外れ対応を
    # 除く。inlier countと再投影誤差も必ず結果へ残す。
    matches = [first for first, second in pairs if first.distance < 0.78 * second.distance]
    object_points, image_points = [], []
    for match in matches:
        u0, v0 = np.rint(old_keypoints[match.queryIdx].pt).astype(int)
        u1, v1 = np.rint(new_keypoints[match.trainIdx].pt).astype(int)
        z0, z1 = float(old_depth[v0, u0]) * 0.001, float(new_depth[v1, u1]) * 0.001
        if not (0.5 <= z0 <= 5.0 and 0.5 <= z1 <= 5.0):
            continue
        object_points.append([(u0 - CX) * z0 / FX, (v0 - CY) * z0 / FY, z0])
        image_points.append([float(u1), float(v1)])
    result["ratio_matches"] = len(matches)
    result["depth_valid_matches"] = len(object_points)
    if len(object_points) < 8:
        result["status"] = "depth付き特徴対応が8点未満"
        return result

    object_points = np.asarray(object_points, dtype=np.float32)
    image_points = np.asarray(image_points, dtype=np.float32)
    success, rvec, tvec, inliers = cv2.solvePnPRansac(
        object_points, image_points, CAMERA_MATRIX, None,
        iterationsCount=3000, reprojectionError=5.0, confidence=0.999,
        flags=cv2.SOLVEPNP_EPNP,
    )
    if not success or inliers is None:
        result["status"] = "PnP robust fit失敗"
        return result
    indices = inliers[:, 0]
    rvec, tvec = cv2.solvePnPRefineLM(
        object_points[indices], image_points[indices], CAMERA_MATRIX, None,
        rvec, tvec,
    )
    visual_rotation, _ = cv2.Rodrigues(rvec)
    predicted, _ = cv2.projectPoints(
        object_points[indices], rvec, tvec, CAMERA_MATRIX, None
    )
    residual = np.linalg.norm(predicted[:, 0, :] - image_points[indices], axis=1)

    # URDFのbase_link -> RGB optical frame回転。並進は回転比較には影響しない。
    base_to_camera = rotation_from_rpy([-90.0, 0.0, -90.0])
    old_rpy = [old_source["base_pose"][key]
               for key in ("roll_deg", "pitch_deg", "yaw_deg")]
    new_rpy = [new_source["base_pose"][key]
               for key in ("roll_deg", "pitch_deg", "yaw_deg")]
    old_world_base = rotation_from_rpy(old_rpy)
    new_world_base = rotation_from_rpy(new_rpy)
    expected = base_to_camera.T @ new_world_base.T @ old_world_base @ base_to_camera
    inlier_fraction = float(len(indices) / len(object_points))
    median_error = float(np.median(residual))
    result.update({
        "inlier_count": int(len(indices)),
        "inlier_fraction_of_depth_matches": inlier_fraction,
        "median_reprojection_error_px": median_error,
        "p90_reprojection_error_px": float(np.percentile(residual, 90)),
        "warning": "PnPはfallback intrinsics、aligned RGB/depth仮定、ORB対応誤りの影響を受ける。独立真値ではない。",
    })
    if len(indices) < 8 or inlier_fraction < 0.5 or median_error > 5.0:
        result["status"] = "quality gate reject; pose comparison is inconclusive"
        return result
    result.update({
        "status": "quality gate passed",
        "visual_rotation_angle_axis_deg": angle_axis_degrees(visual_rotation).tolist(),
        "expected_odom_rotation_angle_axis_deg": angle_axis_degrees(expected).tolist(),
        "rotation_disagreement_deg": rotation_error_degrees(expected, visual_rotation),
        "visual_translation_camera2_m": tvec.reshape(3).tolist(),
    })
    return result


def project_source_between_frames(source, other_source, target_depth_image):
    """TFとsource depthから逆投影した地点を他frameへ投影し実depthと照合する。"""
    def camera_pose_world(item):
        pose = item["base_pose"]
        world_base_rotation = rotation_from_rpy(
            [pose["roll_deg"], pose["pitch_deg"], pose["yaw_deg"]]
        )
        base_camera_rotation = rotation_from_rpy([-90.0, 0.0, -90.0])
        rotation = world_base_rotation @ base_camera_rotation
        translation = np.array([pose["x_m"], pose["y_m"], pose["z_m"]])
        translation += world_base_rotation @ np.array([0.0, 0.0, 0.28])
        return rotation, translation

    u, v = source["pixel_uv"]
    depth_m = float(source["depth_m"])
    point_source = np.array([(u - CX) * depth_m / FX,
                             (v - CY) * depth_m / FY, depth_m])
    source_rotation, source_translation = camera_pose_world(source)
    other_rotation, other_translation = camera_pose_world(other_source)
    point_world = source_rotation @ point_source + source_translation
    point_other = other_rotation.T @ (point_world - other_translation)
    if point_other[2] <= 0.0:
        return {"status": "projected point behind camera"}
    projected_uv = [FX * point_other[0] / point_other[2] + CX,
                    FY * point_other[1] / point_other[2] + CY]
    px, py = np.rint(projected_uv).astype(int)
    if not (0 <= px < target_depth_image.shape[1]
            and 0 <= py < target_depth_image.shape[0]):
        return {"status": "projected point outside image",
                "projected_pixel_uv": projected_uv,
                "projected_depth_m": float(point_other[2]),
                "world_xyz_m": point_world.tolist()}
    patch = target_depth_image[max(0, py - 1):min(target_depth_image.shape[0], py + 2),
                               max(0, px - 1):min(target_depth_image.shape[1], px + 2)]
    valid = patch[(patch >= 400) & (patch <= 5000)].astype(np.float32) * 0.001
    exact = float(target_depth_image[py, px]) * 0.001
    return {
        "status": "in image",
        "projected_pixel_uv": [float(projected_uv[0]), float(projected_uv[1])],
        "projected_depth_m": float(point_other[2]),
        "projected_world_xyz_m": point_world.tolist(),
        "nearest_raw_depth_m": exact,
        "nearby_valid_depth_count_3x3": int(valid.size),
        "nearby_valid_depth_median_3x3": float(np.median(valid)) if valid.size else None,
        "nearby_valid_depth_min_3x3": float(valid.min()) if valid.size else None,
        "nearby_valid_depth_max_3x3": float(valid.max()) if valid.size else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("matches_json", type=Path)
    parser.add_argument("--case", type=int, default=1,
                        help="比較するcase番号（1始まり）")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    metadata = json.loads(args.matches_json.read_text(encoding="utf-8"))
    case = metadata["cases"][args.case - 1]
    folder = args.matches_json.parent
    old, new = case["sources"]["old"], case["sources"]["new"]
    result = {
        "case": args.case,
        "cell": case["cell"],
        "old_stamp_ns": old["stamp_ns"],
        "new_stamp_ns": new["stamp_ns"],
        "odom_yaw_delta_deg": new["base_pose"]["yaw_deg"] - old["base_pose"]["yaw_deg"],
        "rgbd_visual_relative_rotation": estimate_pair(
            folder / old["images"]["color"]["file"],
            folder / new["images"]["color"]["file"],
            folder / old["images"]["depth_raw_mm"]["file"],
            folder / new["images"]["depth_raw_mm"]["file"],
            old, new,
        ),
        "old_source_projected_into_new_depth": project_source_between_frames(
            old, new,
            cv2.imread(str(folder / new["images"]["depth_raw_mm"]["file"]),
                       cv2.IMREAD_UNCHANGED),
        ),
        "new_source_projected_into_old_depth": project_source_between_frames(
            new, old,
            cv2.imread(str(folder / old["images"]["depth_raw_mm"]["file"]),
                       cv2.IMREAD_UNCHANGED),
        ),
        "odom_pose_reprojection": odom_reprojection_errors(
            folder / old["images"]["color"]["file"],
            folder / new["images"]["color"]["file"],
            folder / old["images"]["depth_raw_mm"]["file"],
            folder / new["images"]["depth_raw_mm"]["file"],
            old, new,
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
