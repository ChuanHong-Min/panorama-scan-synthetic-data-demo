# 全景拼图核心流程伪代码

## 点云配准

```text
输入: 标定点云帧序列 frames
输出: 每帧到全局坐标系的初始变换 transforms

初始化 global_map
初始化 transforms

for frame in frames:
    points = detect_calibration_points(frame)
    descriptors = build_local_descriptors(points)
    matches = match_points(descriptors, global_map.descriptors)
    rigid_transform = estimate_rigid_transform(matches)
    transforms.append(rigid_transform)
    global_map.update(points, rigid_transform)

return smooth(transforms)
```

## 位姿估计

```text
输入: 帧图像序列 frames, 初始变换 transforms
输出: 位姿标签 poses

for frame, transform in zip(frames, transforms):
    gradient = compute_edge_response(frame)
    local_offset = refine_alignment(gradient, transform)
    rotation = estimate_rotation(local_offset)
    scale = estimate_scale(local_offset)
    illumination = estimate_lighting(frame)
    pose = compose_pose(transform, rotation, scale, illumination)
    poses.append(pose)

return reject_unstable_pose_jitter(poses)
```

## 逆向重建

```text
输入: 页面图像 page, 位姿标签 poses, 目标帧尺寸 frame_size
输出: 合成帧 synthetic_frames

for pose in poses:
    homography = build_inverse_homography(pose, frame_size)
    patch = warp_page_to_camera_view(page, homography)
    patch = apply_rotation_scale_and_crop(patch, pose)
    patch = apply_lighting_model(patch, pose.illumination)
    patch = add_sensor_like_degradation(patch)
    synthetic_frames.append(patch)

return synthetic_frames
```

## 质量筛选

```text
输入: 位姿标签 poses, 合成帧 synthetic_frames
输出: 可用于训练的数据样本 accepted_samples

for trajectory in group_by_scan(synthetic_frames, poses):
    continuity = measure_pose_continuity(trajectory.poses)
    overlap = measure_frame_overlap(trajectory.poses)
    blur_score = estimate_blur(trajectory.frames)
    invalid_ratio = count_invalid_frames(trajectory) / count_all_frames(trajectory)

    if continuity is stable
       and overlap is sufficient
       and blur_score is within_limit
       and invalid_ratio is below_threshold:
        accepted_samples.append(trajectory)

return accepted_samples
```

## 难例生成

```text
输入: 基础页面集合 pages, 失败模式集合 failure_modes
输出: 定向增强样本 hard_samples

for page in pages:
    regions = locate_structured_regions(page)
    for mode in failure_modes:
        trajectory = sample_pose_path(mode)
        frames = reconstruct_frames(page, regions, trajectory)
        labels = build_pose_labels(trajectory)
        if passes_quality_filter(frames, labels):
            hard_samples.append((frames, labels, mode.category))

return balance_by_failure_category(hard_samples)
```
