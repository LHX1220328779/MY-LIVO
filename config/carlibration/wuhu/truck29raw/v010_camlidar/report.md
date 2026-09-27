# wuhu/truck29raw/v010_camlidar

- stage: camlidar (相机-LiDAR 偏航/横滚复核)
- parent: wuhu/truck29raw/v009_camloop
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v002_axle -> wuhu/truck29raw/v003_ground -> wuhu/truck29raw/v004_driven -> wuhu/truck29raw/v005_camtime -> wuhu/truck29raw/v006_camlidar -> wuhu/truck29raw/v009_camloop -> wuhu/truck29raw/v010_camlidar
- created: 2026-09-26T18:07:47+08:00
- note: mcp-yawroll-audit

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- run: results/mcp_t29raw_p02_livo120_r2/Log/fusion/1790413711524616733/
- frames: 16
- window: [5.0, 115.0]

## changes vs parent

```
{
  "camera_to_rear": {
    "dt_cm": [
      0.0,
      0.0,
      0.0
    ],
    "drpy_deg": [
      0.002,
      0.112,
      0.093
    ]
  }
}
```

## metrics

```
{
  "frames": 16,
  "edge_points": 20069,
  "score_before_px": 9.4659,
  "score_after_px": 9.4625,
  "delta_rot_deg_cam_xyz": [
    0.0,
    -0.1053,
    0.1005
  ],
  "delta_trans_m_cam_xyz": [
    0.0,
    0.0,
    0.0
  ],
  "per_frame_before": [
    9.105,
    8.9,
    9.063,
    9.083,
    9.199,
    9.348,
    8.819,
    8.782,
    9.249,
    9.905,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0
  ],
  "per_frame_after": [
    9.174,
    8.85,
    9.0,
    9.078,
    9.21,
    9.345,
    8.765,
    8.739,
    9.275,
    9.867,
    10.095,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0
  ],
  "focal_hypothesis": {
    "fx": 1638.7,
    "fy": 1664.9,
    "score_px": 9.4491,
    "delta_rot_deg_cam_xyz": [
      0.0001,
      -0.1055,
      0.1006
    ],
    "delta_trans_m_cam_xyz": [
      0.0001,
      0.0002,
      0.0002
    ]
  },
  "focal_scan": [
    {
      "fx_scale": 0.92,
      "fx": 1511.2,
      "score_px": 9.7058,
      "rot_deg": [
        0.0,
        -0.705,
        0.101
      ]
    },
    {
      "fx_scale": 0.96,
      "fx": 1576.9,
      "score_px": 9.5736,
      "rot_deg": [
        0.0,
        -0.605,
        0.101
      ]
    },
    {
      "fx_scale": 1.0,
      "fx": 1642.6,
      "score_px": 9.4625,
      "rot_deg": [
        0.0,
        -0.105,
        0.101
      ]
    },
    {
      "fx_scale": 1.04,
      "fx": 1708.3,
      "score_px": 9.9622,
      "rot_deg": [
        0.0,
        0.395,
        0.101
      ]
    },
    {
      "fx_scale": 1.08,
      "fx": 1774.0,
      "score_px": 10.5638,
      "rot_deg": [
        0.0,
        0.495,
        0.101
      ]
    },
    {
      "fx_scale": 1.12,
      "fx": 1839.7,
      "score_px": 11.1263,
      "rot_deg": [
        0.0,
        0.495,
        0.101
      ]
    },
    {
      "fx_scale": 1.16,
      "fx": 1905.4,
      "score_px": 11.7573,
      "rot_deg": [
        0.0,
        0.495,
        0.101
      ]
    }
  ],
  "intrinsics_used": [
    1642.57,
    1855.53,
    971.01,
    534.26
  ],
  "intrinsics_solved": false,
  "image_scale": 0.5,
  "dof": "yaw+roll (vertical edges, pitch fixed)",
  "frame_speeds_mps": [
    2.13,
    2.55,
    3.97,
    5.88,
    6.72,
    6.88,
    6.69,
    6.47,
    6.36,
    5.36,
    3.68,
    3.43,
    4.25,
    4.83,
    5.38,
    5.46
  ],
  "overlay_dir": "/home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v010_camlidar/overlays",
  "applied": true,
  "max_delta_deg": 0.5
}
```

## summary

```
{
  "lidars": [
    {
      "lidar": "front_right_lidar",
      "key": "transform_matrix_lidar_1",
      "t": [
        2.756,
        5.9895,
        1.8333
      ],
      "rpy_deg": [
        -1.001,
        -3.026,
        -1.433
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1094,
        -0.311,
        0.6862
      ],
      "rpy_deg": [
        -1.079,
        0.746,
        -91.392
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1269,
        5.7908,
        1.9755
      ],
      "rpy_deg": [
        -3.418,
        -0.801,
        178.584
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.108,
        8.0271,
        0.051
      ],
      "rpy_deg": [
        1.452,
        -1.533,
        89.921
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0037,
      8.0291,
      1.7517
    ],
    "rpy_deg": [
      -96.495,
      1.729,
      0.351
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1642.570556640625,
      1855.53173828125,
      971.0083025529602,
      534.2602337509306
    ],
    "distortion": [
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    ],
    "source": "defaults (wuhu truck29 yaml)"
  },
  "time_offsets": {
    "lidar": -0.1463,
    "camera": -0.1211,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "camera_source": "optical flow vs gyro on /home/project/data/haibo/wuhu_livo/ros2bag (corr 0.92)"
  }
}
```
