# wuhu/truck29/v017_camlidar

- stage: camlidar (相机-LiDAR 外参精修)
- parent: wuhu/truck29/v016_camtime
- lineage: wuhu/truck29/v007_import -> wuhu/truck29/v012_ground -> wuhu/truck29/v013_driven -> wuhu/truck29/v014_camtime -> wuhu/truck29/v016_camtime -> wuhu/truck29/v017_camlidar
- created: 2026-09-22T18:23:20+08:00
- note: pipeline: camera-lidar

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck29_livo_0922_181958/Log/fusion/1790072415472741896
- frames: 12
- window: [5.0, 60.0]

## changes vs parent

```
{
  "camera_to_rear": {
    "dt_cm": [
      0.01,
      0.01,
      -0.01
    ],
    "drpy_deg": [
      -0.201,
      0.091,
      -0.107
    ]
  }
}
```

## metrics

```
{
  "frames": 12,
  "edge_points": 43204,
  "score_before_px": 4.5523,
  "score_after_px": 4.4398,
  "delta_rot_deg_cam_xyz": [
    -0.1979,
    0.0958,
    0.1021
  ],
  "delta_trans_m_cam_xyz": [
    0.0001,
    0.0001,
    0.0
  ],
  "per_frame_before": [
    2.509,
    2.899,
    3.806,
    3.413,
    3.936,
    7.224,
    4.806,
    6.231,
    3.371,
    6.716,
    5.342,
    4.377
  ],
  "per_frame_after": [
    2.144,
    2.822,
    3.506,
    3.394,
    3.943,
    7.07,
    4.711,
    6.253,
    3.156,
    6.642,
    5.23,
    4.405
  ],
  "focal_hypothesis": {
    "fx": 1642.1,
    "fy": 1852.3,
    "score_px": 4.4392,
    "delta_rot_deg_cam_xyz": [
      -0.1997,
      0.0915,
      0.1039
    ],
    "delta_trans_m_cam_xyz": [
      0.0001,
      0.0001,
      0.0
    ]
  },
  "focal_scan": [
    {
      "fx_scale": 0.92,
      "fx": 1511.2,
      "score_px": 4.8404,
      "rot_deg": [
        -0.298,
        -0.504,
        0.102
      ]
    },
    {
      "fx_scale": 0.96,
      "fx": 1576.9,
      "score_px": 4.6233,
      "rot_deg": [
        -0.198,
        -0.504,
        0.102
      ]
    },
    {
      "fx_scale": 1.0,
      "fx": 1642.6,
      "score_px": 4.4398,
      "rot_deg": [
        -0.198,
        0.096,
        0.102
      ]
    },
    {
      "fx_scale": 1.04,
      "fx": 1708.3,
      "score_px": 4.8126,
      "rot_deg": [
        -0.198,
        0.696,
        0.102
      ]
    },
    {
      "fx_scale": 1.08,
      "fx": 1774.0,
      "score_px": 5.694,
      "rot_deg": [
        -0.098,
        0.696,
        0.102
      ]
    },
    {
      "fx_scale": 1.12,
      "fx": 1839.7,
      "score_px": 6.5731,
      "rot_deg": [
        -0.198,
        0.696,
        0.102
      ]
    },
    {
      "fx_scale": 1.16,
      "fx": 1905.4,
      "score_px": 7.0989,
      "rot_deg": [
        -0.398,
        0.696,
        0.102
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
  "frame_speeds_mps": [
    2.13,
    2.49,
    2.72,
    4.36,
    5.32,
    6.28,
    6.65,
    6.78,
    6.82,
    6.59,
    6.39,
    6.53
  ],
  "overlay_dir": "/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v017_camlidar/overlays"
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
        2.835,
        5.769,
        1.8093
      ],
      "rpy_deg": [
        -0.802,
        -3.13,
        -1.216
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        0.0076,
        -0.5292,
        0.6086
      ],
      "rpy_deg": [
        -0.979,
        0.794,
        -91.174
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.033,
        5.5449,
        1.9503
      ],
      "rpy_deg": [
        -3.582,
        -0.661,
        178.827
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.033,
        7.793,
        0.051
      ],
      "rpy_deg": [
        1.3,
        -1.67,
        90.16
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0743,
      7.7915,
      1.7518
    ],
    "rpy_deg": [
      -96.57,
      1.573,
      0.552
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
    "lidar": -0.125,
    "camera": -0.115,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "camera_source": "optical flow vs gyro on /home/project/data/haibo/wuhu_livo/ros2bag (corr 0.92)"
  }
}
```
