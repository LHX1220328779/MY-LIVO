# wuhu/truck29/v015_camlidar

- stage: camlidar (相机-LiDAR 外参精修)
- parent: wuhu/truck29/v014_camtime
- lineage: wuhu/truck29/v007_import -> wuhu/truck29/v012_ground -> wuhu/truck29/v013_driven -> wuhu/truck29/v014_camtime -> wuhu/truck29/v015_camlidar
- created: 2026-09-22T15:23:43+08:00
- note: pipeline: camera-lidar

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck29_livo_0922_152050/Log/fusion/1790061666549621594
- frames: 12
- window: [5.0, 60.0]

## changes vs parent

```
{
  "camera_to_rear": {
    "dt_cm": [
      0.03,
      0.02,
      0.01
    ],
    "drpy_deg": [
      -0.193,
      0.316,
      -0.129
    ]
  }
}
```

## metrics

```
{
  "frames": 12,
  "edge_points": 43378,
  "score_before_px": 4.6614,
  "score_after_px": 4.5451,
  "delta_rot_deg_cam_xyz": [
    -0.1896,
    0.0926,
    0.3283
  ],
  "delta_trans_m_cam_xyz": [
    0.0003,
    -0.0,
    0.0001
  ],
  "per_frame_before": [
    2.999,
    2.784,
    4.127,
    3.533,
    3.987,
    7.256,
    4.951,
    5.621,
    3.461,
    6.837,
    5.129,
    5.252
  ],
  "per_frame_after": [
    2.796,
    2.744,
    4.078,
    3.431,
    3.86,
    7.134,
    4.679,
    5.544,
    3.249,
    6.696,
    5.245,
    5.086
  ],
  "focal_hypothesis": {
    "fx": 1639.8,
    "fy": 1844.5,
    "score_px": 4.5376,
    "delta_rot_deg_cam_xyz": [
      -0.1917,
      0.0906,
      0.3156
    ],
    "delta_trans_m_cam_xyz": [
      0.0003,
      -0.0,
      0.0001
    ]
  },
  "focal_scan": [
    {
      "fx_scale": 0.92,
      "fx": 1511.2,
      "score_px": 4.9745,
      "rot_deg": [
        -0.19,
        -0.507,
        0.328
      ]
    },
    {
      "fx_scale": 0.96,
      "fx": 1576.9,
      "score_px": 4.7215,
      "rot_deg": [
        -0.19,
        -0.507,
        0.328
      ]
    },
    {
      "fx_scale": 1.0,
      "fx": 1642.6,
      "score_px": 4.5451,
      "rot_deg": [
        -0.19,
        0.093,
        0.328
      ]
    },
    {
      "fx_scale": 1.04,
      "fx": 1708.3,
      "score_px": 4.9463,
      "rot_deg": [
        -0.19,
        0.693,
        0.328
      ]
    },
    {
      "fx_scale": 1.08,
      "fx": 1774.0,
      "score_px": 5.7069,
      "rot_deg": [
        -0.09,
        0.693,
        0.328
      ]
    },
    {
      "fx_scale": 1.12,
      "fx": 1839.7,
      "score_px": 6.4697,
      "rot_deg": [
        -0.09,
        0.693,
        0.328
      ]
    },
    {
      "fx_scale": 1.16,
      "fx": 1905.4,
      "score_px": 7.0811,
      "rot_deg": [
        -0.29,
        0.693,
        0.328
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
    2.09,
    2.44,
    2.61,
    4.49,
    5.37,
    6.16,
    6.69,
    6.59,
    6.65,
    6.52,
    6.38,
    6.38
  ],
  "overlay_dir": "/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v015_camlidar/overlays"
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
      0.0745,
      7.7916,
      1.752
    ],
    "rpy_deg": [
      -96.562,
      1.798,
      0.53
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
