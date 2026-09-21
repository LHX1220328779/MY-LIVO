# wuhu/truck29/v006_camlidar

- stage: camlidar
- parent: wuhu/truck29/v005_camtime
- created: 2026-09-21T18:14:41+08:00
- note: camera-lidar edge refinement (long-edge filter)

## inputs

- bag: results/prepared/wuhu_livo_camera
- run: results/wuhu_livo_103_offset_precheck/Log/fusion/1789984266404494299
- frames: 12
- window: [5.0, 60.0]

## metrics

```
{
  "frames": 12,
  "edge_points": 44210,
  "score_before_px": 4.6339,
  "score_after_px": 4.5498,
  "delta_rot_deg_cam_xyz": [
    -0.1,
    0.1041,
    0.2001
  ],
  "delta_trans_m_cam_xyz": [
    0.0,
    0.0,
    -0.0
  ],
  "per_frame_before": [
    2.877,
    2.726,
    4.301,
    3.438,
    3.938,
    7.22,
    4.865,
    5.664,
    3.557,
    6.714,
    5.131,
    5.175
  ],
  "per_frame_after": [
    2.759,
    2.681,
    4.273,
    3.301,
    3.852,
    7.113,
    4.672,
    5.57,
    3.414,
    6.648,
    5.222,
    5.093
  ],
  "focal_hypothesis": {
    "fx": 1640.7,
    "fy": 1849.7,
    "score_px": 4.5467,
    "delta_rot_deg_cam_xyz": [
      -0.1014,
      0.1032,
      0.2001
    ],
    "delta_trans_m_cam_xyz": [
      0.0,
      0.0,
      -0.0
    ]
  },
  "focal_scan": [
    {
      "fx_scale": 0.92,
      "fx": 1511.2,
      "score_px": 4.9777,
      "rot_deg": [
        -0.2,
        -0.496,
        0.2
      ]
    },
    {
      "fx_scale": 0.96,
      "fx": 1576.9,
      "score_px": 4.7272,
      "rot_deg": [
        -0.2,
        -0.496,
        0.2
      ]
    },
    {
      "fx_scale": 1.0,
      "fx": 1642.6,
      "score_px": 4.5498,
      "rot_deg": [
        -0.1,
        0.104,
        0.2
      ]
    },
    {
      "fx_scale": 1.04,
      "fx": 1708.3,
      "score_px": 4.9496,
      "rot_deg": [
        -0.1,
        0.704,
        0.2
      ]
    },
    {
      "fx_scale": 1.08,
      "fx": 1774.0,
      "score_px": 5.7417,
      "rot_deg": [
        -0.1,
        0.704,
        0.2
      ]
    },
    {
      "fx_scale": 1.12,
      "fx": 1839.7,
      "score_px": 6.4948,
      "rot_deg": [
        -0.1,
        0.704,
        0.2
      ]
    },
    {
      "fx_scale": 1.16,
      "fx": 1905.4,
      "score_px": 7.118,
      "rot_deg": [
        -0.2,
        0.704,
        0.2
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
    2.12,
    2.48,
    2.61,
    4.35,
    5.4,
    6.19,
    6.69,
    6.79,
    6.77,
    6.54,
    6.44,
    6.42
  ],
  "overlay_dir": "/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v006_camlidar/overlays"
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
        2.8296,
        5.7648,
        1.8091
      ],
      "rpy_deg": [
        -0.804,
        -3.132,
        -1.22
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0042,
        -0.5528,
        0.5995
      ],
      "rpy_deg": [
        -0.988,
        0.76,
        -91.154
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0469,
        5.536,
        1.9515
      ],
      "rpy_deg": [
        -3.574,
        -0.665,
        178.807
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
      0.0742,
      7.7914,
      1.7519
    ],
    "rpy_deg": [
      -96.542,
      1.674,
      0.573
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
    "camera_source": "optical flow vs gyro on results/prepared/wuhu_livo_camera (corr 0.92)"
  }
}
```
