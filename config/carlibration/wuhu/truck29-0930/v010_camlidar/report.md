# wuhu/truck29-0930/v010_camlidar

- stage: camlidar (相机-LiDAR 偏航/横滚复核)
- parent: wuhu/truck29-0930/v009_camtime
- lineage: wuhu/truck29-0930/v002_import -> wuhu/truck29-0930/v003_axle -> wuhu/truck29-0930/v004_ground -> wuhu/truck29-0930/v005_driven -> wuhu/truck29-0930/v007_rtktime -> wuhu/truck29-0930/v008_gcons -> wuhu/truck29-0930/v009_camtime -> wuhu/truck29-0930/v010_camlidar
- created: 2026-10-19T17:11:58+08:00
- note: pole yaw not applied (only 1 usable pole observations): extrinsic kept; camera roll +0.288 deg from the sky line (groups [0.3, 0.3, 0.3])

## inputs

- bag: /home/project/data/haibo/wuhu/29/20261013/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/29/20261013/ros2bag', 'kind': 'raw_split', 'segments': 58, 'duration_s': 869.98, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck29-0930_livo_1019_170854/Log/fusion/1792400970084019260
- frames: 30
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
      -0.001,
      0.286,
      -0.032
    ]
  }
}
```

## metrics

```
{
  "method": "pole yaw (LiDAR pole axis vs image pole column)",
  "frames_checked": 30,
  "frames_with_poles": 2,
  "observations": [
    {
      "t": 28.1,
      "image": 758,
      "depth_m": 15.1,
      "height_m": 6.7,
      "rows": 583,
      "du_px": -3.77,
      "mad_px": 2.88,
      "yaw_deg": 0.1295,
      "bar_width_px": 10,
      "snr": 2.73,
      "speed_mps": 4.18,
      "roll_deg": -1.3489,
      "u_px": 1178.4
    }
  ],
  "rejected": [
    {
      "t": 6.8,
      "depth_m": 35.5,
      "bar_px": 16.4,
      "rows": 256,
      "rejected": "rows disagree (MAD 6.6 px over 19 rows, width 16 px)"
    }
  ],
  "image_scale": 1.0,
  "error": "only 1 usable pole observations",
  "max_delta_deg": 0.5,
  "applied": false,
  "roll_applied": true,
  "roll": {
    "roll_deg": 0.288,
    "frames": 76,
    "per_frame_mad_deg": 0.2,
    "group_roll_deg": [
      0.3,
      0.3,
      0.3
    ],
    "group_spread_deg": 0.0,
    "cost_px_at_zero": 10.2544,
    "cost_px_at_min": 10.1638,
    "curvature_px": 0.0594,
    "profile": [
      [
        -1.6,
        11.9726
      ],
      [
        -1.4,
        11.7791
      ],
      [
        -1.2,
        11.5295
      ],
      [
        -1.0,
        11.2811
      ],
      [
        -0.8,
        11.0384
      ],
      [
        -0.6,
        10.8092
      ],
      [
        -0.4,
        10.5469
      ],
      [
        -0.2,
        10.3872
      ],
      [
        0.0,
        10.2544
      ],
      [
        0.2,
        10.1808
      ],
      [
        0.4,
        10.1915
      ],
      [
        0.6,
        10.2747
      ],
      [
        0.8,
        10.4103
      ],
      [
        1.0,
        10.5573
      ],
      [
        1.2,
        10.7158
      ],
      [
        1.4,
        10.9478
      ],
      [
        1.6,
        11.1497
      ]
    ],
    "method": "sky line: image vs LiDAR top envelope (tools/calib/camera_roll.py)",
    "frames_checked": 150,
    "accumulate_s": 1.5,
    "window_s": [
      5.0,
      56.59999990463257
    ],
    "overlay_dir": "/home/project/MY-LIVO/config/carlibration/wuhu/truck29-0930/v010_camlidar/overlays",
    "applied": true,
    "reasons": []
  }
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
        2.7568,
        5.9059,
        1.8332
      ],
      "rpy_deg": [
        -1.026,
        -3.031,
        -1.254
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0977,
        -0.3133,
        0.7135
      ],
      "rpy_deg": [
        -1.06,
        0.482,
        -91.378
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1232,
        5.651,
        2.0183
      ],
      "rpy_deg": [
        3.257,
        -4.535,
        178.187
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1296,
        7.9462,
        0.051
      ],
      "rpy_deg": [
        1.477,
        -1.57,
        89.955
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0171,
      7.9472,
      1.7516
    ],
    "rpy_deg": [
      -96.468,
      1.947,
      0.424
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
    "source": "midrange_camera[0].camera_in_params"
  },
  "time_offsets": {
    "lidar": -0.0372,
    "lidar_frontend": -0.0322,
    "camera": 0.0,
    "rtk_position": 0.0209,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0372 + imu/ins clock lag +0.0050 (corr 0.998); template -0.1250 replaced",
    "rtk_position_source": "rtk-time: map consistency, secant of both rounds (measured gain 0.74); effective +15.9 ms (se 0.9 ms)"
  }
}
```
