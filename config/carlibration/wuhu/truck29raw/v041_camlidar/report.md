# wuhu/truck29raw/v041_camlidar

- stage: camlidar (相机-LiDAR 偏航/横滚复核)
- parent: wuhu/truck29raw/v040_camtime
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v034_axle -> wuhu/truck29raw/v035_ground -> wuhu/truck29raw/v036_driven -> wuhu/truck29raw/v038_rtktime -> wuhu/truck29raw/v039_gcons -> wuhu/truck29raw/v040_camtime -> wuhu/truck29raw/v041_camlidar
- created: 2026-10-15T09:13:07+08:00
- note: pole yaw not applied (inconsistent observations): extrinsic kept

## inputs

- bag: /home/project/data/haibo/wuhu/29/20261013/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/29/20261013/ros2bag', 'kind': 'raw_split', 'segments': 58, 'duration_s': 869.98, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck29raw_livo_1015_090255/Log/fusion/1792026211558691207
- frames: 30
- window: [5.0, 115.0]

## metrics

```
{
  "method": "pole yaw (LiDAR pole axis vs image pole column)",
  "frames_checked": 30,
  "frames_with_poles": 5,
  "observations": [
    {
      "t": 17.5,
      "image": 830,
      "depth_m": 35.4,
      "height_m": 9.0,
      "rows": 47,
      "du_px": -4.87,
      "mad_px": 0.26,
      "yaw_deg": 0.1578,
      "bar_width_px": 7,
      "snr": 2.26,
      "speed_mps": 3.16,
      "roll_deg": null,
      "u_px": 517.6
    },
    {
      "t": 38.9,
      "image": 1472,
      "depth_m": 31.1,
      "height_m": 8.6,
      "rows": 87,
      "du_px": 2.65,
      "mad_px": 0.25,
      "yaw_deg": -0.0715,
      "bar_width_px": 7,
      "snr": 4.06,
      "speed_mps": 6.63,
      "roll_deg": null,
      "u_px": 87.2
    },
    {
      "t": 42.4,
      "image": 1577,
      "depth_m": 30.6,
      "height_m": 5.4,
      "rows": 72,
      "du_px": -19.19,
      "mad_px": 2.74,
      "yaw_deg": 0.6108,
      "bar_width_px": 3,
      "snr": 2.33,
      "speed_mps": 7.21,
      "roll_deg": -2.1273,
      "u_px": 1481.9
    }
  ],
  "rejected": [
    {
      "t": 10.3,
      "depth_m": 25.7,
      "bar_px": 27.8,
      "rows": 297,
      "rejected": "rows disagree (MAD 7.9 px over 39 rows, width 28 px)"
    },
    {
      "t": 17.5,
      "depth_m": 24.5,
      "bar_px": 27.1,
      "rows": 486,
      "rejected": "rows disagree (MAD 3.2 px over 463 rows, width 5 px)"
    },
    {
      "t": 21.0,
      "depth_m": 11.0,
      "bar_px": 60.3,
      "rows": 737,
      "rejected": "rows disagree (MAD 4.5 px over 703 rows, width 14 px)"
    }
  ],
  "image_scale": 1.0,
  "roll_delta_deg": 0.0,
  "roll_observations": 1,
  "roll_note": "fewer than 3 poles with enough row span; roll kept",
  "yaw_delta_deg": 0.1578,
  "yaw_mad_deg": 0.2293,
  "used": 3,
  "du_median_px": -4.87,
  "delta_rot_deg_cam_xyz": [
    0.0,
    0.1578,
    0.0
  ],
  "delta_trans_m_cam_xyz": [
    0.0,
    0.0,
    0.0
  ],
  "overlay_dir": "/home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v041_camlidar/overlays",
  "max_delta_deg": 0.5,
  "applied": false
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
        2.7755,
        5.9149,
        1.8312
      ],
      "rpy_deg": [
        -1.031,
        -3.057,
        -1.219
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0611,
        -0.2984,
        0.7111
      ],
      "rpy_deg": [
        -1.047,
        0.481,
        -91.365
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0915,
        5.6621,
        2.0104
      ],
      "rpy_deg": [
        3.243,
        -4.537,
        178.24
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.106,
        7.9492,
        0.051
      ],
      "rpy_deg": [
        1.485,
        -1.56,
        89.984
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0067,
      7.9506,
      1.7516
    ],
    "rpy_deg": [
      -96.477,
      1.668,
      0.485
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
    "lidar": -0.0361,
    "camera": 0.0,
    "rtk_position": 0.0151,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "lidar_frontend": -0.0311,
    "lidar_frontend_source": "axle: lidar -0.0361 + imu/ins clock lag +0.0050 (corr 0.998); template -0.1250 replaced",
    "rtk_position_source": "rtk-time: map consistency, secant of both rounds (measured gain 0.84); effective +10.1 ms (se 2.0 ms)"
  }
}
```
