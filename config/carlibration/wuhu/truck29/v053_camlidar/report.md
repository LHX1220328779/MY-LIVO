# wuhu/truck29/v053_camlidar

- stage: camlidar (相机-LiDAR 偏航/横滚复核)
- parent: wuhu/truck29/v052_camtime
- lineage: wuhu/truck29/v037_import -> wuhu/truck29/v049_axle -> wuhu/truck29/v050_ground -> wuhu/truck29/v051_driven -> wuhu/truck29/v052_camtime -> wuhu/truck29/v053_camlidar
- created: 2026-09-24T13:00:26+08:00
- note: mcp turn4: camera pipeline 60s

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck29_livo_0924_125710/Log/fusion/1790225840555744472
- frames: 30
- window: [5.0, 115.0]

## metrics

```
{
  "method": "pole yaw (LiDAR pole axis vs image pole column)",
  "frames_checked": 30,
  "frames_with_poles": 15,
  "observations": [
    {
      "t": 5.0,
      "image": 210,
      "depth_m": 33.7,
      "height_m": 6.0,
      "rows": 260,
      "du_px": -5.66,
      "mad_px": 0.37,
      "yaw_deg": 0.1971,
      "bar_width_px": 5,
      "snr": 2.38,
      "speed_mps": 2.08
    },
    {
      "t": 6.8,
      "image": 263,
      "depth_m": 29.3,
      "height_m": 7.2,
      "rows": 315,
      "du_px": -0.75,
      "mad_px": 0.48,
      "yaw_deg": 0.026,
      "bar_width_px": 7,
      "snr": 2.43,
      "speed_mps": 2.71
    },
    {
      "t": 8.6,
      "image": 317,
      "depth_m": 24.7,
      "height_m": 7.7,
      "rows": 440,
      "du_px": 3.3,
      "mad_px": 0.67,
      "yaw_deg": -0.1116,
      "bar_width_px": 7,
      "snr": 2.5,
      "speed_mps": 2.27
    },
    {
      "t": 10.3,
      "image": 370,
      "depth_m": 30.9,
      "height_m": 7.5,
      "rows": 348,
      "du_px": 5.0,
      "mad_px": 0.97,
      "yaw_deg": -0.1518,
      "bar_width_px": 7,
      "snr": 2.59,
      "speed_mps": 2.49
    },
    {
      "t": 12.1,
      "image": 424,
      "depth_m": 28.8,
      "height_m": 8.2,
      "rows": 372,
      "du_px": 5.96,
      "mad_px": 1.13,
      "yaw_deg": -0.2077,
      "bar_width_px": 5,
      "snr": 2.37,
      "speed_mps": 2.56
    },
    {
      "t": 13.9,
      "image": 477,
      "depth_m": 23.2,
      "height_m": 8.2,
      "rows": 467,
      "du_px": 9.21,
      "mad_px": 0.89,
      "yaw_deg": -0.2647,
      "bar_width_px": 7,
      "snr": 2.53,
      "speed_mps": 2.41
    },
    {
      "t": 49.6,
      "image": 1547,
      "depth_m": 27.7,
      "height_m": 8.7,
      "rows": 389,
      "du_px": -2.0,
      "mad_px": 1.96,
      "yaw_deg": 0.066,
      "bar_width_px": 3,
      "snr": 2.14,
      "speed_mps": 6.73
    },
    {
      "t": 51.4,
      "image": 1601,
      "depth_m": 16.1,
      "height_m": 8.7,
      "rows": 379,
      "du_px": -5.98,
      "mad_px": 0.79,
      "yaw_deg": 0.1758,
      "bar_width_px": 14,
      "snr": 2.46,
      "speed_mps": 6.62
    },
    {
      "t": 56.7,
      "image": 1761,
      "depth_m": 31.0,
      "height_m": 5.4,
      "rows": 210,
      "du_px": -5.49,
      "mad_px": 0.03,
      "yaw_deg": 0.1798,
      "bar_width_px": 5,
      "snr": 2.64,
      "speed_mps": 6.23
    }
  ],
  "rejected": [
    {
      "t": 26.4,
      "depth_m": 29.8,
      "bar_px": 17.2,
      "rows": 431,
      "rejected": "rows disagree (MAD 5.7 px over 397 rows, width 5 px)"
    },
    {
      "t": 28.2,
      "depth_m": 19.9,
      "bar_px": 29.7,
      "rows": 533,
      "rejected": "rows disagree (MAD 7.4 px over 457 rows, width 19 px)"
    },
    {
      "t": 33.5,
      "depth_m": 33.9,
      "bar_px": 11.8,
      "rows": 448,
      "rejected": "rows disagree (MAD 3.4 px over 423 rows, width 5 px)"
    },
    {
      "t": 35.3,
      "depth_m": 22.2,
      "bar_px": 21.2,
      "rows": 483,
      "rejected": "rows disagree (MAD 5.0 px over 460 rows, width 14 px)"
    },
    {
      "t": 42.4,
      "depth_m": 31.2,
      "bar_px": 21.0,
      "rows": 395,
      "rejected": "rows disagree (MAD 3.9 px over 363 rows, width 5 px)"
    },
    {
      "t": 44.2,
      "depth_m": 19.1,
      "bar_px": 34.2,
      "rows": 479,
      "rejected": "rows disagree (MAD 4.5 px over 404 rows, width 14 px)"
    }
  ],
  "image_scale": 1.0,
  "yaw_delta_deg": 0.026,
  "yaw_mad_deg": 0.1538,
  "used": 9,
  "du_median_px": -0.75,
  "delta_rot_deg_cam_xyz": [
    0.0,
    0.026,
    0.0
  ],
  "delta_trans_m_cam_xyz": [
    0.0,
    0.0,
    0.0
  ],
  "overlay_dir": "/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v053_camlidar/overlays",
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
        2.7472,
        5.9783,
        1.8375
      ],
      "rpy_deg": [
        -1.014,
        -3.031,
        -1.448
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1211,
        -0.325,
        0.6932
      ],
      "rpy_deg": [
        -1.055,
        0.746,
        -91.374
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1345,
        5.7832,
        1.9759
      ],
      "rpy_deg": [
        -3.416,
        -0.798,
        178.576
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1141,
        8.0179,
        0.051
      ],
      "rpy_deg": [
        1.452,
        -1.531,
        89.91
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0024,
      8.02,
      1.7517
    ],
    "rpy_deg": [
      -96.507,
      1.636,
      0.41
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
    "lidar": -0.1539,
    "camera": -0.1211,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "camera_source": "optical flow vs gyro on /home/project/data/haibo/wuhu_livo/ros2bag (corr 0.92)"
  }
}
```
