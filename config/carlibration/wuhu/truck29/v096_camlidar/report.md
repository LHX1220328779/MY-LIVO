# wuhu/truck29/v096_camlidar

- stage: camlidar (相机-LiDAR 偏航/横滚复核)
- parent: wuhu/truck29/v095_camtime
- lineage: wuhu/truck29/v001_import -> wuhu/truck29/v090_axle -> wuhu/truck29/v091_ground -> wuhu/truck29/v092_driven -> wuhu/truck29/v094_rtktime -> wuhu/truck29/v095_camtime -> wuhu/truck29/v096_camlidar
- created: 2026-10-12T10:24:41+08:00
- note: mcp: camera pipeline on axle-fixed v094

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck29_livo_1012_102119/Log/fusion/1791771689291111344
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
      -0.005,
      -0.021,
      -0.182
    ]
  }
}
```

## metrics

```
{
  "method": "pole yaw (LiDAR pole axis vs image pole column)",
  "frames_checked": 30,
  "frames_with_poles": 14,
  "observations": [
    {
      "t": 5.0,
      "image": 214,
      "depth_m": 33.5,
      "height_m": 6.1,
      "rows": 260,
      "du_px": -5.41,
      "mad_px": 0.46,
      "yaw_deg": 0.1882,
      "bar_width_px": 5,
      "snr": 2.4,
      "speed_mps": 2.2,
      "roll_deg": 0.3192,
      "u_px": 1053.7
    },
    {
      "t": 8.6,
      "image": 322,
      "depth_m": 24.4,
      "height_m": 7.7,
      "rows": 445,
      "du_px": -5.4,
      "mad_px": 1.01,
      "yaw_deg": 0.1815,
      "bar_width_px": 7,
      "snr": 2.47,
      "speed_mps": 2.34,
      "roll_deg": -0.5561,
      "u_px": 656.7
    },
    {
      "t": 10.3,
      "image": 373,
      "depth_m": 30.9,
      "height_m": 8.1,
      "rows": 358,
      "du_px": -6.33,
      "mad_px": 0.92,
      "yaw_deg": 0.1935,
      "bar_width_px": 5,
      "snr": 2.75,
      "speed_mps": 2.5,
      "roll_deg": 0.8154,
      "u_px": 1588.6
    },
    {
      "t": 12.1,
      "image": 427,
      "depth_m": 28.6,
      "height_m": 8.2,
      "rows": 387,
      "du_px": -5.3,
      "mad_px": 0.82,
      "yaw_deg": 0.1848,
      "bar_width_px": 5,
      "snr": 2.43,
      "speed_mps": 2.55,
      "roll_deg": -0.5027,
      "u_px": 932.5
    },
    {
      "t": 13.9,
      "image": 481,
      "depth_m": 22.9,
      "height_m": 8.1,
      "rows": 454,
      "du_px": -4.57,
      "mad_px": 1.01,
      "yaw_deg": 0.1294,
      "bar_width_px": 7,
      "snr": 2.55,
      "speed_mps": 2.51,
      "roll_deg": 0.5801,
      "u_px": 187.0
    },
    {
      "t": 49.4,
      "image": 1546,
      "depth_m": 28.1,
      "height_m": 8.7,
      "rows": 380,
      "du_px": -1.16,
      "mad_px": 1.96,
      "yaw_deg": 0.0381,
      "bar_width_px": 3,
      "snr": 2.15,
      "speed_mps": 6.65,
      "roll_deg": 1.6424,
      "u_px": 1370.1
    },
    {
      "t": 51.2,
      "image": 1600,
      "depth_m": 16.5,
      "height_m": 8.8,
      "rows": 397,
      "du_px": -1.1,
      "mad_px": 0.86,
      "yaw_deg": 0.0325,
      "bar_width_px": 14,
      "snr": 2.44,
      "speed_mps": 6.51,
      "roll_deg": 0.7652,
      "u_px": 1663.5
    },
    {
      "t": 56.5,
      "image": 1759,
      "depth_m": 31.5,
      "height_m": 7.9,
      "rows": 358,
      "du_px": -3.8,
      "mad_px": 0.98,
      "yaw_deg": 0.1248,
      "bar_width_px": 5,
      "snr": 2.96,
      "speed_mps": 6.33,
      "roll_deg": -0.9649,
      "u_px": 1385.8
    }
  ],
  "rejected": [
    {
      "t": 26.3,
      "depth_m": 29.8,
      "bar_px": 16.3,
      "rows": 427,
      "rejected": "rows disagree (MAD 5.7 px over 406 rows, width 7 px)"
    },
    {
      "t": 28.1,
      "depth_m": 19.7,
      "bar_px": 30.3,
      "rows": 527,
      "rejected": "rows disagree (MAD 7.9 px over 463 rows, width 7 px)"
    },
    {
      "t": 33.4,
      "depth_m": 34.1,
      "bar_px": 10.4,
      "rows": 446,
      "rejected": "rows disagree (MAD 3.4 px over 426 rows, width 5 px)"
    },
    {
      "t": 35.2,
      "depth_m": 22.2,
      "bar_px": 20.7,
      "rows": 482,
      "rejected": "rows disagree (MAD 5.0 px over 462 rows, width 14 px)"
    },
    {
      "t": 42.3,
      "depth_m": 31.4,
      "bar_px": 21.5,
      "rows": 400,
      "rejected": "rows disagree (MAD 3.9 px over 367 rows, width 5 px)"
    },
    {
      "t": 44.1,
      "depth_m": 19.3,
      "bar_px": 32.5,
      "rows": 472,
      "rejected": "rows disagree (MAD 4.6 px over 406 rows, width 14 px)"
    }
  ],
  "image_scale": 1.0,
  "roll_delta_deg": 0.0,
  "roll_mad_deg": 0.6591,
  "roll_used": 8,
  "roll_observations": 8,
  "yaw_delta_deg": 0.1831,
  "yaw_mad_deg": 0.0317,
  "used": 6,
  "du_median_px": -4.94,
  "delta_rot_deg_cam_xyz": [
    0.0,
    0.1831,
    0.0
  ],
  "delta_trans_m_cam_xyz": [
    0.0,
    0.0,
    0.0
  ],
  "overlay_dir": "/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v096_camlidar/overlays",
  "max_delta_deg": 0.5,
  "applied": true
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
        2.7482,
        6.0106,
        1.8345
      ],
      "rpy_deg": [
        -1.007,
        -3.028,
        -1.45
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1311,
        -0.1709,
        0.6466
      ],
      "rpy_deg": [
        -1.05,
        0.666,
        -91.31
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1091,
        5.8074,
        1.9688
      ],
      "rpy_deg": [
        -3.444,
        -0.775,
        178.578
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1089,
        8.0403,
        0.051
      ],
      "rpy_deg": [
        1.452,
        -1.532,
        89.924
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0028,
      8.0424,
      1.7516
    ],
    "rpy_deg": [
      -96.58,
      1.619,
      0.281
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
    "lidar": -0.146,
    "camera": -0.1211,
    "rtk_position": -0.02,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "lidar_frontend": -0.125,
    "lidar_frontend_source": "axle: import default (parent version had no lidar_frontend)",
    "rtk_position_source": "rtk-time: map consistency, first + probe rounds with the prior gain 0.6, standard-error weighted (secant gain 0.73 not usable); effective -41.0 ms (se 5.1 ms)",
    "camera_source": "optical flow vs gyro on /home/project/data/haibo/wuhu_livo/ros2bag (corr 0.92)"
  }
}
```
