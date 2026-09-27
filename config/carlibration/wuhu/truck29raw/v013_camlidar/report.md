# wuhu/truck29raw/v013_camlidar

- stage: camlidar (相机-LiDAR 偏航/横滚复核)
- parent: wuhu/truck29raw/v009_camloop
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v002_axle -> wuhu/truck29raw/v003_ground -> wuhu/truck29raw/v004_driven -> wuhu/truck29raw/v005_camtime -> wuhu/truck29raw/v006_camlidar -> wuhu/truck29raw/v009_camloop -> wuhu/truck29raw/v013_camlidar
- created: 2026-09-26T18:47:50+08:00
- note: mcp-pole-yaw-roll-u

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- run: results/mcp_t29raw_p02_livo120_r2/Log/fusion/1790413711524616733/
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
      -0.001,
      -0.005
    ]
  }
}
```

## metrics

```
{
  "method": "pole yaw (LiDAR pole axis vs image pole column)",
  "frames_checked": 30,
  "frames_with_poles": 8,
  "observations": [
    {
      "t": 5.0,
      "image": 210,
      "depth_m": 33.7,
      "height_m": 5.9,
      "rows": 254,
      "du_px": -0.51,
      "mad_px": 0.38,
      "yaw_deg": 0.0178,
      "bar_width_px": 5,
      "snr": 2.36,
      "speed_mps": 2.13,
      "roll_deg": 0.2668,
      "u_px": 1045.4
    },
    {
      "t": 8.8,
      "image": 324,
      "depth_m": 24.2,
      "height_m": 7.7,
      "rows": 448,
      "du_px": 4.45,
      "mad_px": 0.91,
      "yaw_deg": -0.1489,
      "bar_width_px": 7,
      "snr": 2.59,
      "speed_mps": 2.4,
      "roll_deg": -0.5075,
      "u_px": 632.9
    },
    {
      "t": 50.5,
      "image": 1576,
      "depth_m": 21.5,
      "height_m": 8.6,
      "rows": 374,
      "du_px": -0.15,
      "mad_px": 1.3,
      "yaw_deg": 0.0049,
      "bar_width_px": 10,
      "snr": 2.6,
      "speed_mps": 6.57,
      "roll_deg": 0.9936,
      "u_px": 1492.1
    },
    {
      "t": 58.1,
      "image": 1803,
      "depth_m": 22.1,
      "height_m": 8.1,
      "rows": 439,
      "du_px": 0.18,
      "mad_px": 0.87,
      "yaw_deg": -0.0056,
      "bar_width_px": 7,
      "snr": 2.31,
      "speed_mps": 6.24,
      "roll_deg": -0.7343,
      "u_px": 1569.2
    },
    {
      "t": 65.7,
      "image": 2031,
      "depth_m": 22.1,
      "height_m": 8.6,
      "rows": 366,
      "du_px": 5.2,
      "mad_px": 0.98,
      "yaw_deg": -0.1557,
      "bar_width_px": 5,
      "snr": 2.81,
      "speed_mps": 6.46,
      "roll_deg": -0.2061,
      "u_px": 1641.1
    }
  ],
  "rejected": [
    {
      "t": 27.8,
      "depth_m": 22.2,
      "bar_px": 26.0,
      "rows": 491,
      "rejected": "rows disagree (MAD 8.4 px over 467 rows, width 10 px)"
    },
    {
      "t": 35.3,
      "depth_m": 22.1,
      "bar_px": 15.9,
      "rows": 512,
      "rejected": "rows disagree (MAD 5.1 px over 458 rows, width 16 px)"
    },
    {
      "t": 42.9,
      "depth_m": 27.9,
      "bar_px": 24.6,
      "rows": 428,
      "rejected": "rows disagree (MAD 3.3 px over 382 rows, width 7 px)"
    }
  ],
  "image_scale": 1.0,
  "roll_delta_deg": 0.0,
  "roll_mad_deg": 0.4729,
  "roll_used": 5,
  "roll_observations": 5,
  "yaw_delta_deg": 0.0049,
  "yaw_mad_deg": 0.0234,
  "used": 3,
  "du_median_px": 0.18,
  "delta_rot_deg_cam_xyz": [
    0.0,
    0.0049,
    0.0
  ],
  "delta_trans_m_cam_xyz": [
    0.0,
    0.0,
    0.0
  ],
  "overlay_dir": "/home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v013_camlidar/overlays",
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
      -96.498,
      1.616,
      0.253
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
