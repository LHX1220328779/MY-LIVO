# wuhu/truck29/v077_ground

- stage: ground (静止地面精修)
- parent: wuhu/truck29/v076_axle
- lineage: wuhu/truck29/v073_import -> wuhu/truck29/v076_axle -> wuhu/truck29/v077_ground
- created: 2026-10-11T23:01:24+08:00
- note: pipeline: ground

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- stationary: [2, 20]
- moving: [300, 40]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        1.32
      ],
      "drpy_deg": [
        -0.123,
        0.356,
        0.007
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        1.97
      ],
      "drpy_deg": [
        0.128,
        0.228,
        0.001
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -0.86
      ],
      "drpy_deg": [
        -0.14,
        1.308,
        0.003
      ]
    }
  }
}
```

## metrics

```
{
  "corrections": {
    "front_right_lidar": {
      "rotvec_deg": [
        -0.11375252532893412,
        0.35971172011967334,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        1.3162870586212532
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "front_left_lidar": {
      "rotvec_deg": [
        0.10584386170568374,
        -1.311256874923346,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -0.8598019669626147
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "back_lidar": {
      "rotvec_deg": [
        0.22455630770273077,
        -0.13309519679281726,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        1.9722373004125837
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    }
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
        2.7326,
        5.9395,
        1.8261
      ],
      "rpy_deg": [
        -1.016,
        -2.913,
        -1.458
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1075,
        -0.274,
        0.7245
      ],
      "rpy_deg": [
        -0.994,
        0.629,
        -91.376
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1321,
        5.7274,
        1.9791
      ],
      "rpy_deg": [
        -3.349,
        -0.73,
        178.49
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1088,
        8.0401,
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
      0.0029,
      8.0422,
      1.7517
    ],
    "rpy_deg": [
      -96.506,
      1.636,
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
    "intrinsics_check": {
      "method": "supplier ground-distance curves (midrange_camera[0].equation_*m)",
      "curves": 10,
      "distances_m": [
        10.0,
        20.0,
        30.0,
        40.0,
        50.0,
        60.0,
        70.0,
        80.0,
        90.0,
        100.0
      ],
      "fits": {
        "free": {
          "intrinsics": [
            1642.2,
            1846.33,
            968.83,
            532.82
          ],
          "rms_px": 0.07,
          "circle_centre_rear_m": [
            0.007,
            7.533
          ],
          "ground_z_rear_m": -1.264
        },
        "square": {
          "intrinsics": [
            1642.2,
            1642.2,
            917.71,
            510.63
          ],
          "rms_px": 0.36,
          "circle_centre_rear_m": [
            0.255,
            7.49
          ],
          "ground_z_rear_m": -1.64
        }
      },
      "model": "independent fx / fy",
      "rms_px": 0.07,
      "camera_height_m": 3.019,
      "deviation_vs_truck29_defaults": {
        "fx_pct": -0.02,
        "fy_pct": -0.5,
        "cx_px": -2.2,
        "cy_px": -1.4
      }
    },
    "source": "defaults (wuhu truck29 yaml), confirmed by the supplier ground-distance curves (rms 0.07 px)"
  },
  "time_offsets": {
    "lidar": -0.1459,
    "lidar_frontend": -0.125,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v076_axle/calibration.json
bag=/home/project/data/haibo/wuhu_livo/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 300..340 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 11.3 cm / P90 35.0 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 12228
back_lidar: stationary aggregated points 2427
front_left_lidar: stationary aggregated points 18528
front_lidar: stationary aggregated points 43542
front_right_lidar: yaw/xy solved=False; matches 3775->3751; |res| median 6.06->4.02 cm; ground 6.07->4.01 cm (n=3663); vertical 4.89->3.11 cm (n=38); correction rot deg (x,y,z)=(-0.114,+0.360,+0.000) trans cm=(+0.0,+0.0,+1.3); iters 17
front_left_lidar: yaw/xy solved=False; matches 4543->4862; |res| median 16.46->5.10 cm; ground 16.68->5.10 cm (n=4745); vertical 5.50->5.09 cm (n=53); correction rot deg (x,y,z)=(+0.106,-1.311,+0.000) trans cm=(+0.0,+0.0,-0.9); iters 25
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 36617->36769; |res| median 8.22->7.97 cm; ground 7.83->7.61 cm (n=31445); vertical n=3995; correction rot deg (x,y,z)=(+0.225,-0.133,+0.000) trans cm=(+0.0,+0.0,+2.0); iters 16
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v077_ground/calibration.json
```
