# zongyang/truck07/v003_ground

- stage: ground (静止地面精修)
- parent: zongyang/truck07/v002_axle
- lineage: zongyang/truck07/v001_import -> zongyang/truck07/v002_axle -> zongyang/truck07/v003_ground
- created: 2026-10-08T17:29:50+08:00
- note: pipeline: ground

## inputs

- bag: /home/project/data/haibo/zongyang/07/20261007/ros2bag
- dataset: {'path': '/home/project/data/haibo/zongyang/07/20261007/ros2bag', 'kind': 'raw_split', 'segments': 12, 'duration_s': 359.99, 'has_camera': False}
- stationary: [2, 20]
- moving: [2750, 40]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -0.1
      ],
      "drpy_deg": [
        0.236,
        0.042,
        0.004
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -1.83
      ],
      "drpy_deg": [
        -0.49,
        -0.406,
        -0.007
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        1.44
      ],
      "drpy_deg": [
        -0.014,
        0.225,
        0.0
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
        0.23576689089201694,
        0.0405002967407208,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -0.0926817027854639
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "front_left_lidar": {
      "rotvec_deg": [
        0.011434317795864407,
        -0.22578994009563466,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        1.435170448756437
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "back_lidar": {
      "rotvec_deg": [
        -0.40329563988596606,
        0.49246613539344464,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -1.8276398226083184
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
        2.7655,
        5.1845,
        1.7483
      ],
      "rpy_deg": [
        -2.143,
        1.026,
        -0.333
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1687,
        -0.8432,
        0.6762
      ],
      "rpy_deg": [
        0.433,
        0.696,
        -90.399
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.2282,
        4.9732,
        1.8022
      ],
      "rpy_deg": [
        2.406,
        0.164,
        179.344
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1092,
        7.2631,
        2.5109
      ],
      "rpy_deg": [
        0.485,
        2.809,
        89.258
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.1403,
      7.2764,
      2.4087
    ],
    "rpy_deg": [
      -90.448,
      1.632,
      0.97
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      621.1,
      900.65,
      958.95,
      537.17
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
            621.1,
            900.65,
            958.95,
            537.17
          ],
          "rms_px": 0.309,
          "circle_centre_rear_m": [
            -0.01,
            7.536
          ],
          "ground_z_rear_m": -1.263
        },
        "square": {
          "intrinsics": [
            621.33,
            621.33,
            1050.56,
            537.09
          ],
          "rms_px": 2.404,
          "circle_centre_rear_m": [
            -0.942,
            7.544
          ],
          "ground_z_rear_m": -2.916
        }
      },
      "model": "independent fx / fy",
      "rms_px": 0.309,
      "camera_height_m": 3.672,
      "deviation_vs_truck29_defaults": {
        "fx_pct": -62.19,
        "fy_pct": -51.46,
        "cx_px": -12.1,
        "cy_px": 2.9
      }
    },
    "source": "supplier ground-distance curves equation_*m (independent fx / fy, rms 0.309 px)"
  },
  "time_offsets": {
    "lidar": -0.0041,
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
calibration=/home/project/MY-LIVO/config/carlibration/zongyang/truck07/v002_axle/calibration.json
bag=/home/project/data/haibo/zongyang/07/20261007/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 2750..2790 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 11.1 cm / P90 53.2 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 17925
back_lidar: stationary aggregated points 31954
front_left_lidar: stationary aggregated points 16708
front_lidar: stationary aggregated points 21589
front_right_lidar: yaw/xy solved=False; matches 2397->2405; |res| median 9.60->6.16 cm; ground 9.60->6.16 cm (n=2405); vertical nan->nan cm (n=0); correction rot deg (x,y,z)=(+0.236,+0.041,+0.000) trans cm=(+0.0,+0.0,-0.1); iters 13
front_left_lidar: yaw/xy solved=False; matches 2556->2565; |res| median 6.44->5.94 cm; ground 6.44->5.94 cm (n=2565); vertical nan->nan cm (n=0); correction rot deg (x,y,z)=(+0.011,-0.226,+0.000) trans cm=(+0.0,+0.0,+1.4); iters 11
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 28116->28084; |res| median 7.08->5.67 cm; ground 7.07->5.66 cm (n=28025); vertical n=22; correction rot deg (x,y,z)=(-0.403,+0.492,+0.000) trans cm=(+0.0,+0.0,-1.8); iters 11
written /home/project/MY-LIVO/config/carlibration/zongyang/truck07/v003_ground/calibration.json
```
