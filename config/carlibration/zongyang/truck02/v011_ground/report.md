# zongyang/truck02/v011_ground

- stage: ground (静止地面精修)
- parent: zongyang/truck02/v010_axle
- lineage: zongyang/truck02/v001_import -> zongyang/truck02/v008_axle -> zongyang/truck02/v009_ground -> zongyang/truck02/v010_axle -> zongyang/truck02/v011_ground
- created: 2026-10-27T13:40:28+08:00
- note: agent: start auto calibration after empty body_filter fix

## inputs

- bag: /home/project/data/haibo/zongyang/02/20261026/ros2bag
- dataset: {'path': '/home/project/data/haibo/zongyang/02/20261026/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 239.98, 'has_camera': False}
- stationary: [2, 20]
- moving: [50, 40]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        2.19
      ],
      "drpy_deg": [
        -0.049,
        -0.018,
        0.0
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        1.84
      ],
      "drpy_deg": [
        0.048,
        -0.085,
        -0.003
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        1.48
      ],
      "drpy_deg": [
        0.041,
        -0.008,
        -0.001
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
        -0.04993276475516328,
        -0.015189083877862575,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        2.1966452931583604
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 10,
        "matches": 3180,
        "ground_before_cm": 8.31,
        "ground_after_cm": 8.29,
        "rotation_deg": 0.052,
        "translation_cm": 2.2
      },
      "plane_check": {
        "tilt_deg": 0.499,
        "fore_aft_deg": 0.414,
        "lateral_deg": -0.279,
        "dz_cm": -11.6,
        "pairs": 1,
        "outside_basin": true,
        "applied": false
      }
    },
    "front_left_lidar": {
      "rotvec_deg": [
        -0.04042319422858173,
        0.009642324217596775,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        1.4862115645307203
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 9,
        "matches": 3549,
        "ground_before_cm": 9.05,
        "ground_after_cm": 9.0,
        "rotation_deg": 0.042,
        "translation_cm": 1.5
      },
      "plane_check": {
        "tilt_deg": 0.103,
        "fore_aft_deg": -0.096,
        "lateral_deg": -0.039,
        "dz_cm": 1.7,
        "pairs": 1,
        "outside_basin": false,
        "applied": false
      }
    },
    "back_lidar": {
      "rotvec_deg": [
        -0.08546640869480882,
        -0.04782369185289666,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        1.8417790971827732
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 11,
        "matches": 26225,
        "ground_before_cm": 6.31,
        "ground_after_cm": 6.06,
        "rotation_deg": 0.098,
        "translation_cm": 1.8
      },
      "plane_check": {
        "tilt_deg": 0.496,
        "fore_aft_deg": 0.258,
        "lateral_deg": -0.443,
        "dz_cm": 4.1,
        "pairs": 40,
        "outside_basin": false,
        "applied": false
      }
    }
  },
  "quality": "ok"
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
        2.518,
        4.9782,
        1.822
      ],
      "rpy_deg": [
        -0.306,
        -0.243,
        -2.919
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.2268,
        -1.03,
        0.8172
      ],
      "rpy_deg": [
        -0.077,
        -3.573,
        -90.324
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1637,
        4.9062,
        1.9075
      ],
      "rpy_deg": [
        -1.634,
        -0.38,
        178.247
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0705,
        7.1162,
        2.4626
      ],
      "rpy_deg": [
        1.039,
        1.785,
        90.946
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.1056,
      7.1264,
      2.1813
    ],
    "rpy_deg": [
      -90.039,
      3.007,
      -0.068
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      611.054443359375,
      895.9816284179688,
      954.4519112515845,
      541.4709836341353
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
    "lidar": 0.0033,
    "lidar_frontend": 0.004,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0010 + imu/ins clock lag +0.0050 (corr 1.000); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/zongyang/truck02/v010_axle/calibration.json
bag=/home/project/data/haibo/zongyang/02/20261026/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 50..90 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 19.1 cm / P90 58.1 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 16208
back_lidar: stationary aggregated points 12041
front_left_lidar: stationary aggregated points 19405
front_lidar: stationary aggregated points 40241
front_right_lidar: plane start: own ground vs reference tilt 0.50 deg (fore-aft +0.41, lateral -0.28), height -11.6 cm over 1 plane pair(s) -> start rot deg (x,y,z)=(+0.414,+0.279,-0.008) z -11.6 cm; ground residual 8.31->9.88 cm (n=3035->3024): not used (does not improve the overlap)
front_right_lidar: yaw/xy solved=False; matches 3175->3180; |res| median 8.25->8.25 cm; ground 8.31->8.29 cm (n=3039); vertical 6.67->6.97 cm (n=81); correction rot deg (x,y,z)=(-0.050,-0.015,+0.000) trans cm=(+0.0,+0.0,+2.2); iters 10  [quality: ok]
front_left_lidar: yaw/xy solved=False; matches 3553->3549; |res| median 9.04->9.00 cm; ground 9.05->9.00 cm (n=3397); vertical 6.17->5.72 cm (n=38); correction rot deg (x,y,z)=(-0.040,+0.010,+0.000) trans cm=(+0.0,+0.0,+1.5); iters 9  [quality: ok]
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 26203->26225; |res| median 6.31->6.06 cm; ground 6.31->6.06 cm (n=26186); vertical nan->nan cm (n=0); correction rot deg (x,y,z)=(-0.085,-0.048,+0.000) trans cm=(+0.0,+0.0,+1.8); iters 11  [quality: ok]
quality: ok
written /home/project/MY-LIVO/config/carlibration/zongyang/truck02/v011_ground/calibration.json
```
