# wuhu/truck29raw/v029_ground

- stage: ground (静止地面精修)
- parent: wuhu/truck29raw/v028_axle
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v028_axle -> wuhu/truck29raw/v029_ground
- created: 2026-10-14T23:23:20+08:00
- note: agent 2026-10-15: driven same-time pairing (archive, raw density) + whole-recording lateral axle, from v001_import

## inputs

- bag: /home/project/data/haibo/wuhu/29/20261013/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/29/20261013/ros2bag', 'kind': 'raw_split', 'segments': 58, 'duration_s': 869.98, 'has_camera': True}
- stationary: [2, 20]
- moving: [810, 40]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        3.05,
        4.88,
        -0.79
      ],
      "drpy_deg": [
        -0.127,
        0.287,
        0.108
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -5.55
      ],
      "drpy_deg": [
        0.164,
        -0.418,
        0.001
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        1.55,
        3.02,
        -2.96
      ],
      "drpy_deg": [
        6.369,
        -2.439,
        -0.279
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
        -0.12022190690348655,
        0.2898819033493801,
        0.10170351721660748
      ],
      "trans_cm": [
        3.048808744201054,
        4.882253440556311,
        -0.7850459744707222
      ],
      "yaw_xy_solved": true,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 19,
        "matches": 5484,
        "ground_before_cm": 9.53,
        "ground_after_cm": 6.7,
        "rotation_deg": 0.33,
        "translation_cm": 5.8
      },
      "plane_check": {
        "tilt_deg": 0.205,
        "fore_aft_deg": -0.14,
        "lateral_deg": -0.15,
        "dz_cm": -1.1,
        "pairs": 1,
        "outside_basin": false,
        "applied": false
      }
    },
    "front_left_lidar": {
      "rotvec_deg": [
        -6.289624494493789,
        2.6122535196676124,
        0.08747785049395428
      ],
      "trans_cm": [
        1.5596971407001357,
        3.023812515598985,
        -2.9655032522577423
      ],
      "yaw_xy_solved": true,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "warn",
        "reasons": [
          "plane start applied (6.75 deg / -3.9 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage"
        ],
        "iterations": 25,
        "matches": 6468,
        "ground_before_cm": 15.64,
        "ground_after_cm": 8.65,
        "rotation_deg": 6.811,
        "translation_cm": 4.5
      },
      "plane_check": {
        "tilt_deg": 6.748,
        "fore_aft_deg": -6.232,
        "lateral_deg": -2.597,
        "dz_cm": -3.9,
        "pairs": 1,
        "outside_basin": true,
        "applied": true
      }
    },
    "back_lidar": {
      "rotvec_deg": [
        -0.4215231648414212,
        -0.15497145187959316,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -5.544636914130138
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 21,
        "matches": 32528,
        "ground_before_cm": 10.57,
        "ground_after_cm": 9.45,
        "rotation_deg": 0.449,
        "translation_cm": 5.5
      },
      "plane_check": {
        "tilt_deg": 2.031,
        "fore_aft_deg": -0.126,
        "lateral_deg": 2.028,
        "dz_cm": 10.4,
        "pairs": 40,
        "outside_basin": true,
        "applied": false
      }
    }
  },
  "quality": "warn"
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
        2.7692,
        5.8995,
        1.8024
      ],
      "rpy_deg": [
        -0.992,
        -2.949,
        -1.297
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0954,
        -0.3651,
        0.6452
      ],
      "rpy_deg": [
        -0.991,
        0.011,
        -91.316
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1101,
        5.6626,
        1.9586
      ],
      "rpy_deg": [
        3.133,
        -4.511,
        178.27
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
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "lidar_frontend": -0.0311,
    "lidar_frontend_source": "axle: lidar -0.0361 + imu/ins clock lag +0.0050 (corr 0.998); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v028_axle/calibration.json
bag=/home/project/data/haibo/wuhu/29/20261013/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 810..850 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 10.9 cm / P90 163.5 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 22803
back_lidar: stationary aggregated points 820
front_left_lidar: stationary aggregated points 24944
front_lidar: stationary aggregated points 56876
front_right_lidar: yaw/xy solved=True; matches 5542->5484; |res| median 8.53->6.12 cm; ground 9.53->6.70 cm (n=4412); vertical 7.38->5.03 cm (n=677); correction rot deg (x,y,z)=(-0.120,+0.290,+0.102) trans cm=(+3.0,+4.9,-0.8); iters 19  [quality: ok]
front_left_lidar: plane start: own ground vs reference tilt 6.75 deg (fore-aft -6.23, lateral -2.60), height -3.9 cm over 1 plane pair(s) -> start rot deg (x,y,z)=(-6.231,+2.589,+0.110) z -3.9 cm; ground residual 15.64->8.77 cm (n=2731->5363): used
front_left_lidar: yaw/xy solved=True; matches 3105->6468; |res| median 15.79->7.58 cm; ground 15.64->8.65 cm (n=5372); vertical 17.91->3.40 cm (n=601); correction rot deg (x,y,z)=(-6.290,+2.612,+0.087) trans cm=(+1.6,+3.0,-3.0); iters 25  [quality: warn; plane start applied (6.75 deg / -3.9 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage]
back_lidar: plane start: own ground vs reference tilt 2.03 deg (fore-aft -0.13, lateral +2.03), height +10.4 cm over 40 plane pair(s) -> start rot deg (x,y,z)=(-0.126,-2.028,+0.005) z +10.4 cm; ground residual 10.57->16.56 cm (n=29455->24634): not used (does not improve the overlap)
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 32450->32528; |res| median 11.54->10.35 cm; ground 10.57->9.45 cm (n=29506); vertical 25.27->26.23 cm (n=1693); correction rot deg (x,y,z)=(-0.422,-0.155,+0.000) trans cm=(+0.0,+0.0,-5.5); iters 21  [quality: ok]
quality: warn  (front_left_lidar: plane start applied (6.75 deg / -3.9 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage)
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v029_ground/calibration.json
```
