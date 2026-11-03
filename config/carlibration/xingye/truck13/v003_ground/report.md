# xingye/truck13/v003_ground

- stage: ground (静止地面精修)
- parent: xingye/truck13/v002_axle
- lineage: xingye/truck13/v001_import -> xingye/truck13/v002_axle -> xingye/truck13/v003_ground
- created: 2026-10-28T11:03:54+08:00
- note: pipeline: ground

## inputs

- bag: /home/project/data/haibo/xingye/13/20261027/ros2bag
- dataset: {'path': '/home/project/data/haibo/xingye/13/20261027/ros2bag', 'kind': 'raw_split', 'segments': 68, 'duration_s': 1020.12, 'has_camera': True}
- stationary: [2, 20]
- moving: [6990, 40]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -2.17,
        -1.65,
        -2.88
      ],
      "drpy_deg": [
        0.218,
        -0.18,
        -0.063
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -8.98
      ],
      "drpy_deg": [
        0.458,
        0.236,
        -0.031
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -6.22,
        0.6,
        -6.9
      ],
      "drpy_deg": [
        -0.474,
        -0.14,
        -0.221
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
        0.2549543079585278,
        -0.12109284690624032,
        -0.06167797474179967
      ],
      "trans_cm": [
        -2.1650166100298383,
        -1.6490951181482476,
        -2.876557992511415
      ],
      "yaw_xy_solved": true,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "warn",
        "reasons": [
          "plane start applied (0.66 deg / -7.6 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage"
        ],
        "iterations": 13,
        "matches": 3612,
        "ground_before_cm": 5.62,
        "ground_after_cm": 1.9,
        "rotation_deg": 0.289,
        "translation_cm": 4.0
      },
      "plane_check": {
        "tilt_deg": 0.662,
        "fore_aft_deg": 0.643,
        "lateral_deg": -0.156,
        "dz_cm": -7.6,
        "pairs": 1,
        "outside_basin": true,
        "applied": true
      }
    },
    "front_left_lidar": {
      "rotvec_deg": [
        0.49245328248145304,
        0.04066576856239458,
        -0.22524329872624155
      ],
      "trans_cm": [
        -6.215178266483271,
        0.6070177690514329,
        -6.89466943163533
      ],
      "yaw_xy_solved": true,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "warn",
        "reasons": [
          "plane start applied (0.51 deg / -6.4 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage"
        ],
        "iterations": 13,
        "matches": 3528,
        "ground_before_cm": 8.11,
        "ground_after_cm": 2.07,
        "rotation_deg": 0.543,
        "translation_cm": 9.3
      },
      "plane_check": {
        "tilt_deg": 0.514,
        "fore_aft_deg": 0.482,
        "lateral_deg": -0.179,
        "dz_cm": -6.4,
        "pairs": 1,
        "outside_basin": true,
        "applied": true
      }
    },
    "back_lidar": {
      "rotvec_deg": [
        0.23698768120591027,
        -0.4566570715653413,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -8.977948710825828
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 11,
        "matches": 41983,
        "ground_before_cm": 9.09,
        "ground_after_cm": 5.19,
        "rotation_deg": 0.514,
        "translation_cm": 9.0
      },
      "plane_check": {
        "tilt_deg": 1.409,
        "fore_aft_deg": 1.333,
        "lateral_deg": 0.339,
        "dz_cm": -21.3,
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
        2.0006,
        6.954,
        1.0263
      ],
      "rpy_deg": [
        -1.791,
        -0.427,
        14.139
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0796,
        -1.9239,
        0.4519
      ],
      "rpy_deg": [
        0.323,
        -3.659,
        -89.967
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -1.8983,
        6.9297,
        1.03
      ],
      "rpy_deg": [
        0.78,
        -0.602,
        168.113
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0002,
        7.0096,
        0.6541
      ],
      "rpy_deg": [
        0.426,
        2.516,
        90.688
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0123,
      7.0409,
      1.4569
    ],
    "rpy_deg": [
      -98.675,
      1.353,
      -0.512
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1642.15,
      1855.56,
      963.247,
      544.29
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
    "lidar": -0.0502,
    "lidar_frontend": -0.0427,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0502 + imu/ins clock lag +0.0075 (corr 0.997); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/xingye/truck13/v002_axle/calibration.json
bag=/home/project/data/haibo/xingye/13/20261027/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 6990..7030 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 8.6 cm / P90 41.6 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 12534
back_lidar: stationary aggregated points 9992
front_left_lidar: stationary aggregated points 12447
front_lidar: stationary aggregated points 21478
front_right_lidar: plane start: own ground vs reference tilt 0.66 deg (fore-aft +0.64, lateral -0.16), height -7.6 cm over 1 plane pair(s) -> start rot deg (x,y,z)=(+0.643,+0.156,-0.001) z -7.6 cm; ground residual 5.62->3.06 cm (n=2107->2170): used
front_right_lidar: yaw/xy solved=True; matches 3582->3612; |res| median 4.43->2.04 cm; ground 5.62->1.90 cm (n=2095); vertical 2.55->2.16 cm (n=1337); correction rot deg (x,y,z)=(+0.255,-0.121,-0.062) trans cm=(-2.2,-1.6,-2.9); iters 13  [quality: warn; plane start applied (0.66 deg / -7.6 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage]
front_left_lidar: plane start: own ground vs reference tilt 0.51 deg (fore-aft +0.48, lateral -0.18), height -6.4 cm over 1 plane pair(s) -> start rot deg (x,y,z)=(+0.482,+0.179,-0.001) z -6.4 cm; ground residual 8.11->2.84 cm (n=2089->2106): used
front_left_lidar: yaw/xy solved=True; matches 3531->3528; |res| median 6.20->2.29 cm; ground 8.11->2.07 cm (n=2099); vertical 4.60->2.46 cm (n=1264); correction rot deg (x,y,z)=(+0.492,+0.041,-0.225) trans cm=(-6.2,+0.6,-6.9); iters 13  [quality: warn; plane start applied (0.51 deg / -6.4 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage]
back_lidar: plane start: own ground vs reference tilt 1.41 deg (fore-aft +1.33, lateral +0.34), height -21.3 cm over 40 plane pair(s) -> start rot deg (x,y,z)=(+1.332,-0.339,+0.016) z -21.3 cm; ground residual 9.09->24.82 cm (n=30880->27721): not used (does not improve the overlap)
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 41044->41983; |res| median 7.56->5.11 cm; ground 9.09->5.19 cm (n=31771); vertical nan->nan cm (n=0); correction rot deg (x,y,z)=(+0.237,-0.457,+0.000) trans cm=(+0.0,+0.0,-9.0); iters 11  [quality: ok]
quality: warn  (front_right_lidar: plane start applied (0.66 deg / -7.6 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage; front_left_lidar: plane start applied (0.51 deg / -6.4 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage)
written /home/project/MY-LIVO/config/carlibration/xingye/truck13/v003_ground/calibration.json
```
