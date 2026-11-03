# xingye/truck08/v003_ground

- stage: ground (静止地面精修)
- parent: xingye/truck08/v002_axle
- lineage: xingye/truck08/v001_import -> xingye/truck08/v002_axle -> xingye/truck08/v003_ground
- created: 2026-10-20T16:58:09+08:00
- note: pipeline: ground

## inputs

- bag: /home/project/data/haibo/xingye/08/20261018/ros2bag
- dataset: {'path': '/home/project/data/haibo/xingye/08/20261018/ros2bag', 'kind': 'raw_split', 'segments': 10, 'duration_s': 299.99, 'has_camera': False}
- stationary: [2, 20]
- moving: [2520, 40]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        2.59,
        -1.9,
        -3.73
      ],
      "drpy_deg": [
        0.33,
        0.125,
        0.096
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -7.74
      ],
      "drpy_deg": [
        -0.541,
        0.64,
        0.01
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -1.19,
        0.74,
        -3.25
      ],
      "drpy_deg": [
        -0.328,
        -0.019,
        -0.085
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
        0.2892258677142475,
        0.2026813880731128,
        0.09510878480894755
      ],
      "trans_cm": [
        2.596689607852711,
        -1.9001146953435404,
        -3.730537629151183
      ],
      "yaw_xy_solved": true,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 25,
        "matches": 4767,
        "ground_before_cm": 12.59,
        "ground_after_cm": 8.03,
        "rotation_deg": 0.366,
        "translation_cm": 4.9
      },
      "plane_check": {
        "tilt_deg": 7.851,
        "fore_aft_deg": 2.716,
        "lateral_deg": -7.373,
        "dz_cm": -146.9,
        "pairs": 1,
        "outside_basin": true,
        "applied": false
      }
    },
    "front_left_lidar": {
      "rotvec_deg": [
        0.32340026257208143,
        -0.05698462680676269,
        -0.09364445423868564
      ],
      "trans_cm": [
        -1.184661102526864,
        0.7489572746086861,
        -3.250885435348992
      ],
      "yaw_xy_solved": true,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 15,
        "matches": 4970,
        "ground_before_cm": 10.64,
        "ground_after_cm": 6.55,
        "rotation_deg": 0.341,
        "translation_cm": 3.5
      },
      "plane_check": {
        "tilt_deg": 4.891,
        "fore_aft_deg": 3.71,
        "lateral_deg": -3.202,
        "dz_cm": -109.2,
        "pairs": 1,
        "outside_basin": true,
        "applied": false
      }
    },
    "back_lidar": {
      "rotvec_deg": [
        0.6407592066745372,
        0.5392893061216929,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -7.742327605775295
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 25,
        "matches": 20280,
        "ground_before_cm": 13.15,
        "ground_after_cm": 5.98,
        "rotation_deg": 0.837,
        "translation_cm": 7.7
      },
      "plane_check": {
        "tilt_deg": 2.101,
        "fore_aft_deg": -1.145,
        "lateral_deg": 1.779,
        "dz_cm": 3.5,
        "pairs": 40,
        "outside_basin": true,
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
        1.8773,
        6.921,
        1.1296
      ],
      "rpy_deg": [
        -0.823,
        0.279,
        14.352
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.2914,
        -1.9658,
        0.6063
      ],
      "rpy_deg": [
        -0.484,
        -0.777,
        -90.148
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -2.039,
        6.9437,
        1.1994
      ],
      "rpy_deg": [
        -0.008,
        -1.491,
        166.565
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0833,
        6.9974,
        0.7941
      ],
      "rpy_deg": [
        1.031,
        3.331,
        90.436
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.1729,
      6.9547,
      1.384
    ],
    "rpy_deg": [
      -101.245,
      2.746,
      -1.747
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1642.35,
      1855.45,
      959.039,
      539.189
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
    "lidar": -0.0054,
    "lidar_frontend": 0.0021,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0054 + imu/ins clock lag +0.0075 (corr 0.999); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/xingye/truck08/v002_axle/calibration.json
bag=/home/project/data/haibo/xingye/08/20261018/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 2520..2560 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 8.0 cm / P90 30.2 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 13945
back_lidar: stationary aggregated points 7282
front_left_lidar: stationary aggregated points 21490
front_lidar: stationary aggregated points 24704
front_right_lidar: plane start: own ground vs reference tilt 7.85 deg (fore-aft +2.72, lateral -7.37), height -146.9 cm over 1 plane pair(s) -> start rot deg (x,y,z)=(+2.711,+7.367,+0.105) z -146.9 cm; ground residual 12.59->21.24 cm (n=3425->2010): not used (does not improve the overlap)
front_right_lidar: yaw/xy solved=True; matches 4826->4767; |res| median 8.07->5.98 cm; ground 12.59->8.03 cm (n=3384); vertical 2.98->2.87 cm (n=710); correction rot deg (x,y,z)=(+0.289,+0.203,+0.095) trans cm=(+2.6,-1.9,-3.7); iters 25  [quality: ok]
front_left_lidar: plane start: own ground vs reference tilt 4.89 deg (fore-aft +3.71, lateral -3.20), height -109.2 cm over 1 plane pair(s) -> start rot deg (x,y,z)=(+3.695,+3.200,-0.182) z -109.2 cm; ground residual 10.64->21.67 cm (n=3529->2377): not used (does not improve the overlap)
front_left_lidar: yaw/xy solved=True; matches 4996->4970; |res| median 7.66->5.04 cm; ground 10.64->6.55 cm (n=3526); vertical 2.63->2.48 cm (n=713); correction rot deg (x,y,z)=(+0.323,-0.057,-0.094) trans cm=(-1.2,+0.7,-3.3); iters 15  [quality: ok]
back_lidar: plane start: own ground vs reference tilt 2.10 deg (fore-aft -1.15, lateral +1.78), height +3.5 cm over 40 plane pair(s) -> start rot deg (x,y,z)=(-1.144,-1.778,+0.013) z +3.5 cm; ground residual 13.15->16.52 cm (n=14386->12276): not used (does not improve the overlap)
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 19610->20280; |res| median 13.18->7.40 cm; ground 13.15->5.98 cm (n=14943); vertical 17.83->19.94 cm (n=1978); correction rot deg (x,y,z)=(+0.641,+0.539,+0.000) trans cm=(+0.0,+0.0,-7.7); iters 25  [quality: ok]
quality: ok
written /home/project/MY-LIVO/config/carlibration/xingye/truck08/v003_ground/calibration.json
```
