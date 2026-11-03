# wuhu/truck29-0930/v014_ground

- stage: ground (静止地面精修)
- parent: wuhu/truck29-0930/v013_axle
- lineage: wuhu/truck29-0930/v002_import -> wuhu/truck29-0930/v013_axle -> wuhu/truck29-0930/v014_ground
- created: 2026-10-19T17:27:05+08:00
- note: pipeline: ground

## inputs

- bag: /home/project/data/haibo/wuhu/29/20261013/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/29/20261013/ros2bag', 'kind': 'raw_split', 'segments': 58, 'duration_s': 869.98, 'has_camera': True}
- stationary: [2, 20]
- moving: [1260, 40]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        2.64,
        3.65,
        -1.17
      ],
      "drpy_deg": [
        -0.009,
        0.056,
        0.089
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -4.38
      ],
      "drpy_deg": [
        0.096,
        -0.284,
        0.0
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -5.04
      ],
      "drpy_deg": [
        -0.167,
        -0.117,
        0.008
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
        -0.00785803578195938,
        0.05617787519412676,
        0.08857064688944878
      ],
      "trans_cm": [
        2.6414711601080687,
        3.652303419883761,
        -1.1774510888201477
      ],
      "yaw_xy_solved": true,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 15,
        "matches": 5365,
        "ground_before_cm": 6.93,
        "ground_after_cm": 6.8,
        "rotation_deg": 0.105,
        "translation_cm": 4.7
      },
      "plane_check": {
        "tilt_deg": 0.101,
        "fore_aft_deg": -0.013,
        "lateral_deg": 0.1,
        "dz_cm": -2.1,
        "pairs": 1,
        "outside_basin": false,
        "applied": false
      }
    },
    "front_left_lidar": {
      "rotvec_deg": [
        0.1695543451226437,
        0.11223300218806169,
        -0.005086584415059723
      ],
      "trans_cm": [
        0.0,
        0.0,
        -5.043290043681474
      ],
      "yaw_xy_solved": true,
      "about": "lidar position in rear-axle frame",
      "chosen": "plane start",
      "quality": {
        "level": "warn",
        "reasons": [
          "fine solve regressed the ground residual (8.40->8.50 cm); kept the plane start matrix (8.40 cm)",
          "plane start applied (0.20 deg / -5.0 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage"
        ],
        "iterations": 17,
        "matches": 6581,
        "ground_before_cm": 8.72,
        "ground_after_cm": 8.4,
        "rotation_deg": 0.203,
        "translation_cm": 5.0
      },
      "plane_check": {
        "tilt_deg": 0.203,
        "fore_aft_deg": 0.17,
        "lateral_deg": -0.112,
        "dz_cm": -5.0,
        "pairs": 1,
        "outside_basin": true,
        "applied": true
      }
    },
    "back_lidar": {
      "rotvec_deg": [
        -0.28580665486582013,
        -0.08944021445002444,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -4.376860148547307
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 19,
        "matches": 23392,
        "ground_before_cm": 9.17,
        "ground_after_cm": 8.46,
        "rotation_deg": 0.299,
        "translation_cm": 4.4
      },
      "plane_check": {
        "tilt_deg": 0.399,
        "fore_aft_deg": -0.128,
        "lateral_deg": 0.183,
        "dz_cm": -8.8,
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
        2.7605,
        5.8945,
        1.8081
      ],
      "rpy_deg": [
        -0.997,
        -2.938,
        -1.289
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1234,
        -0.3683,
        0.6554
      ],
      "rpy_deg": [
        -1.052,
        0.156,
        -91.346
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1208,
        5.5906,
        1.9545
      ],
      "rpy_deg": [
        3.131,
        -4.49,
        178.216
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1296,
        7.9462,
        0.051
      ],
      "rpy_deg": [
        1.477,
        -1.57,
        89.955
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0171,
      7.9472,
      1.7516
    ],
    "rpy_deg": [
      -96.467,
      1.661,
      0.456
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
    "source": "midrange_camera[0].camera_in_params"
  },
  "time_offsets": {
    "lidar": -0.0372,
    "lidar_frontend": -0.0322,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0372 + imu/ins clock lag +0.0050 (corr 0.998); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29-0930/v013_axle/calibration.json
bag=/home/project/data/haibo/wuhu/29/20261013/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 1260..1300 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 10.9 cm / P90 163.5 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 22765
back_lidar: stationary aggregated points 829
front_left_lidar: stationary aggregated points 24954
front_lidar: stationary aggregated points 56840
front_right_lidar: yaw/xy solved=True; matches 5419->5365; |res| median 6.72->6.21 cm; ground 6.93->6.80 cm (n=4319); vertical 6.53->4.59 cm (n=658); correction rot deg (x,y,z)=(-0.008,+0.056,+0.089) trans cm=(+2.6,+3.7,-1.2); iters 15  [quality: ok]
front_left_lidar: plane start: own ground vs reference tilt 0.20 deg (fore-aft +0.17, lateral -0.11), height -5.0 cm over 1 plane pair(s) -> start rot deg (x,y,z)=(+0.170,+0.112,-0.005) z -5.0 cm; ground residual 8.72->8.40 cm (n=5451->5454): used
front_left_lidar: yaw/xy solved=True; matches 6591->6581; |res| median 8.19->7.81 cm; ground 8.72->8.40 cm (n=5454); vertical 7.44->6.90 cm (n=662); correction rot deg (x,y,z)=(+0.170,+0.112,-0.005) trans cm=(+0.0,+0.0,-5.0); iters 17  [quality: warn; fine solve regressed the ground residual (8.40->8.50 cm); kept the plane start matrix (8.40 cm); plane start applied (0.20 deg / -5.0 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage]
back_lidar: plane start: own ground vs reference tilt 0.40 deg (fore-aft -0.13, lateral +0.18), height -8.8 cm over 40 plane pair(s) -> start rot deg (x,y,z)=(-0.128,-0.183,+0.005) z -8.8 cm; ground residual 9.17->9.86 cm (n=23193->23091): not used (does not improve the overlap)
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 23239->23392; |res| median 9.17->8.47 cm; ground 9.17->8.46 cm (n=23339); vertical 21.11->17.64 cm (n=18); correction rot deg (x,y,z)=(-0.286,-0.089,+0.000) trans cm=(+0.0,+0.0,-4.4); iters 19  [quality: ok]
quality: warn  (front_left_lidar: fine solve regressed the ground residual (8.40->8.50 cm); kept the plane start matrix (8.40 cm), plane start applied (0.20 deg / -5.0 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage)
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29-0930/v014_ground/calibration.json
```
