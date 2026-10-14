# wuhu/truck29raw/v017_ground

- stage: ground (静止地面精修)
- parent: wuhu/truck29raw/v015_axle
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v015_axle -> wuhu/truck29raw/v017_ground
- created: 2026-10-14T16:41:06+08:00
- note: pipeline rerun after ground-stage plane-start fix (front_left_lidar moved vs v001_import)

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
        3.12,
        4.78,
        -0.84
      ],
      "drpy_deg": [
        -0.126,
        0.294,
        0.111
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -5.75
      ],
      "drpy_deg": [
        0.167,
        -0.422,
        0.001
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        1.67,
        2.6,
        -3.09
      ],
      "drpy_deg": [
        6.367,
        -2.454,
        -0.274
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
        -0.11855025551452632,
        0.29627186305878495,
        0.10508290040351304
      ],
      "trans_cm": [
        3.1144554778010916,
        4.78302874073508,
        -0.8401158233368909
      ],
      "yaw_xy_solved": true,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 16,
        "matches": 5464,
        "ground_before_cm": 9.6,
        "ground_after_cm": 6.68,
        "rotation_deg": 0.336,
        "translation_cm": 5.8
      },
      "plane_check": {
        "tilt_deg": 0.205,
        "fore_aft_deg": -0.139,
        "lateral_deg": -0.151,
        "dz_cm": -1.1,
        "pairs": 1,
        "outside_basin": false,
        "applied": false
      }
    },
    "front_left_lidar": {
      "rotvec_deg": [
        -6.286666048365164,
        2.6272417795734584,
        0.09347327617663281
      ],
      "trans_cm": [
        1.6690477482442136,
        2.602713903544184,
        -3.0935534141557515
      ],
      "yaw_xy_solved": true,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "warn",
        "reasons": [
          "plane start applied (6.75 deg / -4.0 cm): the sensor moved relative to the parent calibration; yaw/x/y are re-checked by the driven stage"
        ],
        "iterations": 25,
        "matches": 6411,
        "ground_before_cm": 15.47,
        "ground_after_cm": 8.8,
        "rotation_deg": 6.814,
        "translation_cm": 4.4
      },
      "plane_check": {
        "tilt_deg": 6.749,
        "fore_aft_deg": -6.233,
        "lateral_deg": -2.596,
        "dz_cm": -4.0,
        "pairs": 1,
        "outside_basin": true,
        "applied": true
      }
    },
    "back_lidar": {
      "rotvec_deg": [
        -0.42560017532188144,
        -0.1576293613044073,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -5.74774033408779
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 21,
        "matches": 32410,
        "ground_before_cm": 10.71,
        "ground_after_cm": 9.56,
        "rotation_deg": 0.454,
        "translation_cm": 5.7
      },
      "plane_check": {
        "tilt_deg": 1.925,
        "fore_aft_deg": -0.143,
        "lateral_deg": 1.924,
        "dz_cm": 10.6,
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
        2.7467,
        5.8985,
        1.8019
      ],
      "rpy_deg": [
        -0.991,
        -2.942,
        -1.294
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1185,
        -0.3651,
        0.6432
      ],
      "rpy_deg": [
        -0.988,
        0.007,
        -91.316
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1321,
        5.6584,
        1.9573
      ],
      "rpy_deg": [
        3.131,
        -4.526,
        178.275
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1291,
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
      -0.0164,
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
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v015_axle/calibration.json
bag=/home/project/data/haibo/wuhu/29/20261013/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 810..850 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 10.9 cm / P90 163.5 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 22799
back_lidar: stationary aggregated points 830
front_left_lidar: stationary aggregated points 24917
front_lidar: stationary aggregated points 56850
front_right_lidar: yaw/xy solved=True; matches 5510->5464; |res| median 8.52->6.11 cm; ground 9.60->6.68 cm (n=4412); vertical 7.31->4.74 cm (n=693); correction rot deg (x,y,z)=(-0.119,+0.296,+0.105) trans cm=(+3.1,+4.8,-0.8); iters 16  [quality: ok]
front_left_lidar: plane start: own ground vs reference tilt 6.75 deg (fore-aft -6.23, lateral -2.60), height -4.0 cm over 1 plane pair(s) -> start rot deg (x,y,z)=(-6.232,+2.589,+0.110) z -4.0 cm; ground residual 15.47->8.85 cm (n=2672->5352): used
front_left_lidar: yaw/xy solved=True; matches 3004->6411; |res| median 15.53->7.64 cm; ground 15.47->8.80 cm (n=5335); vertical 16.03->3.60 cm (n=617); correction rot deg (x,y,z)=(-6.287,+2.627,+0.093) trans cm=(+1.7,+2.6,-3.1); iters 25  [quality: warn; plane start applied (6.75 deg / -4.0 cm): the sensor moved relative to the parent calibration; yaw/x/y are re-checked by the driven stage]
back_lidar: plane start: own ground vs reference tilt 1.93 deg (fore-aft -0.14, lateral +1.92), height +10.6 cm over 40 plane pair(s) -> start rot deg (x,y,z)=(-0.143,-1.924,+0.006) z +10.6 cm; ground residual 10.71->16.18 cm (n=29320->24993): not used (does not improve the overlap)
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 32392->32410; |res| median 11.58->10.40 cm; ground 10.71->9.56 cm (n=29334); vertical 25.61->26.71 cm (n=1741); correction rot deg (x,y,z)=(-0.426,-0.158,+0.000) trans cm=(+0.0,+0.0,-5.7); iters 21  [quality: ok]
quality: warn  (front_left_lidar: plane start applied (6.75 deg / -4.0 cm): the sensor moved relative to the parent calibration; yaw/x/y are re-checked by the driven stage)
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v017_ground/calibration.json
```
