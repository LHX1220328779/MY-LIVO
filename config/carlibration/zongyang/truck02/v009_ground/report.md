# zongyang/truck02/v009_ground

- stage: ground (静止地面精修)
- parent: zongyang/truck02/v008_axle
- lineage: zongyang/truck02/v001_import -> zongyang/truck02/v008_axle -> zongyang/truck02/v009_ground
- created: 2026-10-27T11:55:30+08:00
- note: pipeline: ground

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
        2.94
      ],
      "drpy_deg": [
        -0.015,
        -0.142,
        0.0
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -4.35
      ],
      "drpy_deg": [
        -0.567,
        -0.033,
        0.009
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        2.21
      ],
      "drpy_deg": [
        0.044,
        0.049,
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
        -0.022097696977646412,
        -0.14086936543051531,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        2.9326000223623234
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 15,
        "matches": 3144,
        "ground_before_cm": 8.46,
        "ground_after_cm": 8.31,
        "rotation_deg": 0.143,
        "translation_cm": 2.9
      },
      "plane_check": {
        "tilt_deg": 0.239,
        "fore_aft_deg": 0.239,
        "lateral_deg": -0.004,
        "dz_cm": -2.0,
        "pairs": 1,
        "outside_basin": false,
        "applied": false
      }
    },
    "front_left_lidar": {
      "rotvec_deg": [
        -0.04574603191773428,
        -0.047408244888317394,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        2.204898043249173
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 17,
        "matches": 3564,
        "ground_before_cm": 9.1,
        "ground_after_cm": 9.04,
        "rotation_deg": 0.066,
        "translation_cm": 2.2
      },
      "plane_check": {
        "tilt_deg": 0.354,
        "fore_aft_deg": -0.332,
        "lateral_deg": -0.121,
        "dz_cm": 10.6,
        "pairs": 1,
        "outside_basin": true,
        "applied": false
      }
    },
    "back_lidar": {
      "rotvec_deg": [
        -0.03001580294518991,
        0.566911410705037,
        -0.02509383344287152
      ],
      "trans_cm": [
        0.0,
        0.0,
        -4.3542864433131765
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "warn",
        "reasons": [
          "plane start applied (1.07 deg / -2.5 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage"
        ],
        "iterations": 13,
        "matches": 26241,
        "ground_before_cm": 8.98,
        "ground_after_cm": 6.07,
        "rotation_deg": 0.568,
        "translation_cm": 4.4
      },
      "plane_check": {
        "tilt_deg": 1.07,
        "fore_aft_deg": 0.269,
        "lateral_deg": -1.035,
        "dz_cm": -2.5,
        "pairs": 40,
        "outside_basin": true,
        "applied": true
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
        2.5189,
        4.9709,
        1.8001
      ],
      "rpy_deg": [
        -0.257,
        -0.225,
        -2.919
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.2259,
        -1.0372,
        0.7988
      ],
      "rpy_deg": [
        -0.125,
        -3.488,
        -90.321
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1628,
        4.899,
        1.8927
      ],
      "rpy_deg": [
        -1.675,
        -0.372,
        178.248
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0695,
        7.109,
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
      -0.1046,
      7.1192,
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
    "lidar": -0.001,
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
calibration=/home/project/MY-LIVO/config/carlibration/zongyang/truck02/v008_axle/calibration.json
bag=/home/project/data/haibo/zongyang/02/20261026/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 50..90 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 19.1 cm / P90 58.1 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 16128
back_lidar: stationary aggregated points 12222
front_left_lidar: stationary aggregated points 19419
front_lidar: stationary aggregated points 40253
front_right_lidar: yaw/xy solved=False; matches 3125->3144; |res| median 8.45->8.25 cm; ground 8.46->8.31 cm (n=3010); vertical 7.19->7.40 cm (n=74); correction rot deg (x,y,z)=(-0.022,-0.141,+0.000) trans cm=(+0.0,+0.0,+2.9); iters 15  [quality: ok]
front_left_lidar: plane start: own ground vs reference tilt 0.35 deg (fore-aft -0.33, lateral -0.12), height +10.6 cm over 1 plane pair(s) -> start rot deg (x,y,z)=(-0.332,+0.121,+0.001) z +10.6 cm; ground residual 9.10->9.23 cm (n=3430->3389): not used (does not improve the overlap)
front_left_lidar: yaw/xy solved=False; matches 3567->3564; |res| median 9.08->9.05 cm; ground 9.10->9.04 cm (n=3428); vertical 2.95->3.35 cm (n=28); correction rot deg (x,y,z)=(-0.046,-0.047,+0.000) trans cm=(+0.0,+0.0,+2.2); iters 17  [quality: ok]
back_lidar: plane start: own ground vs reference tilt 1.07 deg (fore-aft +0.27, lateral -1.04), height -2.5 cm over 40 plane pair(s) -> start rot deg (x,y,z)=(+0.269,+1.035,-0.025) z -2.5 cm; ground residual 8.98->7.08 cm (n=25911->26089): used
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 25950->26241; |res| median 8.97->6.08 cm; ground 8.98->6.07 cm (n=26195); vertical nan->nan cm (n=0); correction rot deg (x,y,z)=(-0.030,+0.567,-0.025) trans cm=(+0.0,+0.0,-4.4); iters 13  [quality: warn; plane start applied (1.07 deg / -2.5 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage]
quality: warn  (back_lidar: plane start applied (1.07 deg / -2.5 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage)
written /home/project/MY-LIVO/config/carlibration/zongyang/truck02/v009_ground/calibration.json
```
