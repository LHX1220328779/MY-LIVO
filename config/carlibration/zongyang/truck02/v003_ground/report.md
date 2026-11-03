# zongyang/truck02/v003_ground

- stage: ground (静止地面精修)
- parent: zongyang/truck02/v002_axle
- lineage: zongyang/truck02/v001_import -> zongyang/truck02/v002_axle -> zongyang/truck02/v003_ground
- created: 2026-10-26T19:09:25+08:00
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
        2.89
      ],
      "drpy_deg": [
        -0.013,
        -0.15,
        0.0
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -5.76
      ],
      "drpy_deg": [
        -0.456,
        -0.042,
        0.005
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
        -0.02052474592937716,
        -0.14929799253276133,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        2.890014226098285
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 15,
        "matches": 3033,
        "ground_before_cm": 8.48,
        "ground_after_cm": 8.44,
        "rotation_deg": 0.151,
        "translation_cm": 2.9
      },
      "plane_check": {
        "tilt_deg": 0.477,
        "fore_aft_deg": 0.474,
        "lateral_deg": 0.047,
        "dz_cm": -12.7,
        "pairs": 1,
        "outside_basin": true,
        "applied": false
      }
    },
    "front_left_lidar": {
      "rotvec_deg": [
        0.0,
        0.0,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        0.0
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "manual",
      "quality": {
        "level": "warn",
        "reasons": [
          "fine solve regressed the ground residual (9.07->9.09 cm); kept the manual matrix (9.07 cm)"
        ],
        "iterations": 22,
        "matches": 3504,
        "ground_before_cm": 9.07,
        "ground_after_cm": 9.07,
        "rotation_deg": 0.0,
        "translation_cm": 0.0
      },
      "plane_check": {
        "tilt_deg": 0.291,
        "fore_aft_deg": -0.109,
        "lateral_deg": -0.27,
        "dz_cm": -0.5,
        "pairs": 1,
        "outside_basin": false,
        "applied": false
      }
    },
    "back_lidar": {
      "rotvec_deg": [
        -0.03916481501450693,
        0.4561062479175991,
        -0.023240065286291377
      ],
      "trans_cm": [
        0.0,
        0.0,
        -5.764764188741364
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "warn",
        "reasons": [
          "plane start applied (1.02 deg / -3.7 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage"
        ],
        "iterations": 12,
        "matches": 26326,
        "ground_before_cm": 8.65,
        "ground_after_cm": 6.01,
        "rotation_deg": 0.458,
        "translation_cm": 5.8
      },
      "plane_check": {
        "tilt_deg": 1.016,
        "fore_aft_deg": 0.289,
        "lateral_deg": -0.98,
        "dz_cm": -3.7,
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
        2.5178,
        4.9185,
        1.8013
      ],
      "rpy_deg": [
        -0.278,
        -0.252,
        -2.862
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.2208,
        -1.0928,
        0.788
      ],
      "rpy_deg": [
        0.004,
        -3.52,
        -90.269
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1639,
        4.8409,
        1.8706
      ],
      "rpy_deg": [
        -1.696,
        -0.402,
        178.305
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.073,
        7.0543,
        2.4626
      ],
      "rpy_deg": [
        1.021,
        1.809,
        91.002
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.108,
      7.0643,
      2.1813
    ],
    "rpy_deg": [
      -90.063,
      2.989,
      -0.013
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
    "lidar": -0.007,
    "lidar_frontend": -0.002,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0070 + imu/ins clock lag +0.0050 (corr 1.000); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/zongyang/truck02/v002_axle/calibration.json
bag=/home/project/data/haibo/zongyang/02/20261026/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 50..90 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 19.1 cm / P90 58.1 cm (limit 50 cm) -> stationary
front_right_lidar: stationary aggregated points 16044
back_lidar: stationary aggregated points 12227
front_left_lidar: stationary aggregated points 19381
front_lidar: stationary aggregated points 40151
front_right_lidar: plane start: own ground vs reference tilt 0.48 deg (fore-aft +0.47, lateral +0.05), height -12.7 cm over 1 plane pair(s) -> start rot deg (x,y,z)=(+0.474,-0.046,-0.004) z -12.7 cm; ground residual 8.48->9.14 cm (n=2913->2937): not used (does not improve the overlap)
front_right_lidar: yaw/xy solved=False; matches 3011->3033; |res| median 8.48->8.42 cm; ground 8.48->8.44 cm (n=2930); vertical 9.64->9.59 cm (n=57); correction rot deg (x,y,z)=(-0.021,-0.149,+0.000) trans cm=(+0.0,+0.0,+2.9); iters 15  [quality: ok]
front_left_lidar: yaw/xy solved=False; matches 3504->3504; |res| median 9.04->9.04 cm; ground 9.07->9.07 cm (n=3385); vertical 3.75->3.75 cm (n=25); correction rot deg (x,y,z)=(+0.000,+0.000,+0.000) trans cm=(+0.0,+0.0,+0.0); iters 22  [quality: warn; fine solve regressed the ground residual (9.07->9.09 cm); kept the manual matrix (9.07 cm)]
back_lidar: plane start: own ground vs reference tilt 1.02 deg (fore-aft +0.29, lateral -0.98), height -3.7 cm over 40 plane pair(s) -> start rot deg (x,y,z)=(+0.289,+0.979,-0.023) z -3.7 cm; ground residual 8.65->7.36 cm (n=26051->26182): used
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 26082->26326; |res| median 8.65->6.01 cm; ground 8.65->6.01 cm (n=26292); vertical nan->nan cm (n=0); correction rot deg (x,y,z)=(-0.039,+0.456,-0.023) trans cm=(+0.0,+0.0,-5.8); iters 12  [quality: warn; plane start applied (1.02 deg / -3.7 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage]
quality: warn  (front_left_lidar: fine solve regressed the ground residual (9.07->9.09 cm); kept the manual matrix (9.07 cm); back_lidar: plane start applied (1.02 deg / -3.7 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage)
written /home/project/MY-LIVO/config/carlibration/zongyang/truck02/v003_ground/calibration.json
```
