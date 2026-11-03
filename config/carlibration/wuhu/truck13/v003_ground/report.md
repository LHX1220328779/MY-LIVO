# wuhu/truck13/v003_ground

- stage: ground (静止地面精修)
- parent: wuhu/truck13/v002_axle
- lineage: wuhu/truck13/v001_import -> wuhu/truck13/v002_axle -> wuhu/truck13/v003_ground
- created: 2026-10-26T17:24:08+08:00
- note: pipeline: ground

## inputs

- bag: /home/project/data/haibo/wuhu/13/1/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/13/1/ros2bag', 'kind': 'raw_split', 'segments': 2, 'duration_s': 60.0, 'has_camera': False}
- stationary: [2, 20]
- moving: [360, 40]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        2.72
      ],
      "drpy_deg": [
        -0.105,
        0.134,
        0.002
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        15.79
      ],
      "drpy_deg": [
        0.182,
        -0.205,
        0.007
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        3.44
      ],
      "drpy_deg": [
        0.256,
        0.126,
        -0.009
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
        -0.1022607231191811,
        0.13634500575399988,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        2.7199899822930056
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 5,
        "matches": 2000,
        "ground_before_cm": 2.41,
        "ground_after_cm": 1.74,
        "rotation_deg": 0.17,
        "translation_cm": 2.7
      },
      "plane_check": {
        "tilt_deg": 0.428,
        "fore_aft_deg": -0.348,
        "lateral_deg": -0.25,
        "dz_cm": 11.3,
        "pairs": 1,
        "outside_basin": true,
        "applied": false
      }
    },
    "front_left_lidar": {
      "rotvec_deg": [
        -0.2562886679766474,
        -0.1247753896168191,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        3.4453864872975215
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 9,
        "matches": 2838,
        "ground_before_cm": 6.51,
        "ground_after_cm": 2.14,
        "rotation_deg": 0.285,
        "translation_cm": 3.4
      },
      "plane_check": {
        "tilt_deg": 1.111,
        "fore_aft_deg": -0.975,
        "lateral_deg": 0.532,
        "dz_cm": 18.9,
        "pairs": 1,
        "outside_basin": true,
        "applied": false
      }
    },
    "back_lidar": {
      "rotvec_deg": [
        -0.2090300483144752,
        -0.17740191712863818,
        0.00042877918377180236
      ],
      "trans_cm": [
        0.0,
        0.0,
        15.784777070721809
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "warn",
        "reasons": [
          "plane start applied (0.21 deg / +7.8 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage"
        ],
        "iterations": 25,
        "matches": 78457,
        "ground_before_cm": 19.02,
        "ground_after_cm": 12.38,
        "rotation_deg": 0.274,
        "translation_cm": 15.8
      },
      "plane_check": {
        "tilt_deg": 0.214,
        "fore_aft_deg": 0.091,
        "lateral_deg": 0.115,
        "dz_cm": 7.8,
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
        2.8383,
        5.6644,
        2.2681
      ],
      "rpy_deg": [
        -1.433,
        -0.972,
        -1.192
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0126,
        -0.65,
        0.9839
      ],
      "rpy_deg": [
        -0.482,
        1.905,
        -91.244
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0644,
        5.661,
        2.2735
      ],
      "rpy_deg": [
        1.546,
        -1.948,
        179.793
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0242,
        7.7501,
        0.4596
      ],
      "rpy_deg": [
        0.215,
        -1.686,
        89.873
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0755,
      7.8739,
      2.2451
    ],
    "rpy_deg": [
      -102.142,
      1.542,
      -4.24
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1849.47862,
      1848.72893,
      1004.25076,
      503.038474
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
    "lidar": 0.021,
    "lidar_frontend": 0.036,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar +0.0210 + imu/ins clock lag +0.0150 (corr 0.996); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck13/v002_axle/calibration.json
bag=/home/project/data/haibo/wuhu/13/1/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 360..400 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 23.5 cm / P90 86.4 cm (limit 50 cm) -> stationary
front_right_lidar: stationary aggregated points 9326
back_lidar: stationary aggregated points 7602
front_left_lidar: stationary aggregated points 19362
front_lidar: stationary aggregated points 27821
front_right_lidar: plane start: own ground vs reference tilt 0.43 deg (fore-aft -0.35, lateral -0.25), height +11.3 cm over 1 plane pair(s) -> start rot deg (x,y,z)=(-0.348,+0.249,-0.006) z +11.3 cm; ground residual 2.41->3.30 cm (n=1996->1991): not used (does not improve the overlap)
front_right_lidar: yaw/xy solved=False; matches 2002->2000; |res| median 2.40->1.76 cm; ground 2.41->1.74 cm (n=1995); vertical nan->nan cm (n=0); correction rot deg (x,y,z)=(-0.102,+0.136,+0.000) trans cm=(+0.0,+0.0,+2.7); iters 5  [quality: ok]
front_left_lidar: plane start: own ground vs reference tilt 1.11 deg (fore-aft -0.98, lateral +0.53), height +18.9 cm over 1 plane pair(s) -> start rot deg (x,y,z)=(-0.975,-0.531,+0.028) z +18.9 cm; ground residual 6.51->13.61 cm (n=2662->2474): not used (does not improve the overlap)
front_left_lidar: yaw/xy solved=False; matches 2822->2838; |res| median 6.57->2.24 cm; ground 6.51->2.14 cm (n=2686); vertical 6.50->5.38 cm (n=54); correction rot deg (x,y,z)=(-0.256,-0.125,+0.000) trans cm=(+0.0,+0.0,+3.4); iters 9  [quality: ok]
back_lidar: plane start: own ground vs reference tilt 0.21 deg (fore-aft +0.09, lateral +0.12), height +7.8 cm over 40 plane pair(s) -> start rot deg (x,y,z)=(+0.091,-0.115,+0.000) z +7.8 cm; ground residual 19.02->15.95 cm (n=55482->56836): used
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 73649->78457; |res| median 19.14->11.38 cm; ground 19.02->12.38 cm (n=59163); vertical 13.60->12.68 cm (n=7497); correction rot deg (x,y,z)=(-0.209,-0.177,+0.000) trans cm=(+0.0,+0.0,+15.8); iters 25  [quality: warn; plane start applied (0.21 deg / +7.8 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage]
quality: warn  (back_lidar: plane start applied (0.21 deg / +7.8 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage)
written /home/project/MY-LIVO/config/carlibration/wuhu/truck13/v003_ground/calibration.json
```
