# huaining/truck03/v003_ground

- stage: ground (静止地面精修)
- parent: huaining/truck03/v002_axle
- lineage: huaining/truck03/v001_import -> huaining/truck03/v002_axle -> huaining/truck03/v003_ground
- created: 2026-10-30T17:06:50+08:00
- note: pipeline: ground

## inputs

- bag: /home/project/data/haibo/huaining/03/0729/ros2bag
- dataset: {'path': '/home/project/data/haibo/huaining/03/0729/ros2bag', 'kind': 'raw_split', 'segments': 12, 'duration_s': 359.99, 'has_camera': False}
- stationary: [62, 20]
- moving: [3350, 40]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -3.16,
        2.51,
        -2.62
      ],
      "drpy_deg": [
        0.102,
        0.197,
        0.01
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -4.67
      ],
      "drpy_deg": [
        -0.472,
        0.192,
        0.0
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -0.6,
        2.92,
        -3.39
      ],
      "drpy_deg": [
        0.05,
        0.088,
        0.01
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
        0.11310304803525617,
        0.1912514069899519,
        0.014480608642623556
      ],
      "trans_cm": [
        -3.1591986859617034,
        2.5134523271551688,
        -2.619082305083534
      ],
      "yaw_xy_solved": true,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 16,
        "matches": 6263,
        "ground_before_cm": 4.65,
        "ground_after_cm": 3.66,
        "rotation_deg": 0.223,
        "translation_cm": 4.8
      },
      "plane_check": {
        "tilt_deg": 0.924,
        "fore_aft_deg": 0.138,
        "lateral_deg": -0.914,
        "dz_cm": 2.1,
        "pairs": 1,
        "outside_basin": true,
        "applied": false
      }
    },
    "front_left_lidar": {
      "rotvec_deg": [
        -0.05304375828914993,
        -0.0860029711566217,
        0.01218302383900022
      ],
      "trans_cm": [
        -0.593223584857294,
        2.9133440626379583,
        -3.3888607884111064
      ],
      "yaw_xy_solved": true,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 19,
        "matches": 6308,
        "ground_before_cm": 7.37,
        "ground_after_cm": 5.46,
        "rotation_deg": 0.102,
        "translation_cm": 4.5
      },
      "plane_check": {
        "tilt_deg": 0.55,
        "fore_aft_deg": -0.136,
        "lateral_deg": -0.533,
        "dz_cm": -0.5,
        "pairs": 1,
        "outside_basin": true,
        "applied": false
      }
    },
    "back_lidar": {
      "rotvec_deg": [
        0.1942529299575298,
        0.4712936197684819,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -4.663451082220168
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame",
      "chosen": "fine",
      "quality": {
        "level": "ok",
        "reasons": [],
        "iterations": 25,
        "matches": 40607,
        "ground_before_cm": 10.57,
        "ground_after_cm": 8.25,
        "rotation_deg": 0.51,
        "translation_cm": 4.7
      },
      "plane_check": {
        "tilt_deg": 0.802,
        "fore_aft_deg": 0.541,
        "lateral_deg": -0.536,
        "dz_cm": -12.9,
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
        1.9481,
        6.6758,
        0.9654
      ],
      "rpy_deg": [
        -1.811,
        -2.175,
        -3.283
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.2541,
        -1.8369,
        0.4365
      ],
      "rpy_deg": [
        -0.359,
        0.052,
        -90.232
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -1.8883,
        6.6906,
        1.0328
      ],
      "rpy_deg": [
        -2.296,
        -2.983,
        177.849
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0239,
        6.87,
        0.5194
      ],
      "rpy_deg": [
        0.126,
        -1.028,
        90.616
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0596,
      6.6937,
      2.6443
    ],
    "rpy_deg": [
      -95.853,
      -0.046,
      -0.59
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1644.2,
      1857.01,
      956.116,
      542.763
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
    "lidar": -0.0016,
    "lidar_frontend": 0.0034,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0016 + imu/ins clock lag +0.0050 (corr 0.999); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/huaining/truck03/v002_axle/calibration.json
bag=/home/project/data/haibo/huaining/03/0729/ros2bag
reference=front_lidar
stationary frames 62..82 (aggregated) for side LiDARs; moving frames 3350..3390 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 6.6 cm / P90 19.9 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 17150
back_lidar: stationary aggregated points 5885
front_left_lidar: stationary aggregated points 23178
front_lidar: stationary aggregated points 51347
front_right_lidar: plane start: own ground vs reference tilt 0.92 deg (fore-aft +0.14, lateral -0.91), height +2.1 cm over 1 plane pair(s) -> start rot deg (x,y,z)=(+0.138,+0.913,-0.007) z +2.1 cm; ground residual 4.65->10.79 cm (n=4834->4874): not used (does not improve the overlap)
front_right_lidar: yaw/xy solved=True; matches 6265->6263; |res| median 4.66->3.55 cm; ground 4.65->3.66 cm (n=4858); vertical 4.74->2.76 cm (n=990); correction rot deg (x,y,z)=(+0.113,+0.191,+0.014) trans cm=(-3.2,+2.5,-2.6); iters 16  [quality: ok]
front_left_lidar: plane start: own ground vs reference tilt 0.55 deg (fore-aft -0.14, lateral -0.53), height -0.5 cm over 1 plane pair(s) -> start rot deg (x,y,z)=(-0.136,+0.533,+0.001) z -0.5 cm; ground residual 7.37->11.59 cm (n=4875->4773): not used (does not improve the overlap)
front_left_lidar: yaw/xy solved=True; matches 6300->6308; |res| median 6.22->4.40 cm; ground 7.37->5.46 cm (n=4870); vertical 3.26->2.54 cm (n=1066); correction rot deg (x,y,z)=(-0.053,-0.086,+0.012) trans cm=(-0.6,+2.9,-3.4); iters 19  [quality: ok]
back_lidar: plane start: own ground vs reference tilt 0.80 deg (fore-aft +0.54, lateral -0.54), height -12.9 cm over 40 plane pair(s) -> start rot deg (x,y,z)=(+0.541,+0.536,-0.000) z -12.9 cm; ground residual 10.57->13.98 cm (n=33217->31039): not used (does not improve the overlap)
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 39628->40607; |res| median 12.59->10.13 cm; ground 10.57->8.25 cm (n=33585); vertical 29.56->29.14 cm (n=6327); correction rot deg (x,y,z)=(+0.194,+0.471,+0.000) trans cm=(+0.0,+0.0,-4.7); iters 25  [quality: ok]
quality: ok
written /home/project/MY-LIVO/config/carlibration/huaining/truck03/v003_ground/calibration.json
```
