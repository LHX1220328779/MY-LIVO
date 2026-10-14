# wuhu/truck29raw/v016_ground

- stage: ground (静止地面精修)
- parent: wuhu/truck29raw/v015_axle
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v015_axle -> wuhu/truck29raw/v016_ground
- created: 2026-10-14T15:59:43+08:00
- note: pipeline: ground

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
        -0.88
      ],
      "drpy_deg": [
        -0.494,
        -0.468,
        -0.002
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -6.44
      ],
      "drpy_deg": [
        0.377,
        -0.741,
        -0.016
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
      "about": "lidar position in rear-axle frame"
    },
    "front_left_lidar": {
      "rotvec_deg": [
        -0.3583969974152925,
        0.7504292306274661,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -6.440558778077513
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "back_lidar": {
      "rotvec_deg": [
        -0.45720165806950597,
        0.5043407377461488,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -0.872736423301515
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    }
  }
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
        0.6919
      ],
      "rpy_deg": [
        -1.649,
        -0.039,
        -91.319
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1488,
        5.6324,
        1.9238
      ],
      "rpy_deg": [
        -2.859,
        -2.813,
        178.533
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
front_right_lidar: yaw/xy solved=True; matches 5510->5464; |res| median 8.52->6.11 cm; ground 9.60->6.68 cm (n=4412); vertical 7.31->4.74 cm (n=693); correction rot deg (x,y,z)=(-0.119,+0.296,+0.105) trans cm=(+3.1,+4.8,-0.8); iters 16
front_left_lidar: yaw/xy solved=False; matches 3004->3246; |res| median 15.53->19.46 cm; ground 15.47->20.29 cm (n=2794); vertical 16.03->13.37 cm (n=258); correction rot deg (x,y,z)=(-0.358,+0.750,+0.000) trans cm=(+0.0,+0.0,-6.4); iters 25
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 17896->17984; |res| median 13.69->11.89 cm; ground 13.23->11.30 cm (n=15498); vertical n=1265; correction rot deg (x,y,z)=(-0.457,+0.504,+0.000) trans cm=(+0.0,+0.0,-0.9); iters 25
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v016_ground/calibration.json
```
