# wuhu/truck29/v050_ground

- stage: ground (静止地面精修)
- parent: wuhu/truck29/v049_axle
- lineage: wuhu/truck29/v037_import -> wuhu/truck29/v049_axle -> wuhu/truck29/v050_ground
- created: 2026-09-24T12:50:17+08:00
- note: mcp turn4: raw auto-detect (exact re-pose)

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- stationary: [2, 20]
- moving: [300, 40]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        1.6
      ],
      "drpy_deg": [
        -0.135,
        0.36,
        0.007
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        2.03
      ],
      "drpy_deg": [
        0.142,
        0.219,
        0.001
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -0.91
      ],
      "drpy_deg": [
        -0.14,
        1.307,
        0.004
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
        -0.1256373800802659,
        0.3638653783197681,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        1.6091803058469154
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "front_left_lidar": {
      "rotvec_deg": [
        0.10452630628623628,
        -1.3105670674410848,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -0.9120594125419231
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "back_lidar": {
      "rotvec_deg": [
        0.21546247067921592,
        -0.14668205673708554,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        2.0326871115484417
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
        2.7268,
        5.9166,
        1.829
      ],
      "rpy_deg": [
        -1.028,
        -2.909,
        -1.471
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1147,
        -0.2961,
        0.7252
      ],
      "rpy_deg": [
        -0.98,
        0.62,
        -91.39
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1379,
        5.706,
        1.9786
      ],
      "rpy_deg": [
        -3.348,
        -0.731,
        178.477
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1141,
        8.0179,
        0.051
      ],
      "rpy_deg": [
        1.452,
        -1.531,
        89.91
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0024,
      8.02,
      1.7517
    ],
    "rpy_deg": [
      -96.507,
      1.636,
      0.41
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
    "lidar": -0.1539,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v049_axle/calibration.json
bag=/home/project/data/haibo/wuhu_livo/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 300..340 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 11.3 cm / P90 35.0 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 12270
back_lidar: stationary aggregated points 2437
front_left_lidar: stationary aggregated points 18499
front_lidar: stationary aggregated points 43584
front_right_lidar: yaw/xy solved=False; matches 3791->3785; |res| median 6.04->4.10 cm; ground 6.04->4.10 cm (n=3723); vertical 7.72->3.98 cm (n=23); correction rot deg (x,y,z)=(-0.126,+0.364,+0.000) trans cm=(+0.0,+0.0,+1.6); iters 25
front_left_lidar: yaw/xy solved=False; matches 4549->4851; |res| median 16.32->5.11 cm; ground 16.69->5.11 cm (n=4745); vertical 3.77->4.75 cm (n=35); correction rot deg (x,y,z)=(+0.105,-1.311,+0.000) trans cm=(+0.0,+0.0,-0.9); iters 25
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 36624->36778; |res| median 8.18->7.97 cm; ground 7.77->7.62 cm (n=31437); vertical n=3983; correction rot deg (x,y,z)=(+0.215,-0.147,+0.000) trans cm=(+0.0,+0.0,+2.0); iters 16
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v050_ground/calibration.json
```
