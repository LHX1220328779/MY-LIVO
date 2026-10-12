# wuhu/truck29/v091_ground

- stage: ground (静止地面精修)
- parent: wuhu/truck29/v090_axle
- lineage: wuhu/truck29/v001_import -> wuhu/truck29/v090_axle -> wuhu/truck29/v091_ground
- created: 2026-10-12T09:59:05+08:00
- note: mcp: axle dof + frontend fix check (v001 lineage)

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
        1.33
      ],
      "drpy_deg": [
        -0.123,
        0.357,
        0.007
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        1.93
      ],
      "drpy_deg": [
        0.13,
        0.228,
        0.002
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        -0.91
      ],
      "drpy_deg": [
        -0.141,
        1.307,
        0.003
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
        -0.11411954070860465,
        0.36020524487822614,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        1.3293588897352007
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "front_left_lidar": {
      "rotvec_deg": [
        0.10673838038661784,
        -1.3109256433013996,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        -0.9022167170256057
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "back_lidar": {
      "rotvec_deg": [
        0.22465376793455455,
        -0.13477833051741434,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        1.9331865658505758
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
        2.7326,
        5.9396,
        1.8262
      ],
      "rpy_deg": [
        -1.016,
        -2.912,
        -1.458
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1075,
        -0.2737,
        0.7242
      ],
      "rpy_deg": [
        -0.992,
        0.629,
        -91.376
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1322,
        5.7277,
        1.9786
      ],
      "rpy_deg": [
        -3.35,
        -0.731,
        178.49
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1089,
        8.0403,
        0.051
      ],
      "rpy_deg": [
        1.452,
        -1.532,
        89.924
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0028,
      8.0424,
      1.7516
    ],
    "rpy_deg": [
      -96.575,
      1.64,
      0.463
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
    "lidar": -0.146,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "lidar_frontend": -0.125,
    "lidar_frontend_source": "axle: import default (parent version had no lidar_frontend)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v090_axle/calibration.json
bag=/home/project/data/haibo/wuhu_livo/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 300..340 (per frame) for back LiDAR
stationary check (front_lidar, first vs last frame of 20): scan displacement median 11.3 cm / P90 35.0 cm (limit 30 cm) -> stationary
front_right_lidar: stationary aggregated points 12229
back_lidar: stationary aggregated points 2425
front_left_lidar: stationary aggregated points 18515
front_lidar: stationary aggregated points 43541
front_right_lidar: yaw/xy solved=False; matches 3775->3750; |res| median 6.04->4.02 cm; ground 6.06->4.02 cm (n=3669); vertical 4.70->3.12 cm (n=38); correction rot deg (x,y,z)=(-0.114,+0.360,+0.000) trans cm=(+0.0,+0.0,+1.3); iters 16
front_left_lidar: yaw/xy solved=False; matches 4553->4872; |res| median 16.54->5.11 cm; ground 16.73->5.11 cm (n=4756); vertical 6.61->5.09 cm (n=53); correction rot deg (x,y,z)=(+0.107,-1.311,+0.000) trans cm=(+0.0,+0.0,-0.9); iters 25
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 36623->36770; |res| median 8.23->7.98 cm; ground 7.84->7.62 cm (n=31433); vertical n=4007; correction rot deg (x,y,z)=(+0.225,-0.135,+0.000) trans cm=(+0.0,+0.0,+1.9); iters 18
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v091_ground/calibration.json
```
