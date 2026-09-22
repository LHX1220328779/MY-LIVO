# wuhu/truck29/v010_ground

- stage: ground (静止地面精修)
- parent: wuhu/truck29/v007_import
- lineage: wuhu/truck29/v007_import -> wuhu/truck29/v010_ground
- created: 2026-09-22T14:50:47+08:00
- note: Studio v2 验证：LiDAR 流水线（未激活）

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
        1.77
      ],
      "drpy_deg": [
        -0.064,
        0.082,
        0.003
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        9.05
      ],
      "drpy_deg": [
        0.186,
        0.013,
        0.003
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        0.26
      ],
      "drpy_deg": [
        0.053,
        0.092,
        -0.001
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
        -0.061787652758260106,
        0.08309966002559983,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        1.7757440047724524
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "front_left_lidar": {
      "rotvec_deg": [
        -0.05498358832349467,
        -0.09071537045934852,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        0.25823758995354185
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "back_lidar": {
      "rotvec_deg": [
        0.009767132835071034,
        -0.1868638944365991,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        9.050264718778646
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
        2.8299,
        5.7649,
        1.8268
      ],
      "rpy_deg": [
        -0.867,
        -3.05,
        -1.217
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0042,
        -0.5528,
        0.69
      ],
      "rpy_deg": [
        -0.801,
        0.774,
        -91.151
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0463,
        5.5356,
        1.954
      ],
      "rpy_deg": [
        -3.522,
        -0.573,
        178.806
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.033,
        7.793,
        0.051
      ],
      "rpy_deg": [
        1.3,
        -1.67,
        90.16
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0742,
      7.7914,
      1.7519
    ],
    "rpy_deg": [
      -96.369,
      1.482,
      0.659
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
    "lidar": -0.125,
    "camera": -0.115,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v007_import/calibration.json
bag=/home/project/data/haibo/wuhu_livo/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 300..340 (per frame) for back LiDAR
front_right_lidar: stationary aggregated points 12359
back_lidar: stationary aggregated points 2517
front_left_lidar: stationary aggregated points 18513
front_lidar: stationary aggregated points 44072
front_right_lidar: yaw/xy solved=False; matches 3787->3785; |res| median 4.19->4.03 cm; ground 4.18->4.01 cm (n=3700); vertical 3.72->3.24 cm (n=34); correction rot deg (x,y,z)=(-0.062,+0.083,+0.000) trans cm=(+0.0,+0.0,+1.8); iters 15
front_left_lidar: yaw/xy solved=False; matches 4827->4828; |res| median 6.30->5.08 cm; ground 6.30->5.02 cm (n=4685); vertical 6.25->5.65 cm (n=66); correction rot deg (x,y,z)=(-0.055,-0.091,+0.000) trans cm=(+0.0,+0.0,+0.3); iters 25
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 36704->36932; |res| median 9.97->7.93 cm; ground 9.84->7.66 cm (n=31585); vertical n=3965; correction rot deg (x,y,z)=(+0.010,-0.187,+0.000) trans cm=(+0.0,+0.0,+9.1); iters 17
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v010_ground/calibration.json
```
