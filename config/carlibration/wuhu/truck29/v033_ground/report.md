# wuhu/truck29/v033_ground

- stage: ground (静止地面精修)
- parent: wuhu/truck29/v032_camloop
- lineage: wuhu/truck29/v007_import -> wuhu/truck29/v012_ground -> wuhu/truck29/v013_driven -> wuhu/truck29/v014_camtime -> wuhu/truck29/v018_camtime -> wuhu/truck29/v019_camlidar -> wuhu/truck29/v022_camloop -> wuhu/truck29/v023_camtime -> wuhu/truck29/v024_camlidar -> wuhu/truck29/v027_camloop -> wuhu/truck29/v028_camloop -> wuhu/truck29/v029_camtime -> wuhu/truck29/v030_camlidar -> wuhu/truck29/v031_camloop -> wuhu/truck29/v032_camloop -> wuhu/truck29/v033_ground
- created: 2026-09-23T17:26:08+08:00
- note: 效率集成测试 ground

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
        1.76
      ],
      "drpy_deg": [
        -0.065,
        0.08,
        0.004
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        8.34
      ],
      "drpy_deg": [
        0.167,
        -0.021,
        0.002
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        0.55
      ],
      "drpy_deg": [
        0.066,
        0.089,
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
        -0.06279989033418391,
        0.08102088773519586,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        1.7613630321548095
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "front_left_lidar": {
      "rotvec_deg": [
        -0.06810100620629238,
        -0.08768849328270907,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        0.5435033678453096
      ],
      "yaw_xy_solved": false,
      "about": "lidar position in rear-axle frame"
    },
    "back_lidar": {
      "rotvec_deg": [
        -0.024074345309057624,
        -0.16682449694268928,
        0.0
      ],
      "trans_cm": [
        0.0,
        0.0,
        8.33551275752048
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
        2.835,
        5.769,
        1.8269
      ],
      "rpy_deg": [
        -0.867,
        -3.05,
        -1.212
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        0.0076,
        -0.5292,
        0.692
      ],
      "rpy_deg": [
        -0.812,
        0.773,
        -91.172
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.033,
        5.5449,
        1.9558
      ],
      "rpy_deg": [
        -3.516,
        -0.572,
        178.826
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
      -96.347,
      1.471,
      0.557
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
    "source": "import (verify with camera-time / LIO evaluation)",
    "camera_source": "optical flow vs gyro on /home/project/data/haibo/wuhu_livo/ros2bag (corr 0.92)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v032_camloop/calibration.json
bag=/home/project/data/haibo/wuhu_livo/ros2bag
reference=front_lidar
stationary frames 2..22 (aggregated) for side LiDARs; moving frames 300..340 (per frame) for back LiDAR
front_right_lidar: stationary aggregated points 12322
back_lidar: stationary aggregated points 2503
front_left_lidar: stationary aggregated points 18408
front_lidar: stationary aggregated points 44072
front_right_lidar: yaw/xy solved=False; matches 3793->3789; |res| median 4.22->4.06 cm; ground 4.21->4.04 cm (n=3705); vertical 5.41->3.75 cm (n=35); correction rot deg (x,y,z)=(-0.063,+0.081,+0.000) trans cm=(+0.0,+0.0,+1.8); iters 14
front_left_lidar: yaw/xy solved=False; matches 4830->4828; |res| median 6.33->5.05 cm; ground 6.34->5.00 cm (n=4682); vertical 6.23->5.86 cm (n=69); correction rot deg (x,y,z)=(-0.068,-0.088,+0.000) trans cm=(+0.0,+0.0,+0.5); iters 25
back_lidar: (moving, per-frame vs refined sides) yaw/xy solved=False; matches 36782->37045; |res| median 10.15->7.94 cm; ground 10.02->7.66 cm (n=31770); vertical n=3906; correction rot deg (x,y,z)=(-0.024,-0.167,+0.000) trans cm=(+0.0,+0.0,+8.3); iters 16
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v033_ground/calibration.json
```
