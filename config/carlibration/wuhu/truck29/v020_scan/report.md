# wuhu/truck29/v020_scan

- stage: scan (闭环扫描点（临时）)
- parent: wuhu/truck29/v019_camlidar
- lineage: wuhu/truck29/v007_import -> wuhu/truck29/v012_ground -> wuhu/truck29/v013_driven -> wuhu/truck29/v014_camtime -> wuhu/truck29/v018_camtime -> wuhu/truck29/v019_camlidar -> wuhu/truck29/v020_scan
- created: 2026-09-22T19:40:58+08:00
- note: 闭环俯仰扫描点 Δpitch=+0.05°（流水线临时版本，勿激活）

## changes vs parent

```
{
  "camera_to_rear": {
    "dt_cm": [
      0.0,
      0.0,
      0.0
    ],
    "drpy_deg": [
      0.05,
      0.0,
      0.0
    ]
  }
}
```

## metrics

```
{
  "changes": {
    "camera_rot_delta_deg_cam_xyz": [
      0.05,
      0.0,
      0.0
    ]
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
        1.8093
      ],
      "rpy_deg": [
        -0.802,
        -3.13,
        -1.216
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        0.0076,
        -0.5292,
        0.6086
      ],
      "rpy_deg": [
        -0.979,
        0.794,
        -91.174
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.033,
        5.5449,
        1.9503
      ],
      "rpy_deg": [
        -3.582,
        -0.661,
        178.827
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
      -96.322,
      1.472,
      0.563
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
