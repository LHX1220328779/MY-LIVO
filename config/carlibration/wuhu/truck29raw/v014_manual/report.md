# wuhu/truck29raw/v014_manual

- stage: manual (手动设置)
- parent: wuhu/truck29raw/v009_camloop
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v002_axle -> wuhu/truck29raw/v003_ground -> wuhu/truck29raw/v004_driven -> wuhu/truck29raw/v005_camtime -> wuhu/truck29raw/v006_camlidar -> wuhu/truck29raw/v009_camloop -> wuhu/truck29raw/v014_manual
- created: 2026-09-26T21:28:49+08:00
- note: 同 truck29 v062：前端时间对回归 IMU 硬件对齐值（-0.125/-0.115），修复彩色投影错位；raw 链同数据同相机适用

## changes vs parent

```
{
  "time_offsets": {
    "camera": {
      "from": -0.1211,
      "to": -0.115
    },
    "lidar": {
      "from": -0.1463,
      "to": -0.125
    }
  }
}
```

## metrics

```
{
  "changes": {
    "time_offsets.camera": -0.115,
    "time_offsets.lidar": -0.125
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
        2.756,
        5.9895,
        1.8333
      ],
      "rpy_deg": [
        -1.001,
        -3.026,
        -1.433
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1094,
        -0.311,
        0.6862
      ],
      "rpy_deg": [
        -1.079,
        0.746,
        -91.392
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1269,
        5.7908,
        1.9755
      ],
      "rpy_deg": [
        -3.418,
        -0.801,
        178.584
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.108,
        8.0271,
        0.051
      ],
      "rpy_deg": [
        1.452,
        -1.533,
        89.921
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0037,
      8.0291,
      1.7517
    ],
    "rpy_deg": [
      -96.497,
      1.617,
      0.258
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
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "camera_source": "operator set (livo_calib.py set): 同 truck29 v062：前端时间对回归 IMU 硬件对齐值（-0.125/-0.115），修复彩色投影错位；raw 链同数据同相机适用"
  }
}
```
