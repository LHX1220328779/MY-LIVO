# wuhu/truck13/v005_scan

- stage: scan (闭环扫描点（临时）)
- parent: wuhu/truck13/v004_driven
- lineage: wuhu/truck13/v001_import -> wuhu/truck13/v002_axle -> wuhu/truck13/v003_ground -> wuhu/truck13/v004_driven -> wuhu/truck13/v005_scan
- created: 2026-10-26T17:25:27+08:00
- note: rtk-time probe point (temporary)

## changes vs parent

```
{
  "time_offsets": {
    "rtk_position": {
      "from": 0.025,
      "to": 0.003
    }
  }
}
```

## metrics

```
{
  "changes": {
    "time_offsets.rtk_position": 0.003
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
        2.8244,
        5.6877,
        2.2819
      ],
      "rpy_deg": [
        -1.47,
        -0.987,
        -1.201
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0094,
        -0.6493,
        0.8899
      ],
      "rpy_deg": [
        -0.51,
        1.54,
        -91.242
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0863,
        5.6514,
        2.2791
      ],
      "rpy_deg": [
        1.549,
        -1.91,
        179.818
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
    "rtk_position": 0.003,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar +0.0210 + imu/ins clock lag +0.0150 (corr 0.996); template -0.1250 replaced"
  }
}
```
