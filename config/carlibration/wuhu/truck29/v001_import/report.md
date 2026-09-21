# wuhu/truck29/v001_import

- stage: import
- parent: None
- created: 2026-09-21T17:58:03+08:00
- note: factory manual calibration (2026-05-13), imported

## inputs

- source: /home/project/MY-LIVO/config/carlibration/perception_truck29.json

## metrics

```
{
  "normalisation": [
    "transform_matrix_lidar_1: re-orthonormalised (max deviation 5.3e-05)",
    "transform_matrix_lidar_2: re-orthonormalised (max deviation 5.3e-05)",
    "transform_matrix_lidar_3: re-orthonormalised (max deviation 5.2e-05)",
    "camera transform re-orthonormalised (max deviation 1.4e-06)"
  ]
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
        2.8123,
        5.6996,
        1.8154
      ],
      "rpy_deg": [
        -0.75,
        -3.418,
        -1.241
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        0.0004,
        -0.5225,
        0.6848
      ],
      "rpy_deg": [
        -0.973,
        0.543,
        -91.144
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.052,
        5.4635,
        1.974
      ],
      "rpy_deg": [
        -3.351,
        -1.89,
        178.724
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
      -96.438,
      1.487,
      0.699
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
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)"
  }
}
```
