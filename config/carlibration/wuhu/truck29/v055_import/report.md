# wuhu/truck29/v055_import

- stage: import (导入出厂标定)
- parent: None
- lineage: wuhu/truck29/v055_import
- created: 2026-09-24T13:35:06+08:00
- note: test2

## inputs

- source: /home/project/MY-LIVO/config/carlibration/wuhu/truck29/raw/perception_truck29.json

## metrics

```
{
  "normalisation": [
    "transform_matrix_lidar_2: re-orthonormalised (max deviation 2.5e-06)",
    "transform_matrix_lidar_3: re-orthonormalised (max deviation 2.4e-06)",
    "transform_matrix_lidar_6: re-orthonormalised (max deviation 5.3e-05)",
    "raw supplier export: camera->rear-axle derived from transform_matrix_lidar_6 and tf_lidar_to_camera",
    "rig-to-rear-axle alignment of a supplier file is unverified: run the axle stage next"
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
        2.8317,
        5.4452,
        1.9131
      ],
      "rpy_deg": [
        -0.289,
        -5.708,
        -2.555
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0696,
        -0.7051,
        0.6261
      ],
      "rpy_deg": [
        1.318,
        0.998,
        -92.391
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0379,
        5.3396,
        1.8347
      ],
      "rpy_deg": [
        -3.812,
        0.4,
        177.452
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        0.1069,
        7.6128,
        0.051
      ],
      "rpy_deg": [
        -1.002,
        -2.072,
        88.958
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.1455,
      7.5982,
      1.7548
    ],
    "rpy_deg": [
      -95.986,
      -0.821,
      -0.616
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
    "lidar_source": "import default (unmeasured; run the axle stage to auto-measure)",
    "source": "import (verify with camera-time / LIO evaluation)"
  }
}
```
