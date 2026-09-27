# wuhu/truck05/v001_import

- stage: import (导入出厂标定)
- parent: None
- lineage: wuhu/truck05/v001_import
- created: 2026-09-24T14:23:26+08:00
- note: imported from perception_truck05.json

## inputs

- source: /home/project/groundextractor_1/data/calibration/calibration_wuhu_2026.01.22_v5.0.6_1/calibration/perception_truck05.json

## metrics

```
{
  "normalisation": [
    "transform_matrix_lidar_1: re-orthonormalised (max deviation 1.2e-06)",
    "transform_matrix_lidar_2: re-orthonormalised (max deviation 1.3e-06)",
    "transform_matrix_lidar_3: re-orthonormalised (max deviation 1.7e-06)",
    "transform_matrix_lidar_6: re-orthonormalised (max deviation 6.1e-05)",
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
        2.7784,
        5.4615,
        1.9115
      ],
      "rpy_deg": [
        1.039,
        -2.675,
        -1.422
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        0.0677,
        -0.726,
        0.7578
      ],
      "rpy_deg": [
        -1.814,
        0.178,
        -90.621
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0327,
        5.4276,
        1.9467
      ],
      "rpy_deg": [
        -0.758,
        -2.331,
        179.608
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1114,
        7.6035,
        0.5356
      ],
      "rpy_deg": [
        0.216,
        -5.375,
        89.55
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0081,
      7.6063,
      2.2529
    ],
    "rpy_deg": [
      -100.312,
      0.147,
      -0.759
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
