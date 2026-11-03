# xingye/truck13/v001_import

- stage: import (导入出厂标定)
- parent: None
- lineage: xingye/truck13/v001_import
- created: 2026-10-28T10:43:51+08:00
- note: imported from perception_truck13.json

## inputs

- source: /home/project/groundextractor_1/data/calibration/calibration_xingye_2026.10.21_v6.3/calibration/perception_truck13.json

## metrics

```
{
  "normalisation": [
    "transform_matrix_lidar_1: re-orthonormalised (max deviation 1.6e-04)",
    "transform_matrix_lidar_2: re-orthonormalised (max deviation 2.3e-04)",
    "transform_matrix_lidar_3: re-orthonormalised (max deviation 1.5e-04)",
    "transform_matrix_lidar_6: re-orthonormalised (max deviation 1.3e-04)",
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
        2.0153,
        6.8906,
        1.0726
      ],
      "rpy_deg": [
        -1.373,
        -0.937,
        13.362
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.2104,
        -1.9643,
        0.4185
      ],
      "rpy_deg": [
        0.38,
        -3.11,
        -90.801
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -1.8436,
        6.8996,
        1.0813
      ],
      "rpy_deg": [
        0.381,
        -0.118,
        167.505
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0026,
        6.9647,
        0.6541
      ],
      "rpy_deg": [
        -0.097,
        1.736,
        89.835
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0216,
      6.9853,
      1.4571
    ],
    "rpy_deg": [
      -97.884,
      0.846,
      -1.33
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1642.15,
      1855.56,
      963.247,
      544.29
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
    "lidar": -0.125,
    "lidar_frontend": -0.125,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "import default (unmeasured; run the axle stage to auto-measure)",
    "source": "import (verify with camera-time / LIO evaluation)"
  }
}
```
