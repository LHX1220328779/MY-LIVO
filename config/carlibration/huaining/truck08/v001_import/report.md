# huaining/truck08/v001_import

- stage: import (导入出厂标定)
- parent: None
- lineage: huaining/truck08/v001_import
- created: 2026-11-11T16:28:47+08:00
- note: imported from perception_truck08.json

## inputs

- source: /home/project/groundextractor_1/data/calibration/calibration_huaining_2026.11.10_v6.3_4/calibration/perception_truck08.json

## metrics

```
{
  "normalisation": [
    "transform_matrix_lidar_1: re-orthonormalised (max deviation 2.1e-04)",
    "transform_matrix_lidar_2: re-orthonormalised (max deviation 1.0e-04)",
    "transform_matrix_lidar_3: re-orthonormalised (max deviation 1.0e-04)",
    "transform_matrix_lidar_6: re-orthonormalised (max deviation 8.3e-05)",
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
        2.0391,
        7.4473,
        1.0684
      ],
      "rpy_deg": [
        0.886,
        -1.473,
        -0.734
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1699,
        -1.0574,
        0.469
      ],
      "rpy_deg": [
        -0.597,
        -0.375,
        -90.416
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -1.8576,
        7.481,
        1.0649
      ],
      "rpy_deg": [
        -3.089,
        -2.147,
        178.033
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        0.083,
        7.6487,
        0.6524
      ],
      "rpy_deg": [
        0.386,
        -1.374,
        90.345
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.1541,
      7.4572,
      2.7261
    ],
    "rpy_deg": [
      -97.739,
      0.639,
      1.132
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1643.61,
      1856.61,
      964.757,
      543.603
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
