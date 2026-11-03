# huaining/truck03/v001_import

- stage: import (导入出厂标定)
- parent: None
- lineage: huaining/truck03/v001_import
- created: 2026-10-30T15:55:29+08:00
- note: imported from perception_truck03.json

## inputs

- source: /home/project/groundextractor_1/data/calibration/calibration_huaining_2026.10.30_v6.2/calibration/perception_truck03.json

## metrics

```
{
  "normalisation": [
    "transform_matrix_lidar_1: re-orthonormalised (max deviation 6.3e-04)",
    "transform_matrix_lidar_2: re-orthonormalised (max deviation 3.1e-04)",
    "transform_matrix_lidar_3: re-orthonormalised (max deviation 3.1e-04)",
    "transform_matrix_lidar_6: re-orthonormalised (max deviation 2.6e-04)",
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
        2.029,
        7.408,
        1.0022
      ],
      "rpy_deg": [
        -1.385,
        -2.703,
        -4.195
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.3311,
        -1.0395,
        0.4047
      ],
      "rpy_deg": [
        0.472,
        0.368,
        -91.11
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -1.8328,
        7.4774,
        1.0531
      ],
      "rpy_deg": [
        -2.867,
        -2.73,
        176.987
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        0.0321,
        7.6622,
        0.5194
      ],
      "rpy_deg": [
        -0.24,
        -1.531,
        89.745
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0199,
      7.4678,
      2.6424
    ],
    "rpy_deg": [
      -95.343,
      -0.401,
      -1.471
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1644.2,
      1857.01,
      956.116,
      542.763
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
