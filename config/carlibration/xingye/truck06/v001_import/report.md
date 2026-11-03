# xingye/truck06/v001_import

- stage: import (导入出厂标定)
- parent: None
- lineage: xingye/truck06/v001_import
- created: 2026-10-13T10:20:20+08:00
- note: imported from perception_truck06.json

## inputs

- source: /home/project/groundextractor_1/data/calibration/calibration_xingye_2026.10.08_v6.2/calibration/perception_truck06.json

## metrics

```
{
  "normalisation": [
    "transform_matrix_lidar_1: re-orthonormalised (max deviation 2.2e-04)",
    "transform_matrix_lidar_2: re-orthonormalised (max deviation 2.1e-04)",
    "transform_matrix_lidar_3: re-orthonormalised (max deviation 2.6e-04)",
    "transform_matrix_lidar_6: re-orthonormalised (max deviation 1.5e-04)",
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
        1.9723,
        7.6122,
        0.9634
      ],
      "rpy_deg": [
        -2.935,
        1.347,
        14.817
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1634,
        -1.2409,
        0.5367
      ],
      "rpy_deg": [
        -2.126,
        -3.248,
        -90.565
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -1.9563,
        7.6352,
        1.0279
      ],
      "rpy_deg": [
        1.115,
        -2.528,
        166.675
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        0.0144,
        7.6679,
        0.671
      ],
      "rpy_deg": [
        2.48,
        3.432,
        89.492
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0267,
      7.6582,
      1.3533
    ],
    "rpy_deg": [
      -99.304,
      3.079,
      0.265
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1641.16,
      1854.4,
      962.311,
      542.525
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
