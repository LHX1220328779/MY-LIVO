# wuhu/truck29-0930/v001_import

- stage: import (导入出厂标定)
- parent: None
- lineage: wuhu/truck29-0930/v001_import
- created: 2026-10-14T21:20:45+08:00
- note: imported from perception_truck29.json

## inputs

- source: /home/project/groundextractor_1/data/calibration/calibration_wuhu_2026.09.30_6.3/calibration/perception_truck29.json

## metrics

```
{
  "normalisation": [
    "transform_matrix_lidar_1: re-orthonormalised (max deviation 1.4e-06)",
    "transform_matrix_lidar_2: re-orthonormalised (max deviation 2.5e-06)",
    "transform_matrix_lidar_3: re-orthonormalised (max deviation 2.9e-06)",
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
        2.8518,
        5.4568,
        1.9237
      ],
      "rpy_deg": [
        -0.425,
        -5.459,
        -2.496
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
        -3.0096,
        5.2989,
        1.8531
      ],
      "rpy_deg": [
        2.719,
        -1.911,
        177.164
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
