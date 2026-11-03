# wuhu/truck27/v001_import

- stage: import (导入出厂标定)
- parent: None
- lineage: wuhu/truck27/v001_import
- created: 2026-10-26T18:17:30+08:00
- note: imported from perception_truck27.json

## inputs

- source: /home/project/groundextractor_1/data/calibration/calibration_wuhu_2026.09.30_6.3/calibration/perception_truck27.json

## metrics

```
{
  "normalisation": [
    "transform_matrix_lidar_1: re-orthonormalised (max deviation 1.1e-04)",
    "transform_matrix_lidar_2: re-orthonormalised (max deviation 6.7e-05)",
    "transform_matrix_lidar_3: re-orthonormalised (max deviation 8.6e-05)",
    "transform_matrix_lidar_6: re-orthonormalised (max deviation 9.4e-05)",
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
        2.7962,
        5.5562,
        1.5271
      ],
      "rpy_deg": [
        0.297,
        0.394,
        0.442
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1311,
        -0.6548,
        0.4281
      ],
      "rpy_deg": [
        0.115,
        2.945,
        -91.333
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.2084,
        5.3621,
        1.5567
      ],
      "rpy_deg": [
        -1.196,
        1.259,
        -179.623
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0288,
        7.6203,
        -0.1579
      ],
      "rpy_deg": [
        -0.464,
        -2.773,
        88.928
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.1328,
      7.5674,
      1.399
    ],
    "rpy_deg": [
      -96.948,
      -1.485,
      -0.427
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1637.070068359375,
      1851.320190429688,
      1030.07353497608,
      496.0184272786501
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
