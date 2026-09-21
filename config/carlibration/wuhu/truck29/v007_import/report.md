# wuhu/truck29/v007_import

- stage: import
- parent: None
- created: 2026-09-21T18:38:41+08:00
- note: production baseline: stage1+stage2 LiDAR refinement (run99) with the schema-4 camera transform; best LIVO native drift (run101/103)

## inputs

- source: /home/project/MY-LIVO/config/perception_truck29_refined_v2.json

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
        2.8299,
        5.7649,
        1.8091
      ],
      "rpy_deg": [
        -0.803,
        -3.132,
        -1.22
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0042,
        -0.5528,
        0.5995
      ],
      "rpy_deg": [
        -0.987,
        0.761,
        -91.154
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0463,
        5.5356,
        1.9514
      ],
      "rpy_deg": [
        -3.575,
        -0.665,
        178.807
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
      -96.369,
      1.482,
      0.659
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
    "camera": -0.115,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)"
  }
}
```
