# wuhu/truck05/v008_import

- stage: import (导入出厂标定)
- parent: None
- lineage: wuhu/truck05/v008_import
- created: 2026-09-24T15:57:13+08:00
- note: 重新导入：相机内参由厂家地面距离曲线反演（原 v001 沿用 truck29 默认内参）

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
    "rig-to-rear-axle alignment of a supplier file is unverified: run the axle stage next",
    "camera intrinsics recovered from the supplier ground-distance curves: fx 1849.4 fy 1849.4 cx 1003.3 cy 502.7 (the truck29 defaults would be off by fx +12.6 % fy -0.3 % cx +32 px cy -32 px)"
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
      1849.37,
      1849.37,
      1003.31,
      502.72
    ],
    "distortion": [
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    ],
    "intrinsics_check": {
      "method": "supplier ground-distance curves (midrange_camera[0].equation_*m)",
      "curves": 10,
      "distances_m": [
        10.0,
        20.0,
        30.0,
        40.0,
        50.0,
        60.0,
        70.0,
        80.0,
        90.0,
        100.0
      ],
      "fits": {
        "free": {
          "intrinsics": [
            1849.32,
            1831.26,
            1004.09,
            499.42
          ],
          "rms_px": 0.05,
          "circle_centre_rear_m": [
            -0.002,
            7.529
          ],
          "ground_z_rear_m": -1.283
        },
        "square": {
          "intrinsics": [
            1849.37,
            1849.37,
            1003.31,
            502.72
          ],
          "rms_px": 0.05,
          "circle_centre_rear_m": [
            0.001,
            7.535
          ],
          "ground_z_rear_m": -1.249
        }
      },
      "model": "square pixels (fx = fy)",
      "rms_px": 0.05,
      "camera_height_m": 3.502,
      "deviation_vs_truck29_defaults": {
        "fx_pct": 12.59,
        "fy_pct": -0.33,
        "cx_px": 32.3,
        "cy_px": -31.5
      }
    },
    "source": "supplier ground-distance curves equation_*m (square pixels (fx = fy), rms 0.05 px)"
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
