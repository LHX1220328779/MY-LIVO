# wuhu/truck29/v098_import

- stage: import (导入出厂标定)
- parent: None
- lineage: wuhu/truck29/v098_import
- created: 2026-10-12T11:23:15+08:00
- note: factory 2026.01.22 v5.0.6 (rerun from factory)

## inputs

- source: /home/project/MY-LIVO/calibration/calibration_wuhu_2026.01.22_v5.0.6_1/calibration/perception_truck29.json

## metrics

```
{
  "normalisation": [
    "transform_matrix_lidar_2: re-orthonormalised (max deviation 2.5e-06)",
    "transform_matrix_lidar_3: re-orthonormalised (max deviation 2.4e-06)",
    "transform_matrix_lidar_6: re-orthonormalised (max deviation 5.3e-05)",
    "raw supplier export: camera->rear-axle derived from transform_matrix_lidar_6 and tf_lidar_to_camera",
    "rig-to-rear-axle alignment of a supplier file is unverified: run the axle stage next",
    "camera intrinsics: truck29 defaults confirmed by the supplier ground-distance curves"
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
            1642.2,
            1846.33,
            968.83,
            532.82
          ],
          "rms_px": 0.07,
          "circle_centre_rear_m": [
            0.007,
            7.533
          ],
          "ground_z_rear_m": -1.264
        },
        "square": {
          "intrinsics": [
            1642.2,
            1642.2,
            917.71,
            510.63
          ],
          "rms_px": 0.36,
          "circle_centre_rear_m": [
            0.255,
            7.49
          ],
          "ground_z_rear_m": -1.64
        }
      },
      "model": "independent fx / fy",
      "rms_px": 0.07,
      "camera_height_m": 3.019,
      "deviation_vs_truck29_defaults": {
        "fx_pct": -0.02,
        "fy_pct": -0.5,
        "cx_px": -2.2,
        "cy_px": -1.4
      }
    },
    "source": "defaults (wuhu truck29 yaml), confirmed by the supplier ground-distance curves (rms 0.07 px)"
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
