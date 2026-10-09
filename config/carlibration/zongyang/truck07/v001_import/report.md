# zongyang/truck07/v001_import

- stage: import (导入出厂标定)
- parent: None
- lineage: zongyang/truck07/v001_import
- created: 2026-10-08T16:14:53+08:00
- note: imported from perception_truck07.json

## inputs

- source: /home/project/groundextractor_1/data/calibration/calibration_zongyang_2026.01.23_v5.0.6/calibration/perception_truck07.json

## metrics

```
{
  "normalisation": [
    "transform_matrix_lidar_1: re-orthonormalised (max deviation 6.9e-05)",
    "transform_matrix_lidar_2: re-orthonormalised (max deviation 7.7e-05)",
    "transform_matrix_lidar_3: re-orthonormalised (max deviation 9.5e-05)",
    "transform_matrix_lidar_6: re-orthonormalised (max deviation 7.4e-05)",
    "raw supplier export: camera->rear-axle derived from transform_matrix_lidar_6 and tf_lidar_to_camera",
    "rig-to-rear-axle alignment of a supplier file is unverified: run the axle stage next",
    "camera intrinsics recovered from the supplier ground-distance curves: fx 621.1 fy 900.6 cx 959.0 cy 537.2 (the truck29 defaults would be off by fx -62.2 % fy -51.5 % cx -12 px cy +3 px)"
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
        2.8712,
        5.4821,
        1.7337
      ],
      "rpy_deg": [
        -1.487,
        0.658,
        0.093
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0127,
        -0.5495,
        0.5685
      ],
      "rpy_deg": [
        1.247,
        1.995,
        -89.966
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1209,
        5.2268,
        1.7344
      ],
      "rpy_deg": [
        1.526,
        0.259,
        179.759
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0229,
        7.5277,
        2.5109
      ],
      "rpy_deg": [
        0.166,
        1.915,
        89.662
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0535,
      7.5424,
      2.4087
    ],
    "rpy_deg": [
      -89.564,
      1.286,
      1.41
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      621.1,
      900.65,
      958.95,
      537.17
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
            621.1,
            900.65,
            958.95,
            537.17
          ],
          "rms_px": 0.309,
          "circle_centre_rear_m": [
            -0.01,
            7.536
          ],
          "ground_z_rear_m": -1.263
        },
        "square": {
          "intrinsics": [
            621.33,
            621.33,
            1050.56,
            537.09
          ],
          "rms_px": 2.404,
          "circle_centre_rear_m": [
            -0.942,
            7.544
          ],
          "ground_z_rear_m": -2.916
        }
      },
      "model": "independent fx / fy",
      "rms_px": 0.309,
      "camera_height_m": 3.672,
      "deviation_vs_truck29_defaults": {
        "fx_pct": -62.19,
        "fy_pct": -51.46,
        "cx_px": -12.1,
        "cy_px": 2.9
      }
    },
    "source": "supplier ground-distance curves equation_*m (independent fx / fy, rms 0.309 px)"
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
