# wuhu/truck28/v017_precheck

- stage: precheck (采纳建图自检修正)
- parent: wuhu/truck28/v004_driven
- lineage: wuhu/truck28/v001_import -> wuhu/truck28/v002_axle -> wuhu/truck28/v003_ground -> wuhu/truck28/v004_driven -> wuhu/truck28/v017_precheck
- created: 2026-10-11T16:53:56+08:00
- note: adopted precheck of wuhu_28_lio_1_test

## inputs

- run_dir: /home/project/MY-LIVO/results/wuhu_28_lio_1_test
- report: /home/project/MY-LIVO/results/wuhu_28_lio_1_test/Log/extrinsic_precheck.json

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        5.63
      ],
      "drpy_deg": [
        -0.143,
        -0.077,
        -0.006
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        11.74
      ],
      "drpy_deg": [
        0.197,
        0.128,
        -0.001
      ]
    }
  }
}
```

## metrics

```
{
  "applied": {
    "/front_right_lidar": {
      "rot_x_deg": -0.1430032504,
      "rot_y_deg": -0.07623438816,
      "dz_cm": 5.631397767,
      "median_before_cm": 2.104966407,
      "median_after_cm": 1.947870966
    },
    "/front_left_lidar": {
      "rot_x_deg": -0.1999146379,
      "rot_y_deg": -0.1240162875,
      "dz_cm": 11.73881333,
      "median_before_cm": 7.204621363,
      "median_after_cm": 1.89765922
    }
  }
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
        2.7415,
        5.8007,
        1.5052
      ],
      "rpy_deg": [
        0.084,
        2.36,
        -0.263
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1764,
        -0.3512,
        0.3407
      ],
      "rpy_deg": [
        -1.774,
        2.55,
        -90.7
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1134,
        5.5768,
        1.675
      ],
      "rpy_deg": [
        -0.85,
        -0.1,
        178.583
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1426,
        7.8507,
        -0.1568
      ],
      "rpy_deg": [
        2.071,
        -3.4,
        89.82
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.1951,
      7.7526,
      1.5925
    ],
    "rpy_deg": [
      -100.421,
      0.211,
      0.764
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1637.05,
      1851.4,
      1029.33,
      495.55
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
            1637.05,
            1851.4,
            1029.33,
            495.55
          ],
          "rms_px": 0.014,
          "circle_centre_rear_m": [
            0.0,
            7.38
          ],
          "ground_z_rear_m": -1.22
        },
        "square": {
          "intrinsics": [
            1639.36,
            1639.36,
            864.27,
            446.63
          ],
          "rms_px": 1.072,
          "circle_centre_rear_m": [
            0.806,
            7.314
          ],
          "ground_z_rear_m": -1.588
        }
      },
      "model": "independent fx / fy",
      "rms_px": 0.014,
      "camera_height_m": 2.827,
      "deviation_vs_truck29_defaults": {
        "fx_pct": -0.34,
        "fy_pct": -0.22,
        "cx_px": 58.3,
        "cy_px": -38.7
      }
    },
    "source": "supplier ground-distance curves equation_*m (independent fx / fy, rms 0.014 px)"
  },
  "time_offsets": {
    "lidar": -0.0151,
    "lidar_frontend": -0.0026,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0151 + imu/ins clock lag +0.0125 (corr 0.998); template -0.1250 replaced"
  }
}
```
