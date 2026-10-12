# wuhu/truck29/v102_scan

- stage: scan (闭环扫描点（临时）)
- parent: wuhu/truck29/v101_driven
- lineage: wuhu/truck29/v098_import -> wuhu/truck29/v099_axle -> wuhu/truck29/v100_ground -> wuhu/truck29/v101_driven -> wuhu/truck29/v102_scan
- created: 2026-10-12T11:33:20+08:00
- note: rtk-time probe point (temporary)

## changes vs parent

```
{
  "time_offsets": {
    "rtk_position": {
      "from": 0.025,
      "to": -0.0193
    }
  }
}
```

## metrics

```
{
  "changes": {
    "time_offsets.rtk_position": -0.0193
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
        2.7465,
        6.0118,
        1.8332
      ],
      "rpy_deg": [
        -1.005,
        -3.03,
        -1.452
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1264,
        -0.1699,
        0.6445
      ],
      "rpy_deg": [
        -1.042,
        0.671,
        -91.311
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1089,
        5.808,
        1.9671
      ],
      "rpy_deg": [
        -3.448,
        -0.773,
        178.58
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1088,
        8.0401,
        0.051
      ],
      "rpy_deg": [
        1.452,
        -1.532,
        89.924
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0029,
      8.0422,
      1.7517
    ],
    "rpy_deg": [
      -96.506,
      1.636,
      0.424
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
    "lidar": -0.1459,
    "lidar_frontend": -0.125,
    "camera": 0.0,
    "rtk_position": -0.0193,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)"
  }
}
```
