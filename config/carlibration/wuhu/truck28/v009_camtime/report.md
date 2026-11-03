# wuhu/truck28/v009_camtime

- stage: camtime (相机时间偏移)
- parent: wuhu/truck28/v004_driven
- lineage: wuhu/truck28/v001_import -> wuhu/truck28/v002_axle -> wuhu/truck28/v003_ground -> wuhu/truck28/v004_driven -> wuhu/truck28/v009_camtime
- created: 2026-10-11T15:47:22+08:00
- note: midrange camera data (replaced fisheye images)

## inputs

- bag: /home/project/data/haibo/wuhu/28/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/28/ros2bag', 'kind': 'raw_split', 'segments': 82, 'duration_s': 1229.98, 'has_camera': True}
- duration: 60.0
- start_s: 1105.0

## changes vs parent

```
{
  "time_offsets": {
    "camera": {
      "from": 0.0,
      "to": 0.0042
    }
  }
}
```

## metrics

```
{
  "camera_offset_s": 0.0042,
  "estimated_offset_s": 0.0042,
  "applied": true,
  "quality": {
    "ok": true,
    "problems": [],
    "segment_spread_ms": 8.51721794355708,
    "gain": 1.215
  },
  "yaw": {
    "lag_s": 0.004176248646577209,
    "corr": 0.966085692186772,
    "gain": 1.214623907894431,
    "n": 1799,
    "sse_ratio_zero_lag": 1.0091689022471206
  },
  "pitch": {
    "lag_s": -0.00506805591316706,
    "corr": 0.6763675143581158,
    "gain": 0.9880849433055408,
    "n": 1799,
    "sse_ratio_zero_lag": 1.004649375538969
  },
  "segments": [
    {
      "start_s": 0,
      "lag_s": 0.0027182819571415625,
      "corr": 0.8765066271048944
    },
    {
      "start_s": 30,
      "lag_s": 0.011235499900698644,
      "corr": 0.9761264023545938
    }
  ],
  "implied_fx_from_gain": 1988.4000684185783
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
        1.4489
      ],
      "rpy_deg": [
        0.227,
        2.437,
        -0.257
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
        1.5576
      ],
      "rpy_deg": [
        -1.047,
        -0.228,
        178.584
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
    "camera": 0.0042,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0151 + imu/ins clock lag +0.0125 (corr 0.998); template -0.1250 replaced",
    "camera_source": "optical flow vs gyro on /home/project/data/haibo/wuhu/28/ros2bag (corr 0.97)"
  }
}
```
