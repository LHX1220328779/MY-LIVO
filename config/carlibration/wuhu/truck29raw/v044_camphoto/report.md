# wuhu/truck29raw/v044_camphoto

- stage: camphoto (相机光度标定（内参复核 + 外参/时间精修 + 暗角）)
- parent: wuhu/truck29raw/v043_camphoto
- lineage: wuhu/truck29raw/v001_import -> wuhu/truck29raw/v034_axle -> wuhu/truck29raw/v035_ground -> wuhu/truck29raw/v036_driven -> wuhu/truck29raw/v038_rtktime -> wuhu/truck29raw/v039_gcons -> wuhu/truck29raw/v040_camtime -> wuhu/truck29raw/v041_camlidar -> wuhu/truck29raw/v042_camloop -> wuhu/truck29raw/v043_camphoto -> wuhu/truck29raw/v044_camphoto
- created: 2026-10-21T19:40:05+08:00
- note: 相机光度标定：外参 / 时间 / 内参无显著改变，暗角（平场）已标定

## inputs

- bag: /home/project/data/haibo/wuhu/29/20261013/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/29/20261013/ros2bag', 'kind': 'raw_split', 'segments': 58, 'duration_s': 869.98, 'has_camera': True}
- run: results/mcp45_livo_final3/Log/fusion/1792574780682946281

## metrics

```
{
  "camera_photometric": {
    "method": "multi-view photo-consistency of the round's returns (LCN cross-view variance)",
    "frames": 8636,
    "views": 260,
    "returns": 50000,
    "cost_zero": [
      0.18166,
      0.18223
    ],
    "halves": [
      {
        "rot_deg": [
          -0.0232,
          -0.0841,
          0.0709
        ],
        "dt_s": -0.00388,
        "train_gain": 0.0167,
        "test_gain": 0.0055,
        "intrinsics": {
          "fx_scale": 1.00057,
          "fy_scale": 0.9998,
          "k1": -0.0081,
          "test_gain": 0.0056
        }
      },
      {
        "rot_deg": [
          -0.0128,
          -0.0187,
          0.0514
        ],
        "dt_s": -0.00355,
        "train_gain": 0.0126,
        "test_gain": 0.0087,
        "intrinsics": {
          "fx_scale": 0.99945,
          "fy_scale": 1.0011,
          "k1": -0.00034,
          "test_gain": 0.0097
        }
      }
    ],
    "gate": {
      "min_test_gain": 0.05,
      "halves_agree": true,
      "gain_ok": false,
      "intrinsics_extra_gain": [
        0.0001,
        0.001
      ],
      "intrinsics_ok": false
    },
    "flat_field": {
      "halves": [
        {
          "coefficients": [
            0.09524,
            -0.46355,
            -0.12561,
            -0.08977,
            -0.01309,
            -0.03573,
            -0.05498,
            0.09667
          ],
          "test_gain": 0.3057
        },
        {
          "coefficients": [
            0.1053,
            -0.48135,
            -0.11315,
            -0.08768,
            -0.04271,
            -0.01679,
            -0.05679,
            0.09188
          ],
          "test_gain": 0.3066
        }
      ],
      "gain_map_spread": 0.0151,
      "y_range": [
        -0.809,
        0.829
      ],
      "min_test_gain": 0.05,
      "max_spread": 0.03,
      "samples": 459758,
      "applied": true,
      "coefficients": [
        0.099224,
        -0.471582,
        -0.119058,
        -0.088122,
        -0.02713,
        -0.025911,
        -0.057104,
        0.093377
      ]
    },
    "applied": {
      "rot_delta_deg_cam_xyz": [
        0.0,
        0.0,
        0.0
      ],
      "camera_time_offset_s": -0.0122,
      "dt_s": 0.0,
      "intrinsics": false,
      "cost_gain_all": 0.0,
      "rotation_time": false,
      "flat_field": true
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
        2.7755,
        5.9149,
        1.8312
      ],
      "rpy_deg": [
        -1.031,
        -3.057,
        -1.219
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0611,
        -0.2984,
        0.7111
      ],
      "rpy_deg": [
        -1.047,
        0.481,
        -91.365
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0915,
        5.6621,
        2.0104
      ],
      "rpy_deg": [
        3.243,
        -4.537,
        178.24
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.106,
        7.9492,
        0.051
      ],
      "rpy_deg": [
        1.485,
        -1.56,
        89.984
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0067,
      7.9506,
      1.7516
    ],
    "rpy_deg": [
      -96.787,
      1.676,
      0.336
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
    "source": "defaults (wuhu truck29 yaml)",
    "flat_field": {
      "model": "poly8",
      "coefficients": [
        0.099224,
        -0.471582,
        -0.119058,
        -0.088122,
        -0.02713,
        -0.025911,
        -0.057104,
        0.093377
      ],
      "y_range": [
        -0.8087,
        0.829
      ],
      "source": "camera-photo: multi-view grey consistency (held-out 30.6 / 30.7 %)"
    }
  },
  "time_offsets": {
    "lidar": -0.0361,
    "camera": -0.0122,
    "rtk_position": 0.0151,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "lidar_frontend": -0.0311,
    "lidar_frontend_source": "axle: lidar -0.0361 + imu/ins clock lag +0.0050 (corr 0.998); template -0.1250 replaced",
    "rtk_position_source": "rtk-time: map consistency, secant of both rounds (measured gain 0.84); effective +10.1 ms (se 2.0 ms)",
    "camera_source": "camera-photo: multi-view photo-consistency, +0.0000 -12.2 ms (held-out gains 26.5 / 27.9 %)"
  }
}
```

## stage report

```
collection: 8636 frames with an image, 260 views, 50000 returns (>= 3 views), 459760 samples
half 0: rotation+time rx -0.023 ry -0.084 rz +0.071 deg, dt -3.9 ms: train +1.7 %, held-out +0.5 %; with intrinsics (fx +0.06 %, fy -0.02 %, k1 -0.0081) held-out +0.6 %
half 1: rotation+time rx -0.013 ry -0.019 rz +0.051 deg, dt -3.6 ms: train +1.3 %, held-out +0.9 %; with intrinsics (fx -0.05 %, fy +0.11 %, k1 -0.0003) held-out +1.0 %
rotation/time: held-out gain below 5 % (rotation spread [0.01, 0.065, 0.019] deg, dt spread 0.3 ms): kept
intrinsics: not applied
flat field half 0: held-out cross-view variance +30.6 %, darkest position -42 % of the principal point
flat field half 1: held-out cross-view variance +30.7 %, darkest position -42 % of the principal point
flat field: applied (held-out 30.6 / 30.7 %, half spread 1.5 %); image positions -42 .. +7 % of the principal point
applied: flat field only (camera rotation / time unchanged) (185 s)
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29raw/v044_camphoto/calibration.json
```
