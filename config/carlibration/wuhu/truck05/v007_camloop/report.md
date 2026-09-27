# wuhu/truck05/v007_camloop

- stage: camloop (相机外参闭环标定)
- parent: wuhu/truck05/v004_driven
- lineage: wuhu/truck05/v001_import -> wuhu/truck05/v002_axle -> wuhu/truck05/v003_ground -> wuhu/truck05/v004_driven -> wuhu/truck05/v007_camloop
- created: 2026-09-24T14:53:42+08:00
- note: 相机俯仰闭环：Δpitch=-0.035°（尺度斜率 0.105/°，沿用历史值 模型，1 轮，预测原生尺度 1.0000）

## changes vs parent

```
{
  "camera_to_rear": {
    "dt_cm": [
      0.0,
      0.0,
      0.0
    ],
    "drpy_deg": [
      -0.035,
      0.0,
      0.0
    ]
  }
}
```

## metrics

```
{
  "changes": {
    "camera_rot_delta_deg_cam_xyz": [
      -0.0352,
      0.0,
      0.0
    ]
  },
  "pitch_fit": {
    "ok": true,
    "points": [
      {
        "round": "calib_truck05_livo_0924_145109_base",
        "pitch_delta_deg": 0.0,
        "path_length_ratio": 1.0037,
        "duration_s": 57.8
      }
    ],
    "target": 1.0,
    "errors": [],
    "slope_per_deg": 0.105,
    "slope_source": "model/previous run",
    "intercept": 1.0037,
    "rms": 0.0,
    "pitch_delta_deg": -0.0352,
    "clamped": false,
    "predicted_ratio": 1.0,
    "reason": "zero crossing with the known slope"
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
        2.8131,
        5.5135,
        1.9875
      ],
      "rpy_deg": [
        0.623,
        -2.247,
        -1.141
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        0.1268,
        -0.7633,
        0.6302
      ],
      "rpy_deg": [
        -1.616,
        -1.82,
        -90.312
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.029,
        5.4106,
        1.883
      ],
      "rpy_deg": [
        -0.583,
        -1.934,
        179.713
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0968,
        7.5648,
        0.5356
      ],
      "rpy_deg": [
        0.072,
        -5.301,
        89.874
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0022,
      7.5704,
      2.2531
    ],
    "rpy_deg": [
      -100.42,
      0.003,
      -0.449
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
    "lidar": -0.0368,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)"
  }
}
```
