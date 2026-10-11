# wuhu/truck29/v072_manual

- stage: manual (手动设置)
- parent: wuhu/truck29/v071_manual
- lineage: wuhu/truck29/v055_import -> wuhu/truck29/v056_axle -> wuhu/truck29/v057_ground -> wuhu/truck29/v058_driven -> wuhu/truck29/v059_camtime -> wuhu/truck29/v060_camlidar -> wuhu/truck29/v061_camloop -> wuhu/truck29/v062_manual -> wuhu/truck29/v063_manual -> wuhu/truck29/v065_driven -> wuhu/truck29/v066_driven -> wuhu/truck29/v067_axle -> wuhu/truck29/v068_ground -> wuhu/truck29/v069_driven -> wuhu/truck29/v071_manual -> wuhu/truck29/v072_manual
- created: 2026-10-11T14:00:29+08:00
- note: back LiDAR 前向 -2.5 cm：守护指标 back LiDAR 沿行驶向偏移 +2.0→+0.7 cm，路侧灯柱 back 点云前向偏移 3–5→1.5–2 cm（原生 LIO 轨迹同样存在，非 RTK 约束所致）；起步灯柱 spread 3.8 不变

## changes vs parent

```
{
  "lidars": {
    "back_lidar": {
      "dt_cm": [
        0.0,
        -2.5,
        0.0
      ],
      "drpy_deg": [
        0.0,
        0.0,
        0.0
      ]
    }
  }
}
```

## metrics

```
{
  "changes": {
    "lidar_translation_delta.transform_matrix_lidar_2": [
      0.0,
      -0.025,
      0.0
    ]
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
        2.7639,
        6.018,
        1.8252
      ],
      "rpy_deg": [
        -0.983,
        -3.018,
        -1.42
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1071,
        -0.1922,
        0.6368
      ],
      "rpy_deg": [
        -1.06,
        0.705,
        -91.303
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0951,
        5.8162,
        1.9615
      ],
      "rpy_deg": [
        -3.467,
        -0.77,
        178.621
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.1094,
        8.0405,
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
      0.0024,
      8.0426,
      1.7517
    ],
    "rpy_deg": [
      -96.512,
      1.62,
      0.291
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
    "lidar": -0.1462,
    "camera": -0.115,
    "rtk_position": -0.015,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "camera_source": "operator set (livo_calib.py set): 时间偏移改回 IMU 硬件对齐对（lidar -0.125 / camera -0.115）：axle 解 -0.1459 含 INS 链路延迟，喂给前端后 21ms 错位打花高角速率段的去畸变与颜色采样（灯柱白度 51.5%→79.3%，牌面着色同步改善）；融合精度在该区间平坦（LIVO 120s xy max 7.12→7.08 cm，达标）。证据链见 docs/mcp-color-regression-time-offsets.md",
    "lidar_frontend": -0.125
  }
}
```
