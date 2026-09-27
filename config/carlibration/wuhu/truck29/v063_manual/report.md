# wuhu/truck29/v063_manual

- stage: manual (手动设置)
- parent: wuhu/truck29/v062_manual
- lineage: wuhu/truck29/v055_import -> wuhu/truck29/v056_axle -> wuhu/truck29/v057_ground -> wuhu/truck29/v058_driven -> wuhu/truck29/v059_camtime -> wuhu/truck29/v060_camlidar -> wuhu/truck29/v061_camloop -> wuhu/truck29/v062_manual -> wuhu/truck29/v063_manual
- created: 2026-09-27T15:11:03+08:00
- note: 时间语义分离：lidar=-0.1459(axle INS 对齐,融合参考) + lidar_frontend=-0.125(IMU 硬件,前端去畸变/图像配对)。LIVMapper 自动以差值平移 rtk.time_offset_sec，parameters.txt 记录生效值，评估自动一致

## changes vs parent

```
{
  "time_offsets": {
    "lidar": {
      "from": -0.125,
      "to": -0.1459
    },
    "lidar_frontend": {
      "from": null,
      "to": -0.125
    }
  }
}
```

## metrics

```
{
  "changes": {
    "time_offsets.lidar": -0.1459,
    "time_offsets.lidar_frontend": -0.125
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
        2.7591,
        6.0014,
        1.8338
      ],
      "rpy_deg": [
        -1.005,
        -3.032,
        -1.426
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.095,
        -0.2998,
        0.6874
      ],
      "rpy_deg": [
        -1.084,
        0.749,
        -91.412
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1289,
        5.8056,
        1.9748
      ],
      "rpy_deg": [
        -3.413,
        -0.811,
        178.588
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
    "lidar": -0.1459,
    "camera": -0.115,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "camera_source": "operator set (livo_calib.py set): 时间偏移改回 IMU 硬件对齐对（lidar -0.125 / camera -0.115）：axle 解 -0.1459 含 INS 链路延迟，喂给前端后 21ms 错位打花高角速率段的去畸变与颜色采样（灯柱白度 51.5%→79.3%，牌面着色同步改善）；融合精度在该区间平坦（LIVO 120s xy max 7.12→7.08 cm，达标）。证据链见 docs/mcp-color-regression-time-offsets.md",
    "lidar_frontend": -0.125
  }
}
```
