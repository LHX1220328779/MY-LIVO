# wuhu/truck29/v109_gcons

- stage: gcons (后向雷达地面一致性（tilt/z）)
- parent: wuhu/truck29/v103_rtktime
- lineage: wuhu/truck29/v098_import -> wuhu/truck29/v099_axle -> wuhu/truck29/v100_ground -> wuhu/truck29/v101_driven -> wuhu/truck29/v103_rtktime -> wuhu/truck29/v109_gcons
- created: 2026-10-12T20:54:08+08:00
- note: rear-facing LiDAR tilt/z from the multi-LiDAR ground consensus

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck29_rtk1_1012_112328/Log/fusion/1791775826844345637

## changes vs parent

```
{
  "lidars": {
    "back_lidar": {
      "dt_cm": [
        0.0,
        0.0,
        6.48
      ],
      "drpy_deg": [
        0.051,
        0.325,
        0.001
      ]
    }
  }
}
```

## metrics

```
{
  "ground_consensus": {
    "corrections": {
      "back_lidar": {
        "rotvec_deg": [
          0.3234,
          -0.0582,
          0.0
        ],
        "trans_cm": [
          0.0,
          0.0,
          6.48
        ],
        "about": "lidar position in rear-axle frame",
        "quality": {
          "level": "ok",
          "reasons": [],
          "rotation_deg": 0.329,
          "z_cm": 6.48,
          "effect_rms_cm": 3.11,
          "halves_rot_deg": 0.051,
          "halves_z_cm": 0.19,
          "median_abs_res_before_cm": 1.53,
          "median_abs_res_after_cm": 1.53
        }
      }
    },
    "run": "/home/project/MY-LIVO/results/calib_truck29_rtk1_1012_112328/Log/fusion/1791775826844345637",
    "targets": [
      "back_lidar"
    ],
    "applied": [
      "back_lidar"
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
        0.7093
      ],
      "rpy_deg": [
        -0.991,
        0.996,
        -91.31
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
    "rtk_position": -0.0174,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "rtk_position_source": "rtk-time: map consistency, first + probe rounds with the prior gain 0.6, standard-error weighted (secant gain 0.71 not usable); effective -38.3 ms (se 5.5 ms)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v103_rtktime/calibration.json
run=/home/project/MY-LIVO/results/calib_truck29_rtk1_1012_112328/Log/fusion/1791775826844345637
window 0.0..117.3 s, stride 4, scans front_right_lidar:292, back_lidar:292, front_left_lidar:292, front_lidar:292; ground-band returns 4265618
front_right_lidar: ground returns 999303, median along-track position +11.2 m (kept (overlaps the reference, driven stage))
back_lidar: ground returns 412263, median along-track position -12.0 m (rear-facing -> solved)
front_left_lidar: ground returns 1084874, median along-track position +5.9 m (kept (overlaps the reference, driven stage))
back_lidar: correction rotvec x +0.323 y -0.058 deg (about the LiDAR, rear-axle frame), z +6.48 cm; own-ground change RMS 3.1 cm; flat-ground |res| median 1.53->1.53 cm (n=192339); signed median by range: 4-10 m -3.4->-0.5, 10-20 m +0.9->+0.4, 20-40 m +3.4->-0.7; halves: 0.051 deg / 0.2 cm  [quality: ok]
result: applied to back_lidar
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v109_gcons/calibration.json
```
