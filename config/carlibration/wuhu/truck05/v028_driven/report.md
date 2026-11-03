# wuhu/truck05/v028_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck05/v027_ground
- lineage: wuhu/truck05/v008_import -> wuhu/truck05/v017_axle -> wuhu/truck05/v018_ground -> wuhu/truck05/v019_driven -> wuhu/truck05/v020_manual -> wuhu/truck05/v021_manual -> wuhu/truck05/v022_manual -> wuhu/truck05/v023_manual -> wuhu/truck05/v024_precheck -> wuhu/truck05/v025_driven -> wuhu/truck05/v026_axle -> wuhu/truck05/v027_ground -> wuhu/truck05/v028_driven
- created: 2026-09-29T14:51:02+08:00
- note: pipeline: driven

## inputs

- bag: /home/project/data/haibo/wuhu/05/20260914/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu/05/20260914/ros2bag', 'kind': 'raw_split', 'segments': 48, 'duration_s': 720.04, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck05_lio_0929_143644/Log/fusion/1790664404039131405
- window: [8.0, 40.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -2.58,
        0.9,
        -0.22
      ],
      "drpy_deg": [
        0.034,
        0.052,
        -0.008
      ]
    },
    "back_lidar": {
      "dt_cm": [
        0.73,
        0.93,
        1.26
      ],
      "drpy_deg": [
        -0.095,
        0.185,
        -0.003
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.6,
        -0.58,
        0.55
      ],
      "drpy_deg": [
        0.008,
        0.048,
        -0.009
      ]
    }
  }
}
```

## metrics

```
{
  "refinement": {
    "refinement": {
      "reference": "front_lidar",
      "corrections": {
        "front_right_lidar": {
          "rotvec_deg": [
            -0.1092163299316272,
            -0.1372571657112491,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            2.148189689532705
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "front_left_lidar": {
          "rotvec_deg": [
            -0.026249553004727092,
            0.080479833254946,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            -0.35107023357528555
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "back_lidar": {
          "rotvec_deg": [
            -0.2925261087002748,
            -0.1653555456976524,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            -0.8033865057801832
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        }
      },
      "source": "/home/project/data/haibo/wuhu/05/20260914/ros2bag",
      "stationary_frames": [
        2,
        20
      ],
      "moving_frames": [
        1840,
        40
      ],
      "stage2_6dof": {
        "reference": "front_lidar",
        "corrections": {
          "front_right_lidar": {
            "rotvec_deg": [
              0.03480495892921224,
              0.051251946056372864,
              -0.007429630363388547
            ],
            "trans_cm": [
              -2.586076606497729,
              0.8958309742437571,
              -0.2165871036211164
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 13,
              "matches": 276971,
              "vertical_matches": 100785,
              "median_before_cm": 3.93,
              "median_after_cm": 3.81,
              "vertical_before_cm": 3.28,
              "vertical_after_cm": 3.21,
              "rotation_deg": 0.062,
              "translation_cm": 2.7,
              "match_time_gap_s": 8.89,
              "pass2": {
                "ground_before_cm": 4.31,
                "ground_after_cm": 4.15,
                "ground_matches": 143890,
                "delta_rotvec_deg": [
                  -0.0415,
                  -0.0057,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  1.95
                ],
                "iterations": 11
              }
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              -0.007725244783290628,
              -0.047953381659489,
              -0.008905996077502174
            ],
            "trans_cm": [
              0.596772276943393,
              -0.5832349739765185,
              0.5516581202197023
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "warn",
              "reasons": [
                "overall residual regressed 3.96->3.97 cm"
              ],
              "converged": true,
              "iterations": 14,
              "matches": 284206,
              "vertical_matches": 91558,
              "median_before_cm": 3.96,
              "median_after_cm": 3.97,
              "vertical_before_cm": 2.82,
              "vertical_after_cm": 2.74,
              "rotation_deg": 0.049,
              "translation_cm": 1.0,
              "match_time_gap_s": 7.27,
              "pass2": {
                "ground_before_cm": 4.82,
                "ground_after_cm": 4.67,
                "ground_matches": 160190,
                "delta_rotvec_deg": [
                  -0.034,
                  0.0191,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  1.0
                ],
                "iterations": 11
              }
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              0.1854010257802491,
              0.09423781754203989,
              -0.0008005844746352259
            ],
            "trans_cm": [
              0.7321242091230066,
              0.9310496994878783,
              1.260808768743344
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 17,
              "matches": 185673,
              "vertical_matches": 104005,
              "median_before_cm": 3.93,
              "median_after_cm": 3.24,
              "vertical_before_cm": 2.78,
              "vertical_after_cm": 2.67,
              "rotation_deg": 0.208,
              "translation_cm": 1.7,
              "match_time_gap_s": 19.64,
              "pass2": {
                "ground_before_cm": 4.52,
                "ground_after_cm": 3.73,
                "ground_matches": 62574,
                "delta_rotvec_deg": [
                  -0.0338,
                  -0.0017,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  2.69
                ],
                "iterations": 13
              }
            }
          }
        },
        "run": "/home/project/MY-LIVO/results/calib_truck05_lio_0929_143644/Log/fusion/1790664404039131405",
        "window": [
          8.0,
          40.0
        ],
        "prior": [
          0.5,
          0.1
        ],
        "poses": {
          "pass1": "native frames.csv",
          "pass2": "RTK-optimized trajectory_optimized.txt",
          "cross_time_s": 2.0
        },
        "quality": "warn"
      }
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
        2.7975,
        5.5716,
        1.9665
      ],
      "rpy_deg": [
        0.676,
        -2.201,
        -1.157
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.0165,
        -0.6675,
        0.8613
      ],
      "rpy_deg": [
        -0.966,
        1.183,
        -90.16
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.0725,
        5.4739,
        1.9276
      ],
      "rpy_deg": [
        -0.446,
        -1.896,
        179.559
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0973,
        7.6438,
        0.5356
      ],
      "rpy_deg": [
        0.058,
        -5.285,
        89.869
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0014,
      7.6498,
      2.2531
    ],
    "rpy_deg": [
      -100.4,
      -0.011,
      -0.455
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1849.37,
      1849.37,
      1003.31,
      502.72
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
            1849.32,
            1831.26,
            1004.09,
            499.42
          ],
          "rms_px": 0.05,
          "circle_centre_rear_m": [
            -0.002,
            7.529
          ],
          "ground_z_rear_m": -1.283
        },
        "square": {
          "intrinsics": [
            1849.37,
            1849.37,
            1003.31,
            502.72
          ],
          "rms_px": 0.05,
          "circle_centre_rear_m": [
            0.001,
            7.535
          ],
          "ground_z_rear_m": -1.249
        }
      },
      "model": "square pixels (fx = fy)",
      "rms_px": 0.05,
      "camera_height_m": 3.502,
      "deviation_vs_truck29_defaults": {
        "fx_pct": 12.59,
        "fy_pct": -0.33,
        "cx_px": 32.3,
        "cy_px": -31.5
      }
    },
    "source": "supplier ground-distance curves equation_*m (square pixels (fx = fy), rms 0.05 px)"
  },
  "time_offsets": {
    "lidar": -0.0267,
    "camera": -0.0019,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend": -0.0119,
    "camera_source": "operator set (livo_calib.py set): 相机偏移按同源时钟规则：camera_h = 家族值-0.115 + 打戳差0.1131 = -0.0019 ≈ 0，与 v016 时代 img=0 最优的实测互证"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck05/v027_ground/calibration.json
run=/home/project/MY-LIVO/results/calib_truck05_lio_0929_143644/Log/fusion/1790664404039131405
window 8.0..40.0 s, lidar_time_offset -0.0119
pass 1 poses: native frames.csv (6 DoF); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:322, back_lidar:322, front_left_lidar:322, front_lidar:322
matching: viewing-side consistent (oriented normals)
reference front_lidar: 572397 world points
front_right_lidar: pass 1 matches 277437->276971 |res| median 3.93->3.81 cm; ground 4.35->4.22 (n=140392); vertical 3.28->3.21 (n=100785); match time gap median 8.9 s; correction rot deg (x,y,z)=(+0.076,+0.057,-0.007) trans cm=(-2.6,+0.9,-2.2); iters 13
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 4.31->4.15 cm (n=143890); tilt about x/y adjusted by (-0.042,-0.006) deg, z by +1.9 cm; iters 11
    final correction rot deg (x,y,z)=(+0.035,+0.051,-0.007) trans cm=(-2.6,+0.9,-0.2)  [quality: ok]
front_left_lidar: pass 1 matches 284366->284206 |res| median 3.96->3.97 cm; ground 4.78->4.85 (n=154359); vertical 2.82->2.74 (n=91558); match time gap median 7.3 s; correction rot deg (x,y,z)=(+0.026,-0.067,-0.009) trans cm=(+0.6,-0.6,-0.5); iters 14
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 4.82->4.67 cm (n=160190); tilt about x/y adjusted by (-0.034,+0.019) deg, z by +1.0 cm; iters 11
    final correction rot deg (x,y,z)=(-0.008,-0.048,-0.009) trans cm=(+0.6,-0.6,+0.6)  [quality: warn; overall residual regressed 3.96->3.97 cm]
back_lidar: pass 1 matches 186964->185673 |res| median 3.93->3.24 cm; ground 7.30->4.35 (n=59656); vertical 2.78->2.67 (n=104005); match time gap median 19.6 s; correction rot deg (x,y,z)=(+0.219,+0.096,-0.001) trans cm=(+0.7,+0.9,-1.4); iters 17
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 4.52->3.73 cm (n=62574); tilt about x/y adjusted by (-0.034,-0.002) deg, z by +2.7 cm; iters 13
    final correction rot deg (x,y,z)=(+0.185,+0.094,-0.001) trans cm=(+0.7,+0.9,+1.3)  [quality: ok]
quality: warn  (front_left_lidar: overall residual regressed 3.96->3.97 cm)
written /home/project/MY-LIVO/config/carlibration/wuhu/truck05/v028_driven/calibration.json
```
