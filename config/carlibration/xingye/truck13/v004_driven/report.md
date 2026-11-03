# xingye/truck13/v004_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: xingye/truck13/v003_ground
- lineage: xingye/truck13/v001_import -> xingye/truck13/v002_axle -> xingye/truck13/v003_ground -> xingye/truck13/v004_driven
- created: 2026-10-28T11:05:00+08:00
- note: pipeline: driven

## inputs

- bag: /home/project/data/haibo/xingye/13/20261027/ros2bag
- dataset: {'path': '/home/project/data/haibo/xingye/13/20261027/ros2bag', 'kind': 'raw_split', 'segments': 68, 'duration_s': 1020.12, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck13_lio_1028_105834/Log/fusion/1793156635908573853
- window: [8.0, 40.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -5.42,
        5.17,
        0.35
      ],
      "drpy_deg": [
        0.036,
        -0.008,
        -0.096
      ]
    },
    "back_lidar": {
      "dt_cm": [
        -6.26,
        1.28,
        0.24
      ],
      "drpy_deg": [
        -0.28,
        0.236,
        -0.172
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        -4.62,
        2.17,
        2.12
      ],
      "drpy_deg": [
        0.068,
        0.032,
        -0.185
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
            0.2549543079585278,
            -0.12109284690624032,
            -0.06167797474179967
          ],
          "trans_cm": [
            -2.1650166100298383,
            -1.6490951181482476,
            -2.876557992511415
          ],
          "yaw_xy_solved": true,
          "about": "lidar position in rear-axle frame",
          "chosen": "fine",
          "quality": {
            "level": "warn",
            "reasons": [
              "plane start applied (0.66 deg / -7.6 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage"
            ],
            "iterations": 13,
            "matches": 3612,
            "ground_before_cm": 5.62,
            "ground_after_cm": 1.9,
            "rotation_deg": 0.289,
            "translation_cm": 4.0
          },
          "plane_check": {
            "tilt_deg": 0.662,
            "fore_aft_deg": 0.643,
            "lateral_deg": -0.156,
            "dz_cm": -7.6,
            "pairs": 1,
            "outside_basin": true,
            "applied": true
          }
        },
        "front_left_lidar": {
          "rotvec_deg": [
            0.49245328248145304,
            0.04066576856239458,
            -0.22524329872624155
          ],
          "trans_cm": [
            -6.215178266483271,
            0.6070177690514329,
            -6.89466943163533
          ],
          "yaw_xy_solved": true,
          "about": "lidar position in rear-axle frame",
          "chosen": "fine",
          "quality": {
            "level": "warn",
            "reasons": [
              "plane start applied (0.51 deg / -6.4 cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage"
            ],
            "iterations": 13,
            "matches": 3528,
            "ground_before_cm": 8.11,
            "ground_after_cm": 2.07,
            "rotation_deg": 0.543,
            "translation_cm": 9.3
          },
          "plane_check": {
            "tilt_deg": 0.514,
            "fore_aft_deg": 0.482,
            "lateral_deg": -0.179,
            "dz_cm": -6.4,
            "pairs": 1,
            "outside_basin": true,
            "applied": true
          }
        },
        "back_lidar": {
          "rotvec_deg": [
            0.23698768120591027,
            -0.4566570715653413,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            -8.977948710825828
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame",
          "chosen": "fine",
          "quality": {
            "level": "ok",
            "reasons": [],
            "iterations": 11,
            "matches": 41983,
            "ground_before_cm": 9.09,
            "ground_after_cm": 5.19,
            "rotation_deg": 0.514,
            "translation_cm": 9.0
          },
          "plane_check": {
            "tilt_deg": 1.409,
            "fore_aft_deg": 1.333,
            "lateral_deg": 0.339,
            "dz_cm": -21.3,
            "pairs": 40,
            "outside_basin": true,
            "applied": false
          }
        }
      },
      "quality": "warn",
      "source": "/home/project/data/haibo/xingye/13/20261027/ros2bag",
      "stationary_frames": [
        2,
        20
      ],
      "moving_frames": [
        6990,
        40
      ],
      "coarse_threshold": [
        0.5,
        0.05
      ],
      "stage2_6dof": {
        "reference": "front_lidar",
        "corrections": {
          "front_right_lidar": {
            "rotvec_deg": [
              0.03698024385130998,
              0.000628401761333237,
              -0.09513140096328059
            ],
            "trans_cm": [
              -5.421837395146518,
              5.1690516346621065,
              0.347106960900382
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "warn",
              "reasons": [
                "only 44 vertical planar matches (yaw/x/y weakly observed)"
              ],
              "converged": true,
              "iterations": 14,
              "matches": 128152,
              "vertical_matches": 44,
              "median_before_cm": 1.69,
              "median_after_cm": 1.44,
              "vertical_before_cm": 3.68,
              "vertical_after_cm": 2.73,
              "rotation_deg": 0.102,
              "translation_cm": 7.5,
              "match_time_gap_s": 0.5,
              "pass2": null
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              -0.07356213380197471,
              -0.017561139071772015,
              -0.1848517820903876
            ],
            "trans_cm": [
              -4.619721746456103,
              2.163854552465647,
              2.117768397567398
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "warn",
              "reasons": [
                "only 67 vertical planar matches (yaw/x/y weakly observed)"
              ],
              "converged": true,
              "iterations": 10,
              "matches": 159899,
              "vertical_matches": 67,
              "median_before_cm": 1.73,
              "median_after_cm": 1.3,
              "vertical_before_cm": 3.15,
              "vertical_after_cm": 2.42,
              "rotation_deg": 0.2,
              "translation_cm": 5.5,
              "match_time_gap_s": 0.5,
              "pass2": null
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              0.23563280087282873,
              0.279859479429467,
              -0.1897558614523027
            ],
            "trans_cm": [
              -6.262833053064061,
              1.2783410857864763,
              0.24183953169615832
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "warn",
              "reasons": [
                "only 70 vertical planar matches (yaw/x/y weakly observed)"
              ],
              "converged": true,
              "iterations": 7,
              "matches": 11378,
              "vertical_matches": 70,
              "median_before_cm": 3.42,
              "median_after_cm": 1.63,
              "vertical_before_cm": 16.1,
              "vertical_after_cm": 10.71,
              "rotation_deg": 0.412,
              "translation_cm": 6.4,
              "match_time_gap_s": 10.0,
              "pass2": {
                "ground_before_cm": 2.34,
                "ground_after_cm": 1.54,
                "ground_matches": 11126,
                "delta_rotvec_deg": [
                  -0.0596,
                  0.0151,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  1.19
                ],
                "iterations": 4
              }
            }
          }
        },
        "run": "/home/project/MY-LIVO/results/calib_truck13_lio_1028_105834/Log/fusion/1793156635908573853",
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
        "pairing": {
          "mode": "same-time",
          "block_s": 1.0,
          "pad_s": 0.5
        },
        "scan_source": "archive",
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
        1.9464,
        7.0057,
        1.0298
      ],
      "rpy_deg": [
        -1.755,
        -0.435,
        14.043
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1422,
        -1.9111,
        0.4543
      ],
      "rpy_deg": [
        0.043,
        -3.423,
        -90.139
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -1.9445,
        6.9514,
        1.0512
      ],
      "rpy_deg": [
        0.848,
        -0.57,
        167.928
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0002,
        7.0096,
        0.6541
      ],
      "rpy_deg": [
        0.426,
        2.516,
        90.688
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0123,
      7.0409,
      1.4569
    ],
    "rpy_deg": [
      -98.675,
      1.353,
      -0.512
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1642.15,
      1855.56,
      963.247,
      544.29
    ],
    "distortion": [
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    ],
    "source": "midrange_camera[0].camera_in_params"
  },
  "time_offsets": {
    "lidar": -0.0502,
    "lidar_frontend": -0.0427,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0502 + imu/ins clock lag +0.0075 (corr 0.997); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/xingye/truck13/v003_ground/calibration.json
run=/home/project/MY-LIVO/results/calib_truck13_lio_1028_105834/Log/fusion/1793156635908573853
window 8.0..40.0 s, lidar_time_offset -0.0427
pass 1 poses: native frames.csv (6 DoF); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:322, back_lidar:322, front_left_lidar:322, front_lidar:322
scan source: round archive scans.bin/sources.bin (mapper deskew; returns at scan end, round extrinsics inverted)
matching: viewing-side consistent (oriented normals)
reference front_lidar: 129913 world points
pairing: same-time for LiDARs overlapping the reference (1.0 s blocks +-0.5 s, 1342503 block points); cross-time LiDARs use the accumulated target (nearest-range voxels)
front_right_lidar: pass 1 matches 146391->128152 |res| median 1.69->1.44 cm; ground 1.67->1.43 (n=126765); vertical 3.68->2.73 (n=44); match time gap median 0.5 s; correction rot deg (x,y,z)=(+0.037,+0.001,-0.095) trans cm=(-5.4,+5.2,+0.3); iters 14  [quality: warn; only 44 vertical planar matches (yaw/x/y weakly observed)]
front_left_lidar: pass 1 matches 183951->159899 |res| median 1.73->1.30 cm; ground 1.72->1.29 (n=158923); vertical 3.15->2.42 (n=67); match time gap median 0.5 s; correction rot deg (x,y,z)=(-0.074,-0.018,-0.185) trans cm=(-4.6,+2.2,+2.1); iters 10  [quality: warn; only 67 vertical planar matches (yaw/x/y weakly observed)]
back_lidar: pass 1 matches 11369->11378 |res| median 3.42->1.63 cm; ground 3.37->1.60 (n=11098); vertical 16.10->10.71 (n=70); match time gap median 10.0 s; correction rot deg (x,y,z)=(+0.295,+0.265,-0.190) trans cm=(-6.3,+1.3,-0.9); iters 7
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 2.34->1.54 cm (n=11126); tilt about x/y adjusted by (-0.060,+0.015) deg, z by +1.2 cm; iters 4
    final correction rot deg (x,y,z)=(+0.236,+0.280,-0.190) trans cm=(-6.3,+1.3,+0.2)  [quality: warn; only 70 vertical planar matches (yaw/x/y weakly observed)]
quality: warn  (front_right_lidar: only 44 vertical planar matches (yaw/x/y weakly observed); front_left_lidar: only 67 vertical planar matches (yaw/x/y weakly observed); back_lidar: only 70 vertical planar matches (yaw/x/y weakly observed))
written /home/project/MY-LIVO/config/carlibration/xingye/truck13/v004_driven/calibration.json
```
