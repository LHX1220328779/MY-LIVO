# huaining/truck03/v004_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: huaining/truck03/v003_ground
- lineage: huaining/truck03/v001_import -> huaining/truck03/v002_axle -> huaining/truck03/v003_ground -> huaining/truck03/v004_driven
- created: 2026-10-30T17:07:56+08:00
- note: pipeline: driven

## inputs

- bag: /home/project/data/haibo/huaining/03/0729/ros2bag
- dataset: {'path': '/home/project/data/haibo/huaining/03/0729/ros2bag', 'kind': 'raw_split', 'segments': 12, 'duration_s': 359.99, 'has_camera': False}
- run: /home/project/MY-LIVO/results/calib_truck03_lio_1030_170309/Log/fusion/1793351211543451536
- window: [8.0, 40.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        -4.14,
        -1.65,
        3.2
      ],
      "drpy_deg": [
        -0.086,
        -0.053,
        -0.048
      ]
    },
    "back_lidar": {
      "dt_cm": [
        8.12,
        2.66,
        -0.95
      ],
      "drpy_deg": [
        0.159,
        0.337,
        -0.139
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        0.69,
        -0.5,
        -0.35
      ],
      "drpy_deg": [
        0.052,
        0.042,
        -0.012
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
            0.11310304803525617,
            0.1912514069899519,
            0.014480608642623556
          ],
          "trans_cm": [
            -3.1591986859617034,
            2.5134523271551688,
            -2.619082305083534
          ],
          "yaw_xy_solved": true,
          "about": "lidar position in rear-axle frame",
          "chosen": "fine",
          "quality": {
            "level": "ok",
            "reasons": [],
            "iterations": 16,
            "matches": 6263,
            "ground_before_cm": 4.65,
            "ground_after_cm": 3.66,
            "rotation_deg": 0.223,
            "translation_cm": 4.8
          },
          "plane_check": {
            "tilt_deg": 0.924,
            "fore_aft_deg": 0.138,
            "lateral_deg": -0.914,
            "dz_cm": 2.1,
            "pairs": 1,
            "outside_basin": true,
            "applied": false
          }
        },
        "front_left_lidar": {
          "rotvec_deg": [
            -0.05304375828914993,
            -0.0860029711566217,
            0.01218302383900022
          ],
          "trans_cm": [
            -0.593223584857294,
            2.9133440626379583,
            -3.3888607884111064
          ],
          "yaw_xy_solved": true,
          "about": "lidar position in rear-axle frame",
          "chosen": "fine",
          "quality": {
            "level": "ok",
            "reasons": [],
            "iterations": 19,
            "matches": 6308,
            "ground_before_cm": 7.37,
            "ground_after_cm": 5.46,
            "rotation_deg": 0.102,
            "translation_cm": 4.5
          },
          "plane_check": {
            "tilt_deg": 0.55,
            "fore_aft_deg": -0.136,
            "lateral_deg": -0.533,
            "dz_cm": -0.5,
            "pairs": 1,
            "outside_basin": true,
            "applied": false
          }
        },
        "back_lidar": {
          "rotvec_deg": [
            0.1942529299575298,
            0.4712936197684819,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            -4.663451082220168
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame",
          "chosen": "fine",
          "quality": {
            "level": "ok",
            "reasons": [],
            "iterations": 25,
            "matches": 40607,
            "ground_before_cm": 10.57,
            "ground_after_cm": 8.25,
            "rotation_deg": 0.51,
            "translation_cm": 4.7
          },
          "plane_check": {
            "tilt_deg": 0.802,
            "fore_aft_deg": 0.541,
            "lateral_deg": -0.536,
            "dz_cm": -12.9,
            "pairs": 40,
            "outside_basin": true,
            "applied": false
          }
        }
      },
      "quality": "ok",
      "source": "/home/project/data/haibo/huaining/03/0729/ros2bag",
      "stationary_frames": [
        62,
        20
      ],
      "moving_frames": [
        3350,
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
              -0.0892491539071542,
              -0.04832806509401627,
              -0.05218220849247341
            ],
            "trans_cm": [
              -4.139260283248302,
              -1.6546783712104403,
              3.2055578586329068
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 7,
              "matches": 161911,
              "vertical_matches": 44655,
              "median_before_cm": 2.44,
              "median_after_cm": 1.58,
              "vertical_before_cm": 2.1,
              "vertical_after_cm": 1.91,
              "rotation_deg": 0.114,
              "translation_cm": 5.5,
              "match_time_gap_s": 0.6,
              "pass2": null
            }
          },
          "front_left_lidar": {
            "rotvec_deg": [
              -0.05366400299009216,
              -0.04038353758391797,
              -0.009275262343838882
            ],
            "trans_cm": [
              0.6866985543403558,
              -0.49940919982215826,
              -0.3451025700790528
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 7,
              "matches": 152321,
              "vertical_matches": 49775,
              "median_before_cm": 1.8,
              "median_after_cm": 1.64,
              "vertical_before_cm": 1.92,
              "vertical_after_cm": 1.72,
              "rotation_deg": 0.068,
              "translation_cm": 0.9,
              "match_time_gap_s": 0.6,
              "pass2": null
            }
          },
          "back_lidar": {
            "rotvec_deg": [
              0.3358676011352768,
              -0.1604128712228,
              -0.13983973601732153
            ],
            "trans_cm": [
              8.117887376139597,
              2.6555410042929313,
              -0.950800494426159
            ],
            "about": "lidar position in rear-axle frame",
            "quality": {
              "level": "ok",
              "reasons": [],
              "converged": true,
              "iterations": 12,
              "matches": 60634,
              "vertical_matches": 24129,
              "median_before_cm": 9.73,
              "median_after_cm": 2.67,
              "vertical_before_cm": 5.89,
              "vertical_after_cm": 2.37,
              "rotation_deg": 0.398,
              "translation_cm": 8.6,
              "match_time_gap_s": 9.6,
              "pass2": {
                "ground_before_cm": 7.88,
                "ground_after_cm": 2.66,
                "ground_matches": 33072,
                "delta_rotvec_deg": [
                  -0.0259,
                  -0.0974,
                  0.0
                ],
                "delta_trans_cm": [
                  0.0,
                  0.0,
                  7.36
                ],
                "iterations": 7
              }
            }
          }
        },
        "run": "/home/project/MY-LIVO/results/calib_truck03_lio_1030_170309/Log/fusion/1793351211543451536",
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
        "quality": "ok"
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
        1.9067,
        6.6593,
        0.9974
      ],
      "rpy_deg": [
        -1.897,
        -2.228,
        -3.331
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1729,
        -1.8103,
        0.427
      ],
      "rpy_deg": [
        -0.2,
        0.389,
        -90.371
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -1.8814,
        6.6856,
        1.0293
      ],
      "rpy_deg": [
        -2.244,
        -2.941,
        177.837
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0239,
        6.87,
        0.5194
      ],
      "rpy_deg": [
        0.126,
        -1.028,
        90.616
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.0596,
      6.6937,
      2.6443
    ],
    "rpy_deg": [
      -95.853,
      -0.046,
      -0.59
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      1644.2,
      1857.01,
      956.116,
      542.763
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
    "lidar": -0.0016,
    "lidar_frontend": 0.0034,
    "camera": 0.0,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0016 + imu/ins clock lag +0.0050 (corr 0.999); template -0.1250 replaced"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/huaining/truck03/v003_ground/calibration.json
run=/home/project/MY-LIVO/results/calib_truck03_lio_1030_170309/Log/fusion/1793351211543451536
window 8.0..40.0 s, lidar_time_offset 0.0034
pass 1 poses: native frames.csv (6 DoF); pass 2 poses: RTK-optimized trajectory_optimized.txt (tilt about x/y and z of LiDARs with median match time gap > 2.0 s)
scans used: front_right_lidar:321, back_lidar:321, front_left_lidar:321, front_lidar:321
scan source: round archive scans.bin/sources.bin (mapper deskew; returns at scan end, round extrinsics inverted)
matching: viewing-side consistent (oriented normals)
reference front_lidar: 187319 world points
pairing: same-time for LiDARs overlapping the reference (1.0 s blocks +-0.5 s, 1914213 block points); cross-time LiDARs use the accumulated target (nearest-range voxels)
front_right_lidar: pass 1 matches 175257->161911 |res| median 2.44->1.58 cm; ground 2.54->1.44 (n=112167); vertical 2.10->1.91 (n=44655); match time gap median 0.6 s; correction rot deg (x,y,z)=(-0.089,-0.048,-0.052) trans cm=(-4.1,-1.7,+3.2); iters 7  [quality: ok]
front_left_lidar: pass 1 matches 167760->152321 |res| median 1.80->1.64 cm; ground 1.70->1.57 (n=97508); vertical 1.92->1.72 (n=49775); match time gap median 0.6 s; correction rot deg (x,y,z)=(-0.054,-0.040,-0.009) trans cm=(+0.7,-0.5,-0.3); iters 7  [quality: ok]
back_lidar: pass 1 matches 60876->60634 |res| median 9.73->2.67 cm; ground 13.92->2.85 (n=33922); vertical 5.89->2.37 (n=24129); match time gap median 9.6 s; correction rot deg (x,y,z)=(+0.362,-0.063,-0.140) trans cm=(+8.1,+2.7,-8.3); iters 12
    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground 7.88->2.66 cm (n=33072); tilt about x/y adjusted by (-0.026,-0.097) deg, z by +7.4 cm; iters 7
    final correction rot deg (x,y,z)=(+0.336,-0.160,-0.140) trans cm=(+8.1,+2.7,-1.0)  [quality: ok]
quality: ok
written /home/project/MY-LIVO/config/carlibration/huaining/truck03/v004_driven/calibration.json
```
