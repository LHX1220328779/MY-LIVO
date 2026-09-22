# wuhu/truck29/v013_driven

- stage: driven (行驶段 6-DoF 精修)
- parent: wuhu/truck29/v012_ground
- lineage: wuhu/truck29/v007_import -> wuhu/truck29/v012_ground -> wuhu/truck29/v013_driven
- created: 2026-09-22T15:20:16+08:00
- note: pipeline: driven

## inputs

- bag: /home/project/data/haibo/wuhu_livo/ros2bag
- dataset: {'path': '/home/project/data/haibo/wuhu_livo/ros2bag', 'kind': 'raw_split', 'segments': 8, 'duration_s': 120.13, 'has_camera': True}
- run: /home/project/MY-LIVO/results/calib_truck29_lio_0922_151641/Log/fusion/1790061410918713566
- window: [8.0, 40.0]

## changes vs parent

```
{
  "lidars": {
    "front_right_lidar": {
      "dt_cm": [
        0.51,
        0.41,
        -1.75
      ],
      "drpy_deg": [
        0.065,
        -0.08,
        0.001
      ]
    },
    "back_lidar": {
      "dt_cm": [
        1.18,
        2.36,
        -8.14
      ],
      "drpy_deg": [
        -0.178,
        0.02,
        -0.023
      ]
    },
    "front_left_lidar": {
      "dt_cm": [
        1.33,
        0.93,
        -0.37
      ],
      "drpy_deg": [
        -0.06,
        -0.088,
        0.021
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
            -0.061787652758260106,
            0.08309966002559983,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            1.7757440047724524
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "front_left_lidar": {
          "rotvec_deg": [
            -0.05498358832349467,
            -0.09071537045934852,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            0.25823758995354185
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        },
        "back_lidar": {
          "rotvec_deg": [
            0.009767132835071034,
            -0.1868638944365991,
            0.0
          ],
          "trans_cm": [
            0.0,
            0.0,
            9.050264718778646
          ],
          "yaw_xy_solved": false,
          "about": "lidar position in rear-axle frame"
        }
      },
      "source": "/home/project/data/haibo/wuhu_livo/ros2bag",
      "stationary_frames": [
        2,
        20
      ],
      "moving_frames": [
        300,
        40
      ],
      "stage2_6dof": {
        "reference": "front_lidar",
        "corrections": {
          "front_right_lidar": {
            "rotvec_deg": [
              0.06292567974634049,
              -0.0808171322049373,
              0.004278399483238644
            ],
            "trans_cm": [
              0.5137975797933411,
              0.41345640926339494,
              -1.7539421844997227
            ],
            "about": "lidar position in rear-axle frame"
          },
          "front_left_lidar": {
            "rotvec_deg": [
              0.06237908820202024,
              0.08624923108652403,
              0.019714802712137696
            ],
            "trans_cm": [
              1.3303588226570182,
              0.9225817792768628,
              -0.3679102954397392
            ],
            "about": "lidar position in rear-axle frame"
          },
          "back_lidar": {
            "rotvec_deg": [
              0.02315547066328747,
              0.17792632902038602,
              -0.020783198765570794
            ],
            "trans_cm": [
              1.1791028847102698,
              2.356558457984472,
              -8.141383098742397
            ],
            "about": "lidar position in rear-axle frame"
          }
        },
        "run": "/home/project/MY-LIVO/results/calib_truck29_lio_0922_151641/Log/fusion/1790061410918713566",
        "window": [
          8.0,
          40.0
        ],
        "prior": [
          0.5,
          0.1
        ]
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
        2.835,
        5.769,
        1.8093
      ],
      "rpy_deg": [
        -0.802,
        -3.13,
        -1.216
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        0.0076,
        -0.5292,
        0.6086
      ],
      "rpy_deg": [
        -0.979,
        0.794,
        -91.174
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.033,
        5.5449,
        1.9503
      ],
      "rpy_deg": [
        -3.582,
        -0.661,
        178.827
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.033,
        7.793,
        0.051
      ],
      "rpy_deg": [
        1.3,
        -1.67,
        90.16
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      0.0742,
      7.7914,
      1.7519
    ],
    "rpy_deg": [
      -96.369,
      1.482,
      0.659
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
    "lidar": -0.125,
    "camera": -0.115,
    "rtk_position": 0.025,
    "ins_attitude": 0.0,
    "source": "import (verify with camera-time / LIO evaluation)"
  }
}
```

## stage report

```
calibration=/home/project/MY-LIVO/config/carlibration/wuhu/truck29/v012_ground/calibration.json
run=/home/project/MY-LIVO/results/calib_truck29_lio_0922_151641/Log/fusion/1790061410918713566
window 8.0..40.0 s, lidar_time_offset -0.125
scans used: front_right_lidar:322, back_lidar:322, front_left_lidar:322, front_lidar:322
reference front_lidar: 516100 world points
front_right_lidar: matches 206841->206782 |res| median 3.42->3.09 cm; ground 3.33->2.95 (n=168999); vertical 4.04->4.00 (n=18310); correction rot deg (x,y,z)=(+0.063,-0.081,+0.004) trans cm=(+0.5,+0.4,-1.8); iters 14
front_left_lidar: matches 234026->233813 |res| median 5.20->4.99 cm; ground 5.20->4.98 (n=153707); vertical 3.97->3.90 (n=27787); correction rot deg (x,y,z)=(+0.062,+0.086,+0.020) trans cm=(+1.3,+0.9,-0.4); iters 15
back_lidar: matches 47734->48003 |res| median 7.16->3.42 cm; ground 7.95->3.25 (n=38560); vertical 3.99->4.27 (n=3416); correction rot deg (x,y,z)=(+0.023,+0.178,-0.021) trans cm=(+1.2,+2.4,-8.1); iters 16
written /home/project/MY-LIVO/config/carlibration/wuhu/truck29/v013_driven/calibration.json
```
