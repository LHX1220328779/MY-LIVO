# zongyang/truck02/v014_rtktime

- stage: rtktime (RTK 位置关联时间)
- parent: zongyang/truck02/v012_driven
- lineage: zongyang/truck02/v001_import -> zongyang/truck02/v008_axle -> zongyang/truck02/v009_ground -> zongyang/truck02/v010_axle -> zongyang/truck02/v011_ground -> zongyang/truck02/v012_driven -> zongyang/truck02/v014_rtktime
- created: 2026-10-27T13:47:37+08:00
- note: RTK position time association +0.0250 -> +0.0669 s (secant of both rounds (measured gain 0.48))

## inputs

- run: /home/project/MY-LIVO/results/calib_truck02_rtk1_1027_133752/Log/fusion/1793079672819270433
- run2: /home/project/MY-LIVO/results/calib_truck02_rtk2_1027_133752/Log/fusion/1793079863325584037

## changes vs parent

```
{
  "time_offsets": {
    "rtk_position": {
      "from": 0.025,
      "to": 0.0669
    }
  }
}
```

## metrics

```
{
  "rtk_time": {
    "parent": "zongyang/truck02/v012_driven",
    "parent_rtk_position_s": 0.025,
    "round1": {
      "run": "/home/project/MY-LIVO/results/calib_truck02_rtk1_1027_133752/Log/fusion/1793079672819270433",
      "effective_s": 0.024300000000000002,
      "correction_s": 0.020092851367544864,
      "se_s": 0.0006697319826102887,
      "observable": true,
      "frames": 1186,
      "informative_voxels": 10285
    },
    "round2": {
      "run": "/home/project/MY-LIVO/results/calib_truck02_rtk2_1027_133752/Log/fusion/1793079863325584037",
      "effective_s": 0.057800000000000004,
      "correction_s": 0.004020775980558478,
      "se_s": 0.00029952063491026705,
      "observable": true,
      "frames": 1186,
      "informative_voxels": 10362
    },
    "method": "secant of both rounds (measured gain 0.48)",
    "gain": 0.48,
    "best_effective_s": 0.06618,
    "best_se_s": 0.00086,
    "rtk_position_s": {
      "from": 0.025,
      "to": 0.0669
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
        2.6732,
        4.9627,
        1.8177
      ],
      "rpy_deg": [
        -0.399,
        -0.102,
        -2.742
      ]
    },
    {
      "lidar": "back_lidar",
      "key": "transform_matrix_lidar_2",
      "t": [
        -0.1868,
        -0.987,
        0.8357
      ],
      "rpy_deg": [
        0.349,
        -3.349,
        -90.324
      ]
    },
    {
      "lidar": "front_left_lidar",
      "key": "transform_matrix_lidar_3",
      "t": [
        -3.1809,
        4.8924,
        1.8327
      ],
      "rpy_deg": [
        -1.846,
        -0.424,
        178.169
      ]
    },
    {
      "lidar": "front_lidar",
      "key": "transform_matrix_lidar_6",
      "t": [
        -0.0705,
        7.1162,
        2.4626
      ],
      "rpy_deg": [
        1.039,
        1.785,
        90.946
      ]
    }
  ],
  "camera_to_rear": {
    "t": [
      -0.1056,
      7.1264,
      2.1813
    ],
    "rpy_deg": [
      -90.039,
      3.007,
      -0.068
    ]
  },
  "camera": {
    "topic": "/midrange_camera/image_raw",
    "width": 1920,
    "height": 1080,
    "intrinsics": [
      611.054443359375,
      895.9816284179688,
      954.4519112515845,
      541.4709836341353
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
    "lidar": 0.0033,
    "lidar_frontend": 0.004,
    "camera": 0.0,
    "rtk_position": 0.0669,
    "ins_attitude": 0.0,
    "lidar_source": "rear-axle alignment (cross-time consistency vs INS poses)",
    "source": "import (verify with camera-time / LIO evaluation)",
    "lidar_frontend_source": "axle: lidar -0.0010 + imu/ins clock lag +0.0050 (corr 1.000); template -0.1250 replaced",
    "rtk_position_source": "rtk-time: map consistency, secant of both rounds (measured gain 0.48); effective +66.2 ms (se 0.9 ms)"
  }
}
```

## stage report

```
round 1: effective +24.30 ms -> correction +20.09 ms (se 0.67)
round 2: effective +57.80 ms -> correction +4.02 ms (se 0.30)
secant of both rounds (measured gain 0.48): best effective +66.18 ms (se 0.86 ms); rtk_position +0.0250 -> +0.0669 s

RTK time correction +20.09 ms (se 0.67 ms) -> best effective offset +44.39 ms (run +24.30); voxels 11290, informative 10285; observable
RTK time correction +4.02 ms (se 0.30 ms) -> best effective offset +61.82 ms (run +57.80); voxels 11328, informative 10362; observable
```
