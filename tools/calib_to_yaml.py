#!/usr/bin/env python3
"""캘리브레이션 결과를 velodyne16_vn100.yaml 에 붙여넣을 형태로 출력한다.

사용법:
    python3 tools/calib_to_yaml.py \
        --imu       calib/imu.yaml \
        --cam-imu   calib/cam_imu-camchain-imucam.yaml \
        --transform ~/rosbags/lcc_conf/transform_result.txt

--transform 는 4단계(ankitdhall) 가 출력한 LiDAR->Camera 4x4 행렬.
파일 대신 --rcl / --pcl 로 직접 넣어도 된다.
"""
import argparse
import sys

import numpy as np
import yaml

np.set_printoptions(precision=9, suppress=True)


def fmt_mat(m, indent):
    rows = [', '.join(f'{v: .9f}' for v in r) for r in m]
    pad = ' ' * indent
    return f'[{rows[0]},\n{pad}{rows[1]},\n{pad}{rows[2]}]'


def fmt_vec(v):
    return '[' + ', '.join(f'{x: .9f}' for x in v) + ']'


p = argparse.ArgumentParser()
p.add_argument('--imu', help='1단계 imu.yaml')
p.add_argument('--cam-imu', help='3단계 cam_imu-camchain-imucam.yaml')
p.add_argument('--transform', help='4단계 LiDAR->Camera 4x4 행렬 텍스트 파일')
p.add_argument('--rcl', nargs=9, type=float, help='4단계 회전 9개 (행 우선)')
p.add_argument('--pcl', nargs=3, type=float, help='4단계 이동 3개')
a = p.parse_args()

print('=' * 66)
print('velodyne16_vn100.yaml 에 붙여넣을 값')
print('=' * 66)

# ---- IMU 노이즈 -> acc_cov / gyr_cov
if a.imu:
    d = yaml.safe_load(open(a.imu))
    rate = float(d['update_rate'])
    acc = float(d['accelerometer_noise_density']) ** 2 * rate
    gyr = float(d['gyroscope_noise_density']) ** 2 * rate
    print('\n  imu:')
    print(f'    acc_cov: {acc:.14g}')
    print(f'    gyr_cov: {gyr:.14g}')

# ---- Camera-IMU -> img_time_offset, 그리고 역산에 쓸 T_cam_imu
R_ci = t_ci = None
if a.cam_imu:
    d = yaml.safe_load(open(a.cam_imu))
    cam0 = d['cam0']
    T = np.array(cam0['T_cam_imu'])
    R_ci, t_ci = T[:3, :3], T[:3, 3]
    print('\n  time_offset:')
    print(f'    img_time_offset: {cam0["timeshift_cam_imu"]}')
    intr = cam0['intrinsics']
    dist = cam0['distortion_coeffs']
    print('\n  (camera_d435.yaml)')
    for k, v in zip(('fx', 'fy', 'cx', 'cy'), intr):
        print(f'    {k}: {v}')
    for i, v in enumerate(dist):
        print(f'    d{i}: {v}')

# ---- LiDAR-Camera -> Rcl / Pcl
Rcl = Pcl = None
if a.transform:
    M = np.array([[float(x) for x in line.split()]
                  for line in open(a.transform)
                  if line.strip() and not line.startswith('#')])
    if M.shape == (4, 4):
        Rcl, Pcl = M[:3, :3], M[:3, 3]
    else:
        sys.exit(f'4x4 행렬이 아님: {M.shape}')
elif a.rcl and a.pcl:
    Rcl = np.array(a.rcl).reshape(3, 3)
    Pcl = np.array(a.pcl)

if Rcl is not None:
    print('\n  extrin_calib:')
    print(f'    Rcl: {fmt_mat(Rcl, 10)}')
    print(f'    Pcl: {fmt_vec(Pcl)}')

    # ---- LiDAR->IMU 역산
    if R_ci is not None:
        R_ic = R_ci.T
        extR = R_ic @ Rcl
        extT = R_ic @ (Pcl - t_ci)
        print(f'    extrinsic_R: {fmt_mat(extR, 18)}')
        print(f'    extrinsic_T: {fmt_vec(extT)}')
    else:
        print('\n  (extrinsic_R/T 는 --cam-imu 를 같이 줘야 계산됨)')

print()
