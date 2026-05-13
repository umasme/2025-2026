#!/usr/bin/env python3
"""
Simple ZMQ subscriber to print perception payloads from tcp://localhost:5555
Run this in a separate terminal alongside `perception.py`.
"""
import zmq
import json
import time

ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.connect("tcp://localhost:5555")
sock.setsockopt_string(zmq.SUBSCRIBE, "")

print("Listening on tcp://localhost:5555")
try:
    while True:
        msg = sock.recv_string()
        p = json.loads(msg)
        ts = time.time()
        imu_yaw = p.get('imu_yaw_deg')
        imu_rate = p.get('imu_yaw_rate')
        print(f"{ts:.3f}  imu_yaw_deg={imu_yaw}  imu_yaw_rate={imu_rate}  obstacles={len(p.get('obstacles', []))}")
except KeyboardInterrupt:
    print('\nSubscriber stopped')
