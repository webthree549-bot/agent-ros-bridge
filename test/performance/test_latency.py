#!/usr/bin/env python3
"""Performance Tests - Latency"""
import time

def test_hal_read_latency(sensor_hal):
    start = time.time()
    sensor_hal.read()
    latency = (time.time() - start) * 1000
    assert latency < 100  # Less than 100ms