#!/usr/bin/env python3
"""Unit Tests for HAL"""

def test_sensor_hal(sensor_hal):
    assert sensor_hal.initialized is True
    
def test_actuator_hal(actuator_hal):
    assert actuator_hal.initialized is True