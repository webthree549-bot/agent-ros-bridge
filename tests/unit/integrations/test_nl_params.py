"""Tests for nl_params module."""

import pytest

from agent_ros_bridge.integrations.nl_params import infer_angle, infer_speed, infer_distance, infer_parameter, parse_numeric


class TestInferAngle:
    """Test angle inference."""

    def test_infer_angle_sharp(self):
        """Test sharp turn."""
        assert infer_angle("sharp") == 90

    def test_infer_angle_slight(self):
        """Test slight turn - 'slightly' maps to 10 degrees."""
        assert infer_angle("slightly") == 10

    def test_infer_angle_full(self):
        """Test full turn - maps to 360."""
        assert infer_angle("full") == 360

    def test_infer_angle_half(self):
        """Test half turn - maps to 180."""
        assert infer_angle("half") == 180

    def test_infer_angle_quarter(self):
        """Test quarter turn - maps to 90."""
        assert infer_angle("quarter") == 90

    def test_infer_angle_default(self):
        """Test default angle when unknown."""
        assert infer_angle("unknown") == 90.0

    def test_infer_angle_empty(self):
        """Test empty modifier returns default."""
        assert infer_angle("") == 90.0


class TestInferSpeed:
    """Test speed inference."""

    def test_infer_speed_slow(self):
        """Test slow speed."""
        assert infer_speed("slow") == 0.2

    def test_infer_speed_fast(self):
        """Test fast speed."""
        assert infer_speed("fast") == 1.0

    def test_infer_speed_quick(self):
        """Test quick speed - maps to 1.2."""
        assert infer_speed("quick") == 1.2

    def test_infer_speed_careful(self):
        """Test careful speed - not in mapping, returns default."""
        # 'careful' is not in the mapping, so it returns default 0.5
        assert infer_speed("careful") == 0.5

    def test_infer_speed_normal(self):
        """Test normal speed."""
        assert infer_speed("normal") == 0.5

    def test_infer_speed_unknown(self):
        """Test unknown modifier returns default."""
        assert infer_speed("unknown") == 0.5

    def test_infer_speed_empty(self):
        """Test empty modifier returns default."""
        assert infer_speed("") == 0.5


class TestInferDistance:
    """Test distance inference."""

    def test_infer_distance_default(self):
        """Test default distance."""
        assert infer_distance("") == 1.0

    def test_infer_distance_a_bit(self):
        """Test 'a bit' distance."""
        assert infer_distance("a bit") == 0.5


class TestInferParameter:
    """Test parameter inference."""

    def test_infer_parameter_valid(self):
        """Test valid parameter."""
        assert infer_parameter("speed", "slow") == 0.2

    def test_infer_parameter_invalid_type(self):
        """Test invalid parameter type."""
        assert infer_parameter("unknown", "value") is None

    def test_infer_parameter_not_found(self):
        """Test parameter not found in mapping."""
        assert infer_parameter("speed", "notamatch") is None


class TestParseNumeric:
    """Test numeric parsing."""

    def test_parse_numeric_with_number(self):
        """Test parsing number from string."""
        assert parse_numeric("90 degrees") == 90.0

    def test_parse_numeric_with_decimal(self):
        """Test parsing decimal number."""
        assert parse_numeric("1.5 meters") == 1.5

    def test_parse_numeric_no_number(self):
        """Test parsing string without number."""
        assert parse_numeric("no numbers here") is None

    def test_parse_numeric_with_about(self):
        """Test parsing with 'about' prefix."""
        assert parse_numeric("about 5 meters") == 5.0
