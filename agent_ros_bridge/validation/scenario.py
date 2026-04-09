"""Validation scenarios for testing robot behavior."""

from dataclasses import dataclass
from typing import Any


@dataclass
class ValidationResult:
    """Result of validation scenario."""

    passed: bool
    message: str
    details: dict[str, Any]


@dataclass
class ValidationScenario:
    """A validation scenario with success criteria."""

    name: str
    success_criteria: dict[str, Any]
    description: str = ""

    def validate(self, data: dict[str, Any]) -> ValidationResult:
        """Validate data against success criteria.

        Args:
            data: Data to validate (metrics, results, etc.)

        Returns:
            ValidationResult with pass/fail status
        """
        details = {}
        all_passed = True

        # Check min_agreement_rate
        if "min_agreement_rate" in self.success_criteria:
            required = self.success_criteria["min_agreement_rate"]
            actual = data.get("agreement_rate", 0)
            passed = actual >= required
            details["agreement_rate"] = {
                "required": required,
                "actual": actual,
                "passed": passed,
            }
            if not passed:
                all_passed = False

        # Check min_decisions
        if "min_decisions" in self.success_criteria:
            required = self.success_criteria["min_decisions"]
            actual = data.get("total_decisions", 0)
            passed = actual >= required
            details["total_decisions"] = {
                "required": required,
                "actual": actual,
                "passed": passed,
            }
            if not passed:
                all_passed = False

        # Check safety violations
        if "max_safety_violations" in self.success_criteria:
            allowed = self.success_criteria["max_safety_violations"]
            actual = data.get("safety_violations", 0)
            passed = actual <= allowed
            details["safety_violations"] = {
                "allowed": allowed,
                "actual": actual,
                "passed": passed,
            }
            if not passed:
                all_passed = False

        message = (
            f"Scenario '{self.name}' passed" if all_passed else f"Scenario '{self.name}' failed"
        )

        return ValidationResult(
            passed=all_passed,
            message=message,
            details=details,
        )
