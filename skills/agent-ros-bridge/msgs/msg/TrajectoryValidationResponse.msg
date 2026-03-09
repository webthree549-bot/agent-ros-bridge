# TrajectoryValidationResponse.msg
# Response from the trajectory validator

bool approved                  # True if trajectory passes all safety checks
SafetyCertificate certificate  # Valid safety certificate (only if approved=True)
string rejection_reason        # Reason for rejection (only if approved=False)
float64 validation_time_ms     # Time taken to validate (for performance monitoring)
