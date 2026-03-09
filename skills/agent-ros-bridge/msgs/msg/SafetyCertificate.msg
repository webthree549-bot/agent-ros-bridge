# SafetyCertificate.msg
# A cryptographic certificate validating that a trajectory has passed safety checks
# Valid for 30 seconds from issuance

string validation_id          # Unique UUID for this validation
builtin_interfaces/Time issued_at      # When the certificate was issued
builtin_interfaces/Time expires_at     # When the certificate expires (issued_at + 30s)
string plan_hash              # SHA-256 hash of the validated trajectory plan
bool[] constraints_checked    # Boolean array indicating which constraints were verified
string[] constraint_names     # Names of constraints corresponding to constraints_checked array
