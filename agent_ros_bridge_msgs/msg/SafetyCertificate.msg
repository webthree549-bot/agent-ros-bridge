# SafetyCertificate.msg
# Cryptographic safety certificate for motion plan validation

string plan_hash                  # SHA-256 hash of plan
string validation_id              # UUID for traceability
time valid_from                   # Certificate start time
time valid_until                  # Certificate expiry (30s window)

# Constraints that were verified
string[] constraints_checked

# Validation metadata
string planner_version            # Motion planner version
string verification_engine        # "SMT", "SAMPLING", etc.
string world_state_hash           # Hash of world state at validation

# Cryptographic signature
string signature                  # Signed by safety validator
string validator_id               # ID of validating entity
