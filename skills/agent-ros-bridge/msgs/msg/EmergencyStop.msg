# EmergencyStop.msg
# Emergency stop status message
# Published by the emergency stop node when triggered or cleared

bool triggered                 # True if emergency stop is active
string source                  # Source of the e-stop: "HUMAN", "WATCHDOG", "VALIDATOR", "AI"
builtin_interfaces/Time trigger_time   # When the emergency stop was triggered
string reason                  # Human-readable reason for the emergency stop
