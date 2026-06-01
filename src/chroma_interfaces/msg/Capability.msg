# Capability.msg
# Abstracts robot hardware capabilities into standardized metrics

string type         # e.g. VISION, MOBILITY, BATTERY
float32 value       # health score (0.0, 1.0)
bool is_degraded    # degradation flag when below threshold