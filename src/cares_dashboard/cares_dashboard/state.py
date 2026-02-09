from dataclasses import dataclass, field
from typing import Dict, List, Any

# Global Data Store
robots: Dict[str, Any] = {}
tasks: Dict[str, Any] = {}
bids: Dict[str, Dict[str, float]] = {}
allocations: List[Dict[str, Any]] = []
logs: List[str] = []

metrics = {
    "active_units": 0,
    "victims_found": 0,
    "tasks_completed": 0,
    "swarm_mean_cap": 0.0,
    "network_stability": 100
}

def log_event(message: str):
    import time
    ts = time.strftime("%H:%M:%S")
    entry = f"[{ts}] {message}"
    logs.insert(0, entry)
    if len(logs) > 50: logs.pop()

def get_robot(rid):
    if rid not in robots:
        robots[rid] = {
            'id': rid,
            'type': 'UGV' if 'tb3' in rid else 'UAV',
            'status': 'IDLE',
            'battery': 100.0,
            'caps': {'MOBILITY': 0.0, 'SENSING': 0.0, 'NETWORK': 0.0},
            'history': {'mob': [], 'sen': [], 'net': []}, # For time-series
            'pose': {'x': 0, 'y': 0, 'z': 0}
        }
    return robots[rid]