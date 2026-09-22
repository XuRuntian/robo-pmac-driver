"""Report constrained IK attempts with residual error, not idle boundaries."""
import numpy as np


class IKLimitMonitor:
    def __init__(self):
        self.active = {}
        self._last_report = float("-inf")

    def update(self, result, now: float) -> str | None:
        # Held commands aren't new solves; converged solves aren't blocked.
        active = dict(result.limit_attempts) if result is not None and not result.converged else {}
        previous = self.active
        self.active = active
        changed = active.keys() != previous.keys()
        if not changed and (not active or now - self._last_report < 2.0):
            return None
        self._last_report = now
        parts = []
        for name, (attempted, limit) in active.items():
            unit = "m" if name.startswith("d.") else "rad"
            parts.append(f"{name} proposed={attempted:.6f} limit={limit:.6f} {unit}")
        if active:
            parts.append(f"converged=false weighted_residual={np.linalg.norm(result.error):.6g}")
        cleared = sorted(previous.keys() - active.keys())
        if cleared:
            parts.append("cleared=" + ",".join(cleared))
        return "IK CONSTRAINED | " + " | ".join(parts)
