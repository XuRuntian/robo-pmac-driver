"""Terminal-only diagnostics, no controller access or control decisions."""
import math
import shutil

import numpy as np


def diagnostic_lines(result, previous_u, requested, sent, previous, geometry,
                     *, holding=False, rotation_limited=False):
    u = np.asarray(previous_u if holding else result.u)
    du = u - previous_u
    # phi is periodic; show the shortest reported coordinate change.
    for i in (2, 4):
        du[i] = math.atan2(math.sin(du[i]), math.cos(du[i]))
    if holding:
        du[:] = 0
    near = []
    lo, hi = geometry.insertion_bounds()
    if u[0]-lo <= .001:
        near.append('d.lower')
    if hi-u[0] <= .001:
        near.append('d.upper')
    for name, value, cap in zip(('theta_a', 'theta_c'), u[[1,3]], geometry.curvature_caps(float(u[0]))):
        if cap-value <= math.radians(5):
            near.append(name+'.upper')
    blocked = [] if holding or result.converged else sorted(result.limit_attempts)
    state = 'HOLD' if holding else ('OK' if result.converged else 'NO_CONVERGE')
    vec = lambda x: '[' + ','.join(f'{v:+.3f}' for v in x) + ']'
    joint = du * np.array([1000, 180/math.pi, 180/math.pi, 180/math.pi, 180/math.pi])
    scale = np.array([180/math.pi]*4+[1000])
    return [
        f'IK={state} cmd_cap={int(rotation_limited)} near={",".join(near) or "-"} blocked={",".join(blocked) or "-"}',
        'IK delta [d mm,theta_a,phi_a,theta_c,phi_c deg] '+vec(joint),
        'REQ delta [alpha1..4 deg,d mm] '+vec((np.asarray(requested)-previous)*scale),
        'SENT delta[alpha1..4 deg,d mm] '+vec((np.asarray(sent)-previous)*scale),
    ]


class LiveDiagnostic:
    def __init__(self, stream):
        self.stream = stream
        self.drawn = False

    def draw(self, lines):
        # ANSI cursor motion is for an interactive Git Bash/terminal only.
        if not self.stream.isatty():
            return
        width = max(20, shutil.get_terminal_size((120, 24)).columns-1)
        prefix = '\x1b[4A' if self.drawn else ''
        self.stream.write(prefix + ''.join('\r\x1b[2K'+line[:width]+'\n' for line in lines))
        self.stream.flush()
        self.drawn = True
