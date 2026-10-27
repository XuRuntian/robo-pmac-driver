import io
from types import SimpleNamespace

import numpy as np
from continuum_sdk.control.live_diagnostic import LiveDiagnostic, diagnostic_lines
from continuum_sdk.kinematics.geometry import ContinuumGeometry


def test_near_is_not_blocked_and_requested_differs_from_sent():
    result = SimpleNamespace(u=np.zeros(5), converged=True, limit_attempts={})
    lines = diagnostic_lines(result, np.zeros(5), [0,0,0,0,.002],
                             [0,0,0,0,.00008], np.zeros(5), ContinuumGeometry())
    assert 'near=d.lower' in lines[0] and 'blocked=-' in lines[0]
    assert '+2.000' in lines[2] and '+0.080' in lines[3]
    result.converged = False
    result.limit_attempts = {'d.lower': (-.001,0)}
    assert 'blocked=d.lower' in diagnostic_lines(result, np.zeros(5), [0]*5,[0]*5,
                                                  np.zeros(5),ContinuumGeometry())[0]


def test_display_reuses_four_lines_and_does_not_emit_escape_codes_to_files():
    class Terminal(io.StringIO):
        def isatty(self):
            return True
    terminal=Terminal();display=LiveDiagnostic(terminal)
    display.draw(['a']*4);display.draw(['b']*4)
    assert terminal.getvalue().count('\x1b[4A') == 1
    assert terminal.getvalue().count('\x1b[2K') == 8
    redirected=io.StringIO();LiveDiagnostic(redirected).draw(['a']*4)
    assert redirected.getvalue() == ''
