"""Opt-in Windows waitable timer; no global timer-resolution changes or busy wait."""

import ctypes
from ctypes import wintypes
import math
import sys
import threading
import time


def _load_kernel():
    if sys.platform != "win32":
        raise RuntimeError(
            "windows-highres requires Windows; use --wait-mode system on other platforms"
        )
    k = ctypes.WinDLL("kernel32", use_last_error=True)
    declarations = {
        "CreateWaitableTimerExW": (
            [ctypes.c_void_p, wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD],
            wintypes.HANDLE,
        ),
        "SetWaitableTimer": (
            [
                wintypes.HANDLE,
                ctypes.POINTER(ctypes.c_longlong),
                wintypes.LONG,
                ctypes.c_void_p,
                ctypes.c_void_p,
                wintypes.BOOL,
            ],
            wintypes.BOOL,
        ),
        "WaitForSingleObjectEx": (
            [wintypes.HANDLE, wintypes.DWORD, wintypes.BOOL],
            wintypes.DWORD,
        ),
        "CancelWaitableTimer": ([wintypes.HANDLE], wintypes.BOOL),
        "CloseHandle": ([wintypes.HANDLE], wintypes.BOOL),
    }
    for name, (args, result) in declarations.items():
        fn = getattr(k, name)
        fn.argtypes, fn.restype = args, result
    return k


class WindowsHighResolutionWait:
    def __init__(self):
        self._kernel = _load_kernel()
        self._lock = threading.RLock()
        self._handle = self._kernel.CreateWaitableTimerExW(None, None, 2, 0x100002)
        if not self._handle:
            raise ctypes.WinError(ctypes.get_last_error())

    def sleep(self, seconds):
        if not math.isfinite(seconds) or seconds < 0 or seconds > 3600:
            raise ValueError(
                "Wait duration must be finite and within [0, 3600] seconds"
            )
        with self._lock:
            if self._handle is None:
                raise RuntimeError("High-resolution wait is closed")
            if seconds == 0:
                time.sleep(0)
                return
            due = ctypes.c_longlong(-max(1, math.ceil(seconds * 10_000_000)))
            if not self._kernel.SetWaitableTimer(
                self._handle, ctypes.byref(due), 0, None, None, False
            ):
                raise ctypes.WinError(ctypes.get_last_error())
            try:
                deadline = time.monotonic() + seconds + 1.0
                while True:
                    # Return to Python at least every nominal 50 ms to check signals.
                    # Timer signals wake this wait immediately; 50 ms is not a polling sleep.
                    result = self._kernel.WaitForSingleObjectEx(self._handle, 50, False)
                    if result == 0:
                        return
                    if result != 258:
                        raise ctypes.WinError(ctypes.get_last_error())
                    if time.monotonic() >= deadline:
                        raise TimeoutError("High-resolution timer did not signal")
            except BaseException:
                self._kernel.CancelWaitableTimer(self._handle)
                raise

    def close(self):
        with self._lock:
            if self._handle is not None:
                if not self._kernel.CloseHandle(self._handle):
                    raise ctypes.WinError(ctypes.get_last_error())
                self._handle = None
