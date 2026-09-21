"""OS-level file descriptor redirection to silence C-level printf from external libraries."""

import os
import sys
from contextlib import contextmanager


@contextmanager
def suppress_c_stdout():
    """Temporarily redirect OS file descriptors 1 (stdout) and 2 (stderr) to devnull.

    Catches compiled C/C++ printf calls that bypass sys.stdout.
    """
    sys.stdout.flush()
    sys.stderr.flush()
    try:
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        old_stdout_fd = os.dup(1)
        old_stderr_fd = os.dup(2)
        try:
            os.dup2(devnull_fd, 1)
            os.dup2(devnull_fd, 2)
            yield
        finally:
            sys.stdout.flush()
            sys.stderr.flush()
            os.dup2(old_stdout_fd, 1)
            os.dup2(old_stderr_fd, 2)
            os.close(old_stdout_fd)
            os.close(old_stderr_fd)
            os.close(devnull_fd)
    except Exception:
        # If OS redirection is unavailable, yield normally
        yield
