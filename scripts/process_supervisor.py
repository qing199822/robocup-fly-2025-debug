#!/usr/bin/env python3

"""Run one launcher-owned command and reap all descendants it leaves behind.

Subreaper mode is enabled before the command is spawned. If that command exits
before a detached descendant, Linux reparents the orphan to this process instead
of PID 1, so its ownership remains observable until it is signaled and reaped.
"""

import argparse
import ctypes
import errno
import os
import signal
import subprocess
import sys
import time


PR_SET_CHILD_SUBREAPER = 36
POLL_SECONDS = 0.02
KILL_WAIT_SECONDS = 0.2


def process_record(pid):
    try:
        with open(f"/proc/{pid}/stat", "r", encoding="ascii") as stat_file:
            fields = stat_file.read().rsplit(") ", 1)[1].split()
        return int(fields[1]), int(fields[19])
    except (OSError, IndexError, ValueError):
        return None


def enable_subreaper():
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(PR_SET_CHILD_SUBREAPER, 1, 0, 0, 0) != 0:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number))


class ProcessSupervisor:
    def __init__(self, command, grace_seconds):
        self.command = command
        self.grace_seconds = grace_seconds
        self.supervisor_pid = os.getpid()
        self.tracked = {}
        self.main_pid = None
        self.main_status = None
        self.requested_signal = None

    def request_stop(self, signal_number, _frame):
        if self.requested_signal is None:
            self.requested_signal = signal_number

    def start(self):
        enable_subreaper()
        for signal_number in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
            signal.signal(signal_number, self.request_stop)

        child = subprocess.Popen(self.command)
        self.main_pid = child.pid
        record = process_record(child.pid)
        if record is None:
            child.wait()
            return child.returncode
        self.tracked[child.pid] = record[1]

        while self.main_status is None and self.requested_signal is None:
            scan_deadline = time.monotonic() + POLL_SECONDS
            self.discover(scan_deadline)
            self.reap(scan_deadline)
            if self.main_status is None and self.requested_signal is None:
                time.sleep(POLL_SECONDS)

        cleanup_started = time.monotonic()
        final_deadline = cleanup_started + self.grace_seconds
        term_deadline = max(cleanup_started, final_deadline - KILL_WAIT_SECONDS)
        self.wait_until_empty(term_deadline, signal.SIGTERM)
        self.wait_until_empty(final_deadline, signal.SIGKILL)

        if self.main_status is not None:
            return self.main_status
        return 128 + (self.requested_signal or signal.SIGTERM)

    def discover(self, deadline):
        snapshot = {}
        try:
            proc_entries = os.scandir("/proc")
        except OSError:
            return
        with proc_entries:
            for entry in proc_entries:
                if time.monotonic() >= deadline:
                    return
                if not entry.name.isdigit():
                    continue
                pid = int(entry.name)
                record = process_record(pid)
                if record is not None:
                    snapshot[pid] = record

        ownership = {}

        def is_owned(pid, visiting):
            if time.monotonic() >= deadline:
                return False
            if pid == self.supervisor_pid:
                return True
            if pid in ownership:
                return ownership[pid]
            record = snapshot.get(pid)
            if record is None or pid in visiting:
                ownership[pid] = False
                return False
            expected_start = self.tracked.get(pid)
            if expected_start is not None and expected_start == record[1]:
                ownership[pid] = True
                return True
            visiting.add(pid)
            result = is_owned(record[0], visiting)
            visiting.remove(pid)
            ownership[pid] = result
            return result

        for pid, (_ppid, start_time) in snapshot.items():
            if time.monotonic() >= deadline:
                return
            if pid != self.supervisor_pid and is_owned(pid, set()):
                self.tracked[pid] = start_time

    def identity_matches(self, pid):
        record = process_record(pid)
        return record is not None and record[1] == self.tracked.get(pid)

    def has_live_tracked(self, deadline):
        for pid in self.tracked:
            if time.monotonic() >= deadline:
                return True
            if self.identity_matches(pid):
                return True
        return False

    def stop_descendants(self, signal_number, deadline):
        self.discover(deadline)
        if time.monotonic() >= deadline:
            return
        if self.main_pid in self.tracked:
            self.signal_tracked_process(self.main_pid, signal_number, deadline)
        for pid in self.tracked:
            if time.monotonic() >= deadline:
                return
            if pid == self.main_pid:
                continue
            self.signal_tracked_process(pid, signal_number, deadline)

    def signal_tracked_process(self, pid, signal_number, deadline):
        if time.monotonic() >= deadline:
            return
        if not self.identity_matches(pid):
            return
        try:
            os.kill(pid, signal_number)
        except ProcessLookupError:
            pass
        except PermissionError:
            pass

    def wait_until_empty(self, deadline, signal_number):
        self.stop_descendants(signal_number, deadline)
        while time.monotonic() < deadline:
            self.reap(deadline)
            self.discover(deadline)
            if time.monotonic() >= deadline:
                return False
            if not self.has_live_tracked(deadline):
                return True
            if signal_number == signal.SIGKILL:
                self.stop_descendants(signal_number, deadline)
            time.sleep(min(POLL_SECONDS, max(0, deadline - time.monotonic())))
        return False

    def reap(self, deadline):
        while time.monotonic() < deadline:
            try:
                pid, status = os.waitpid(-1, os.WNOHANG)
            except ChildProcessError:
                return
            except InterruptedError:
                continue
            if pid == 0:
                return
            if pid == self.main_pid:
                if os.WIFEXITED(status):
                    self.main_status = os.WEXITSTATUS(status)
                elif os.WIFSIGNALED(status):
                    self.main_status = 128 + os.WTERMSIG(status)
                else:
                    self.main_status = 1
            self.tracked.pop(pid, None)


def parse_arguments(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--grace-seconds", type=float, required=True)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    arguments = parser.parse_args(argv)
    if arguments.command[:1] == ["--"]:
        arguments.command = arguments.command[1:]
    if not arguments.command:
        parser.error("a command is required after --")
    if arguments.grace_seconds <= 0:
        parser.error("--grace-seconds must be positive")
    return arguments


def main(argv=None):
    arguments = parse_arguments(argv)
    try:
        return ProcessSupervisor(
            arguments.command, arguments.grace_seconds
        ).start()
    except OSError as error:
        if error.errno == errno.ENOENT:
            print(f"process supervisor failed to start command: {error}", file=sys.stderr)
            return 127
        raise


if __name__ == "__main__":
    sys.exit(main())
