import subprocess
import sys
import time
from pathlib import Path

import psutil

from dracoviloc_gui import process_control

ROOT = Path(__file__).resolve().parents[2]


def spawn(code):
    return subprocess.Popen([sys.executable, "-c", code])


def wait_for_children(parent, count, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        children = psutil.Process(parent.pid).children(recursive=True)
        if len(children) >= count:
            return children
        time.sleep(0.05)
    raise AssertionError("children did not start")


def test_ctrl_c_is_tried_first():
    process = spawn("import time\nwhile True: time.sleep(0.1)")
    time.sleep(0.5)
    report = process_control.stop_processes([process.pid], "sleeper")
    assert report.stopped and report.steps[0].startswith("SIGINT: 1/1")
    process.wait(5)


def test_escalates_when_ctrl_c_and_sigterm_are_ignored_and_takes_children():
    # Parent ignores SIGINT/SIGTERM and has a child doing the same: only SIGKILL works.
    code = ("import signal, subprocess, sys, time\n"
            "signal.signal(signal.SIGINT, signal.SIG_IGN); signal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
            "subprocess.Popen([sys.executable, '-c', 'import signal,time\\n"
            "signal.signal(signal.SIGINT, signal.SIG_IGN); signal.signal(signal.SIGTERM, signal.SIG_IGN)\\n"
            "time.sleep(60)'])\n"
            "time.sleep(60)")
    process = spawn(code)
    child = wait_for_children(process, 1)[0]
    original = process_control.ESCALATION
    process_control.ESCALATION = tuple((signum, 0.5) for signum, _ in original)
    try:
        report = process_control.stop_processes([process.pid], "stubborn")
    finally:
        process_control.ESCALATION = original
    assert report.stopped, report.survivors
    assert [step.split(":")[0] for step in report.steps] == ["SIGINT", "SIGTERM", "SIGKILL"]
    assert not psutil.pid_exists(child.pid) or psutil.Process(child.pid).status() == psutil.STATUS_ZOMBIE
    process.wait(5)


def test_inline_shell_commands_are_not_mistaken_for_dracoviloc():
    shell = subprocess.Popen(["bash", "-c", "echo dracoviloc_bringup arm_demo.launch.py; sleep 30"])
    try:
        assert not process_control.is_dracoviloc(psutil.Process(shell.pid), ROOT)
    finally:
        shell.kill()
        shell.wait()


def test_programs_inside_the_checkout_are_recognised(tmp_path):
    script = ROOT / "gui/tests/_sleep_for_process_scan.py"
    script.write_text("import time\ntime.sleep(30)\n")
    process = subprocess.Popen([sys.executable, str(script)])
    try:
        assert process_control.is_dracoviloc(psutil.Process(process.pid), ROOT)
    finally:
        process.kill()
        process.wait()
        script.unlink()
