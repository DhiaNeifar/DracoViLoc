"""Find and reliably stop DracoViLoc processes and containers.

Covers three kinds of things:

* processes inside the control center's systemd units (host stack, YOLO,
  RViz, ...), listed per unit so one child (e.g. rviz2) can be stopped alone;
* stray processes started from a terminal (``ros2 launch ...``, rviz2,
  ODAS, ...) that the GUI did not start;
* Docker containers (the YOLO container, build containers, run_dev.sh).

Stopping always escalates and verifies: SIGINT (like Ctrl+C), then SIGTERM,
then SIGKILL, each followed by a wait; descendants are included so killing a
launch never orphans its nodes. Whatever is still alive afterwards is
reported with the reason (zombie waiting for its parent, or stuck in a driver
in uninterruptible sleep), which a signal cannot fix.
"""
from __future__ import annotations

import os
import re
import signal
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import psutil

from . import docker_control

# Command lines that belong to DracoViLoc (same rules as scripts/stop_dracoviloc.sh).
PROJECT_PATTERNS = (
    r"/workspaces/isaac_ros-dev", r"dracoviloc_", r"arm_demo\.launch\.py",
    r"isaac_ros_yolo_direction", r"yolo_camera\.launch\.py", r"odas_ros",
    r"odas_(core|server|visualization)_node", r"logged_exec\.py",
)
# Generic ROS executables: only ours if they run from this workspace.
GENERIC_ROS = re.compile(
    r"/opt/ros/[^/]+/.*/(robot_state_publisher|move_group|rviz2|static_transform_publisher|"
    r"ros2_control_node|component_container\S*|spawner)\b|\brviz2\b|ros2 bag record")
CONTAINER_NAMES = re.compile(r"^(isaac_yolo|dracoviloc_.*|isaac_ros_dev-.*-container)$")
ESCALATION = ((signal.SIGINT, 6.0), (signal.SIGTERM, 4.0), (signal.SIGKILL, 3.0))


@dataclass
class Item:
    id: str                 # "unit:host", "pid:1234", "container:isaac_yolo"
    kind: str               # unit | process | container
    name: str
    pid: int = 0
    unit: str = ""          # owning control-center unit, if any
    managed: bool = False   # started by the control center
    state: str = ""
    memory_mb: float = 0.0
    started: float = 0.0
    command: str = ""
    children: list[dict] = field(default_factory=list)

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class StopReport:
    target: str
    stopped: bool
    steps: list[str]
    survivors: list[str]

    def to_dict(self) -> dict:
        return asdict(self)

    def summary(self) -> str:
        if self.stopped:
            return f"{self.target}: stopped ({'; '.join(self.steps)})"
        return f"{self.target}: STILL RUNNING after {', '.join(self.steps)} - {'; '.join(self.survivors)}"


STATES = {"R": "running", "S": "sleeping", "D": "disk-sleep", "Z": "zombie", "T": "stopped",
          "t": "tracing-stop", "I": "idle", "X": "dead"}


def _process_table() -> dict[int, dict]:
    """pid -> {pid, ppid, name, state, cmdline, uid} from two small /proc reads
    per process. Much cheaper than psutil.process_iter on a busy Jetson."""
    table = {}
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            uid = os.stat(f"/proc/{entry}").st_uid
            with open(f"/proc/{entry}/stat", "rb") as stream:
                stat = stream.read().decode(errors="replace")
            with open(f"/proc/{entry}/cmdline", "rb") as stream:
                cmdline = [part.decode(errors="replace") for part in stream.read().split(b"\0") if part]
        except OSError:
            continue  # exited while we looked
        name = stat[stat.find("(") + 1:stat.rfind(")")]
        fields = stat[stat.rfind(")") + 2:].split()
        table[int(entry)] = {"pid": int(entry), "ppid": int(fields[1]), "name": name,
                             "status": STATES.get(fields[0], fields[0]), "cmdline": cmdline, "uid": uid}
    return table


def _describe(info: dict) -> tuple[str, str]:
    cmdline = info.get("cmdline") or []
    name = info.get("name") or ""
    command = " ".join(cmdline) or name
    arguments = list(cmdline)
    # Interpreters: name the script they run, e.g. "ros2 launch pkg file".
    if arguments and Path(arguments[0]).name in {"python3", "python", "bash", "sh"}:
        script = next((i for i, part in enumerate(arguments[1:], 1) if not part.startswith("-")), None)
        arguments = arguments[script:] if script else arguments
    executable = Path(arguments[0]).name if arguments else name
    if executable == "ros2":
        executable = " ".join([executable, *arguments[1:4]])
    return executable[:80], command


def _in_container(pid: int) -> bool:
    """Processes inside Docker containers appear in /proc too (and, with the
    host user mapping, as this user); they are handled as their container."""
    try:
        with open(f"/proc/{pid}/cgroup", encoding="utf-8", errors="replace") as stream:
            groups = stream.read()
    except OSError:
        return False
    return "docker" in groups or "containerd" in groups


def _uses_workspace(pid: int, project_root: Path) -> bool:
    try:
        environment = psutil.Process(pid).environ()
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return False
    return str(project_root / "install") in environment.get("COLCON_PREFIX_PATH", "")


def is_dracoviloc(process, project_root: Path) -> bool:
    """process: a psutil.Process or a process_iter info dict."""
    if isinstance(process, dict):
        pid, cmdline = process["pid"], process.get("cmdline") or []
    else:
        try:
            pid, cmdline = process.pid, process.cmdline()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return False
    if not cmdline:
        return False
    # Shells/interpreters running inline code (bash -c "...", python -c "...")
    # may merely mention these words; the programs they start are matched.
    if len(cmdline) > 1 and re.fullmatch(r"-[a-z]*c", cmdline[1]):
        return False
    # The program (or the script an interpreter runs) lives in this checkout.
    # Matching the path anywhere would also catch shells that merely cd here.
    if any(part.startswith(str(project_root) + "/") for part in cmdline[:2]):
        return True
    command = " ".join(cmdline)
    if any(re.search(pattern, command) for pattern in PROJECT_PATTERNS):
        return True
    return bool(GENERIC_ROS.search(command)) and _uses_workspace(pid, project_root)


def _item(info: dict, **extra) -> Item:
    name, command = _describe(info)
    memory_mb, started = 0.0, 0.0
    try:  # only for the few processes that are shown
        process = psutil.Process(info["pid"])
        memory_mb, started = process.memory_info().rss / 2**20, process.create_time()
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        pass
    return Item(id=f"pid:{info['pid']}", kind="process", name=name, pid=info["pid"],
                state=info.get("status") or "", memory_mb=memory_mb, started=started,
                command=command[:400], **extra)


def scan(project_root: Path, units: list[dict], unit_processes) -> list[Item]:
    """units: UnitSupervisor.snapshot(); unit_processes(unit) -> [pid, ...].

    Reads the process table once (a loaded Jetson makes every /proc walk
    expensive) and builds the process trees from that snapshot."""
    table = _process_table()
    children: dict[int, list[int]] = {}
    for pid, info in table.items():
        children.setdefault(info["ppid"], []).append(pid)

    def descendants(pid: int) -> list[int]:
        found, stack = [], list(children.get(pid, []))
        while stack:
            child = stack.pop()
            found.append(child)
            stack.extend(children.get(child, []))
        return found

    me = os.getpid()
    claimed: set[int] = {me, *descendants(me)}
    parent = table.get(me, {}).get("ppid")
    while parent:  # the GUI's own ancestors (terminal, session)
        claimed.add(parent)
        parent = table.get(parent, {}).get("ppid")

    items: list[Item] = []
    for unit in units:
        if unit.get("state") not in {"active", "activating", "deactivating"}:
            continue
        pids = unit_processes(unit["unit"])
        claimed.update(pids)
        items.append(Item(id=f"unit:{unit['name']}", kind="unit", name=unit["name"],
                          pid=unit.get("pid", 0), unit=unit["unit"], managed=True,
                          state=unit.get("state", ""),
                          command=" ".join(unit.get("command", []))[:400],
                          children=[_item(table[pid], unit=unit["name"], managed=True).to_dict()
                                    for pid in pids if pid in table]))
    user = os.getuid()
    for pid, info in table.items():
        if pid in claimed or info["uid"] != user or not is_dracoviloc(info, project_root):
            continue
        if _in_container(pid):
            continue
        parent_info = table.get(info["ppid"])
        # Report the top of each stray tree once; its children go with it.
        if parent_info and info["ppid"] not in claimed and is_dracoviloc(parent_info, project_root):
            continue
        item = _item(info)
        item.children = [_item(table[child]).to_dict() for child in descendants(pid) if child in table]
        items.append(item)
    try:
        for container in docker_control.running_containers():
            if CONTAINER_NAMES.match(container["name"]):
                items.append(Item(id=f"container:{container['name']}", kind="container",
                                  name=container["name"], state=container["status"],
                                  command=container["image"],
                                  managed=container["name"] in {"isaac_yolo", "dracoviloc_isaac_build",
                                                                "dracoviloc_yolo_export", "dracoviloc_yolo_plan"}))
    except docker_control.DockerException:
        pass
    return items


def _why_alive(process: psutil.Process) -> str:
    try:
        status = process.status()
        name = process.name()
    except psutil.NoSuchProcess:
        return ""
    if status == psutil.STATUS_ZOMBIE:
        parent = process.parent()
        return (f"{name} ({process.pid}) is a zombie: it has exited but its parent "
                f"{parent.name() if parent else '?'} ({parent.pid if parent else '?'}) has not collected it; "
                "stopping the parent clears it")
    if status == psutil.STATUS_DISK_SLEEP:
        return (f"{name} ({process.pid}) is stuck in uninterruptible sleep (inside a driver, often the "
                "USB camera or audio device); signals cannot reach it: unplug the device or reboot")
    return f"{name} ({process.pid}) ignored SIGKILL ({status}); this needs a reboot"


def stop_processes(pids: list[int], label: str) -> StopReport:
    """Escalate SIGINT -> SIGTERM -> SIGKILL on the processes and all their descendants."""
    targets: dict[int, psutil.Process] = {}
    for pid in pids:
        try:
            process = psutil.Process(pid)
            targets[pid] = process
            for child in process.children(recursive=True):
                targets[child.pid] = child
        except psutil.NoSuchProcess:
            continue
    steps: list[str] = []
    alive = list(targets.values())
    zombies: list[psutil.Process] = []
    for signum, wait_s in ESCALATION:
        if not alive:
            break
        for process in alive:
            try:
                process.send_signal(signum)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        _, alive = psutil.wait_procs(alive, timeout=wait_s)
        still_running = []
        for process in alive:
            try:
                # A zombie has already exited; only its parent can clear it.
                (zombies if process.status() == psutil.STATUS_ZOMBIE else still_running).append(process)
            except psutil.NoSuchProcess:
                pass
        alive = still_running
        steps.append(f"{signal.Signals(signum).name}: "
                     f"{len(targets) - len(alive) - len(zombies)}/{len(targets)} gone")
    survivors = [reason for process in alive + zombies
                 if process.is_running() and (reason := _why_alive(process))]
    return StopReport(label, not survivors, steps, survivors)


def stop_processes_now(pids: list[int], label: str) -> StopReport:
    """Immediate SIGKILL of the processes and their descendants (no grace period)."""
    targets: list[psutil.Process] = []
    for pid in pids:
        try:
            process = psutil.Process(pid)
            targets += [process, *process.children(recursive=True)]
        except psutil.NoSuchProcess:
            continue
    for process in targets:
        try:
            process.kill()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    _, alive = psutil.wait_procs(targets, timeout=3.0)
    survivors = [reason for process in alive if process.is_running() and (reason := _why_alive(process))]
    return StopReport(label, not survivors, [f"SIGKILL: {len(targets) - len(alive)}/{len(targets)} gone"],
                      survivors)


def check_robot(ip: str, port: int = 20003, timeout_s: float = 1.5) -> dict:
    """Reachability of a FAIRINO controller: ICMP ping, then a TCP connection to
    its XML-RPC port (stateless, so it never disturbs an active session)."""
    import socket
    import subprocess
    try:
        ping = subprocess.run(["ping", "-c", "1", "-W", "1", ip], capture_output=True, timeout=3.0)
        reachable = ping.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        reachable = False
    try:
        with socket.create_connection((ip, port), timeout=timeout_s):
            port_open = True
    except OSError:
        port_open = False
    if port_open:
        return {"ok": True, "state": "ok", "detail": f"controller answers on port {port}"}
    if reachable:
        return {"ok": False, "state": "warning",
                "detail": f"pings, but port {port} is closed: controller software not ready?"}
    return {"ok": False, "state": "error", "detail": "no answer: check cable, power and IP address"}


def stop_container(name: str) -> StopReport:
    steps = []
    try:
        if docker_control.stop(name, timeout_s=10):
            steps.append("docker stop (10 s)")
        if docker_control.container_state(name) == "running":
            docker_control.kill(name)
            steps.append("docker kill")
            time.sleep(1.0)
        state = docker_control.container_state(name)
    except docker_control.DockerException as error:
        return StopReport(name, False, steps, [f"Docker error: {error}"])
    if state == "running":
        return StopReport(name, False, steps, [f"{name} is still running after docker kill"])
    return StopReport(name, True, steps or ["was not running"], [])
