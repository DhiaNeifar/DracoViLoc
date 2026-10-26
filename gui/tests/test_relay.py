"""Check Jetson relay SSH commands without switching the physical relay."""

import asyncio
import json
import sys
from types import SimpleNamespace

import pytest

from dracoviloc_gui.remote_client import RelayPasswordRequired, RemoteJetsonClient
from dracoviloc_gui.controller import AppController
from dracoviloc_gui.models import AppState


class FakeConnection:
    def __init__(self, sudo_result):
        self.commands = []
        self.sudo_result = sudo_result

    async def __aenter__(self):
        return self

    async def __aexit__(self, *args):
        return False

    async def run(self, command, **kwargs):
        self.commands.append((command, kwargs))
        if command.startswith("printf"):
            return SimpleNamespace(stdout="/home/akkurad", stderr="", exit_status=0)
        if command.startswith("test -f"):
            return SimpleNamespace(stdout="", stderr="", exit_status=0)
        return self.sudo_result


def fake_ssh(monkeypatch, sudo_result):
    connection = FakeConnection(sudo_result)
    aliases = []

    def connect(alias, **kwargs):
        aliases.append(alias)
        return connection

    monkeypatch.setitem(sys.modules, "asyncssh", SimpleNamespace(connect=connect))
    return connection, aliases


@pytest.mark.parametrize("action", ["on", "off"])
def test_relay_uses_oneshot_jetson_ssh_and_exact_command(monkeypatch, action):
    connection, aliases = fake_ssh(
        monkeypatch, SimpleNamespace(stdout="done", stderr="", exit_status=0))
    client = object.__new__(RemoteJetsonClient)

    message = asyncio.run(client._set_relay("jetson", action, None))

    assert aliases == ["jetson"]
    assert connection.commands[-1][0] == (
        f"sudo -n python3 /home/akkurad/relay_control.py {action} "
        "--uid 3D59C4B4 -v")
    assert "input" not in connection.commands[-1][1]
    assert "completed" in message


def test_relay_requests_password_and_sends_it_only_on_stdin(monkeypatch):
    connection, _ = fake_ssh(
        monkeypatch, SimpleNamespace(
            stdout="", stderr="sudo: a password is required", exit_status=1))
    client = object.__new__(RemoteJetsonClient)

    with pytest.raises(RelayPasswordRequired):
        asyncio.run(client._set_relay("jetson", "on", None))

    connection.sudo_result = SimpleNamespace(stdout="done", stderr="", exit_status=0)
    asyncio.run(client._set_relay("jetson", "on", "secret-value"))
    command, options = connection.commands[-1]
    assert command == (
        "sudo -S -p '' python3 /home/akkurad/relay_control.py on "
        "--uid 3D59C4B4 -v")
    assert options["input"] == "secret-value\n"
    assert "secret-value" not in command


def test_automatic_pulse_runs_for_30_seconds_on_jetson(monkeypatch):
    connection, _ = fake_ssh(
        monkeypatch, SimpleNamespace(stdout="Relay pulse completed", stderr="", exit_status=0))
    client = object.__new__(RemoteJetsonClient)
    asyncio.run(client._set_relay("jetson", "pulse", "private-test-value"))
    command, options = connection.commands[-1]
    assert command.endswith("pulse --uid 3D59C4B4 -v --pulse-seconds 30")
    assert options["timeout"] == 50
    assert options["input"] == "private-test-value\n"


def test_relay_rejects_invalid_action_without_ssh(monkeypatch):
    connection, aliases = fake_ssh(
        monkeypatch, SimpleNamespace(stdout="", stderr="", exit_status=0))
    client = object.__new__(RemoteJetsonClient)
    with pytest.raises(ValueError):
        asyncio.run(client._set_relay("jetson", "restart", None))
    assert aliases == []
    assert connection.commands == []


def test_rejected_saved_password_prompts_again(monkeypatch):
    fake_ssh(monkeypatch, SimpleNamespace(
        stdout="", stderr="Sorry, try again.\nsudo: 1 incorrect password attempt",
        exit_status=1))
    client = object.__new__(RemoteJetsonClient)
    with pytest.raises(RelayPasswordRequired):
        asyncio.run(client._set_relay("jetson", "off", "wrong-password"))


def test_saved_password_requires_private_file(monkeypatch, tmp_path):
    monkeypatch.setenv("HOME", str(tmp_path))
    path = tmp_path / ".config/dracoviloc-gui/relay-sudo-password"
    path.parent.mkdir(parents=True)
    path.write_text("test-password\n", encoding="utf-8")
    path.chmod(0o644)
    assert AppController._saved_relay_password() is None
    path.chmod(0o600)
    assert AppController._saved_relay_password() == "test-password"


def test_auto_relay_fires_once_only_for_fresh_arm2_start():
    commands = []
    controller = SimpleNamespace(
        _profile=SimpleNamespace(dual_arm=True),
        _state=AppState.TRACKING,
        _auto_relay_seen="",
        _relay_state={"busy": False, "auto_active": False, "needs_password": False},
        _log=lambda *args: None,
        _saved_relay_password=lambda: "private-test-value",
        _send_relay=lambda *args, **kwargs: commands.append((args, kwargs)),
    )
    event = {
        "arm2_started_event": "arm2_started:12345",
        "dual_arm_detected": True,
        "freshness": {"/tracking/arm2_started": 0.2},
    }
    AppController._maybe_auto_relay(controller, json.dumps(event))
    AppController._maybe_auto_relay(controller, json.dumps(event))
    assert commands == [(("pulse", "private-test-value"), {"auto": True})]
    assert controller._relay_state["auto_active"]

    controller._relay_state["auto_active"] = False
    event["arm2_started_event"] = "arm2_started:12346"
    event["freshness"]["/tracking/arm2_started"] = 3.0
    AppController._maybe_auto_relay(controller, json.dumps(event))
    assert len(commands) == 1


def test_auto_relay_marks_off_after_jetson_pulse_completes():
    controller = SimpleNamespace(
        _relay_state={"busy": True, "needs_password": False,
                      "auto_active": True, "last_command": "", "message": ""},
        _quit_after_relay=False,
        _log=lambda *args: None,
        relayChanged=SimpleNamespace(emit=lambda: None),
    )
    AppController._finish_relay(controller, "pulse", True, "Pulse complete", False, True)
    assert controller._relay_state["auto_active"] is False
    assert controller._relay_state["last_command"] == "off"
