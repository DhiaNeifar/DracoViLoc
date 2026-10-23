import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "remote_agent"))

from dracoviloc_agent.config import AgentConfig
from dracoviloc_agent.protocol import encode_message, read_message, write_message
from dracoviloc_agent.server import AgentServer
from dracoviloc_agent.runtime import AgentRuntime
from dracoviloc_gui.remote_client import _read_agent_hello


class FakeRuntime:
    def __init__(self):
        self.status = {"state": "Idle", "run_active": False}
        self.tracking = False

    def start(self): pass
    def stop(self): pass
    def preflight(self, _profile): return []
    def start_run(self, _profile): self.status["run_active"] = True; return True, "started"
    def home(self): return True, "home"
    def set_tracking(self, enabled): self.tracking = enabled; return True, "tracking changed"
    def stop_all(self): self.status["run_active"] = False; return True, "stopped"
    def force_stop_all(self): self.status["run_active"] = False; return True, "forced"
    def list_recordings(self): return []


def test_remote_launch_forces_dashboard_only_and_jetson_recordings(tmp_path):
    runtime = AgentRuntime(
        AgentConfig(project_root=str(tmp_path), isaac_root=str(tmp_path),
                    recordings_root=str(tmp_path / "runs")),
        lambda *_: None, lambda *_: None, lambda *_: None)
    args = runtime._host_args({
        "execution_target": "remote", "ssh_alias": "jetson",
        "use_rviz": True, "recording_root": "/laptop/runs",
        "expert_arguments": {"use_rviz": "true", "recording_root": "/override"},
    })
    assert "use_rviz:=false" in args
    assert f"recording_root:={tmp_path / 'runs'}" in args
    assert f"mobilenetv2_venv_python:={tmp_path / 'trt_env/bin/python3'}" in args
    assert not any("ssh_alias" in arg for arg in args)


async def connect(port):
    reader, writer = await asyncio.open_connection("127.0.0.1", port)
    await write_message(writer, {"type": "hello", "version": 1, "client_name": "test"})
    hello, _ = await read_message(reader)
    await read_message(reader)  # initial status event
    return reader, writer, hello


async def request(reader, writer, number, command, args=None):
    await write_message(writer, {
        "type": "request", "id": str(number), "command": command, "args": args or {}})
    while True:
        response, _ = await read_message(reader)
        if response.get("type") == "response" and response.get("id") == str(number):
            return response


def test_only_one_session_can_control():
    async def scenario():
        server = AgentServer(AgentConfig(listen_port=0))
        server.runtime = FakeRuntime()
        await server.start()
        port = server._server.sockets[0].getsockname()[1]
        r1, w1, _ = await connect(port)
        r2, w2, _ = await connect(port)
        assert (await request(r1, w1, 1, "acquire_control"))["ok"]
        denied = await request(r2, w2, 2, "acquire_control")
        assert denied["ok"] is False
        assert "another GUI" in denied["error"]
        read_only = await request(r2, w2, 3, "set_tracking", {"enabled": True})
        assert read_only["ok"] is False
        assert server.runtime.tracking is False
        w1.close(); w2.close()
        await server.close()
    asyncio.run(scenario())


def test_gui_handshake_tolerates_status_event_from_older_agent():
    async def scenario():
        reader = asyncio.StreamReader()
        reader.feed_data(encode_message({
            "type": "event", "event": "status", "data": {"state": "Idle"}}))
        reader.feed_data(encode_message({
            "type": "hello", "agent_version": "1.0.0", "session_id": "session"}))
        reader.feed_eof()
        hello = await _read_agent_hello(reader, timeout=1.0)
        assert hello["session_id"] == "session"
    asyncio.run(scenario())
