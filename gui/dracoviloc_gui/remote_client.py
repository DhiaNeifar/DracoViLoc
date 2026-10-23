"""SSH-tunnel client and Jetson agent deployment support."""

from __future__ import annotations

import asyncio
import json
import os
import shlex
import threading
import uuid
from pathlib import Path, PurePosixPath
from typing import Any, Callable

from .remote_protocol import AGENT_PORT, PROTOCOL_VERSION, read_message, write_message


class RemoteError(RuntimeError):
    pass


async def _read_agent_hello(reader: asyncio.StreamReader, timeout: float = 8.0) -> dict[str, Any]:
    """Read through harmless early events from agents predating the handshake fix."""
    deadline = asyncio.get_running_loop().time() + timeout
    while True:
        remaining = deadline - asyncio.get_running_loop().time()
        if remaining <= 0:
            raise TimeoutError("Jetson agent handshake timed out")
        header, _body = await asyncio.wait_for(read_message(reader), timeout=remaining)
        if header.get("type") == "hello":
            return header
        if header.get("type") == "hello_error":
            raise RemoteError(header.get("error", "agent handshake failed"))
        if header.get("type") != "event":
            raise RemoteError("agent handshake failed")


class RemoteJetsonClient:
    def __init__(
        self,
        state_callback: Callable[[dict[str, Any]], None],
        event_callback: Callable[[str, dict[str, Any], bytes], None],
        log_callback: Callable[[str, str, str], None],
    ) -> None:
        self.state_callback = state_callback
        self.event_callback = event_callback
        self.log_callback = log_callback
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._ready = threading.Event()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._ssh = None
        self._listener = None
        self._reader: asyncio.StreamReader | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._receive_task: asyncio.Task | None = None
        self._pending: dict[str, asyncio.Future] = {}
        self._write_lock: asyncio.Lock | None = None
        self._connection_lock: asyncio.Lock | None = None
        self._alias = ""
        self._port = AGENT_PORT
        self.session_id: str | None = None
        self.controller_id: str | None = None
        self._closing = False
        self._generation = 0
        self._thread.start()
        self._ready.wait(3.0)

    def _thread_main(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._write_lock = asyncio.Lock()
        self._connection_lock = asyncio.Lock()
        self._ready.set()
        self._loop.run_forever()

    def _submit(self, coroutine) -> asyncio.Future:
        if not self._loop:
            raise RemoteError("remote event loop is unavailable")
        return asyncio.run_coroutine_threadsafe(coroutine, self._loop)

    @property
    def connected(self) -> bool:
        return self._writer is not None and not self._writer.is_closing()

    @property
    def has_control(self) -> bool:
        return bool(self.session_id and self.controller_id == self.session_id)

    def connect(self, alias: str, port: int = AGENT_PORT) -> None:
        self._submit(self._begin_connect(alias.strip(), port))

    async def _begin_connect(self, alias: str, port: int) -> None:
        self._generation += 1
        generation = self._generation
        self._alias, self._port, self._closing = alias, port, False
        await self._connect(generation)

    async def _connect(self, generation: int) -> None:
        assert self._connection_lock is not None
        async with self._connection_lock:
            await self._connect_locked(generation)

    async def _connect_locked(self, generation: int) -> None:
        if generation != self._generation or self._closing:
            return
        await self._disconnect(release=False)
        if generation != self._generation or self._closing:
            return
        self.state_callback({"state": "connecting", "alias": self._alias})
        try:
            try:
                import asyncssh
            except ImportError as exc:
                raise RemoteError("AsyncSSH is not installed; reinstall the GUI dependencies") from exc
            # Omitting known_hosts intentionally uses AsyncSSH's strict default validation.
            self._ssh = await asyncssh.connect(
                self._alias, connect_timeout=8, login_timeout=8)
            if generation != self._generation or self._closing:
                await self._disconnect(release=False)
                return
            # Connecting to the dashboard may recover its lightweight agent,
            # but it must never start the host stack, YOLO, or robot motion.
            start = await asyncio.wait_for(self._ssh.run(
                "systemctl --user start dracoviloc-agent.service", check=False), timeout=12.0)
            if start.exit_status:
                detail = start.stderr.strip() or start.stdout.strip()
                raise RemoteError(
                    "Jetson agent is stopped and could not be started" +
                    (f": {detail}" if detail else ""))
            self._listener = await asyncio.wait_for(
                self._ssh.forward_local_port(
                    "127.0.0.1", 0, "127.0.0.1", self._port), timeout=8.0)
            local_port = self._listener.get_port()
            self._reader, self._writer = await asyncio.wait_for(
                asyncio.open_connection("127.0.0.1", local_port), timeout=8.0)
            await write_message(self._writer, {
                "type": "hello", "version": PROTOCOL_VERSION,
                "client_name": f"DracoViLoc GUI ({os.uname().nodename})",
            })
            hello = await _read_agent_hello(self._reader)
            if generation != self._generation or self._closing:
                await self._disconnect(release=False)
                return
            self.session_id = hello.get("session_id")
            self.controller_id = hello.get("controller_id")
            self.state_callback({
                "state": "connected", "alias": self._alias,
                "agent_version": hello.get("agent_version", "unknown"),
                "has_control": self.has_control,
            })
            self._receive_task = asyncio.create_task(self._receive(generation))
        except asyncio.IncompleteReadError:
            await self._disconnect(release=False)
            if generation == self._generation:
                message = "Jetson agent closed the connection; reconnecting"
                self.state_callback({
                    "state": "error", "alias": self._alias, "error": message})
                self.log_callback("remote", "warning", message)
        except Exception as exc:
            await self._disconnect(release=False)
            if generation == self._generation:
                self.state_callback({
                    "state": "error", "alias": self._alias, "error": str(exc)})
                self.log_callback("remote", "error", str(exc))

    async def _receive(self, generation: int) -> None:
        try:
            assert self._reader is not None
            while True:
                header, body = await read_message(self._reader)
                if header.get("type") == "response":
                    future = self._pending.pop(str(header.get("id")), None)
                    if future and not future.done():
                        if header.get("ok"):
                            future.set_result(header.get("result"))
                        else:
                            future.set_exception(RemoteError(header.get("error", "remote request failed")))
                elif header.get("type") == "event":
                    event = str(header.get("event"))
                    data = header.get("data") or {}
                    if event == "control":
                        self.controller_id = data.get("controller_id")
                        self.state_callback({
                            "state": "connected", "alias": self._alias,
                            "has_control": self.has_control,
                        })
                    self.event_callback(event, data, body)
        except (asyncio.IncompleteReadError, ConnectionError, asyncio.CancelledError):
            pass
        except Exception as exc:
            self.log_callback("remote", "error", str(exc))
        finally:
            if not self._closing and generation == self._generation:
                await self._connection_lost(generation)

    async def _connection_lost(self, generation: int) -> None:
        if generation != self._generation:
            return
        await self._disconnect(release=False, cancel_receiver=False)
        self.state_callback({
            "state": "disconnected", "alias": self._alias,
            "error": "SSH connection lost; the Jetson run was left unchanged",
        })
        for delay in (1, 2, 5, 10, 15, 30):
            if self._closing or generation != self._generation:
                return
            await asyncio.sleep(delay)
            if self._closing or generation != self._generation:
                return
            await self._connect(generation)
            if self.connected:
                self.log_callback("remote", "info", "Reconnected to the Jetson agent")
                return

    async def request(self, command: str, args: dict[str, Any] | None = None) -> Any:
        if not self.connected or not self._writer:
            raise RemoteError("Jetson is not connected")
        request_id = uuid.uuid4().hex
        future = asyncio.get_running_loop().create_future()
        self._pending[request_id] = future
        assert self._write_lock is not None
        async with self._write_lock:
            await write_message(self._writer, {
                "type": "request", "id": request_id,
                "command": command, "args": args or {},
            })
        try:
            return await asyncio.wait_for(future, timeout=90.0)
        finally:
            self._pending.pop(request_id, None)

    def call(
        self, command: str, args: dict[str, Any] | None,
        callback: Callable[[bool, Any], None],
    ) -> None:
        future = self._submit(self.request(command, args))
        def done(result) -> None:
            try:
                callback(True, result.result())
            except Exception as exc:
                callback(False, str(exc))
        future.add_done_callback(done)

    def acquire_control(self, callback: Callable[[bool, Any], None]) -> None:
        def complete(ok: bool, result: Any) -> None:
            if ok:
                self.controller_id = self.session_id
                self.state_callback({"state": "connected", "alias": self._alias, "has_control": True})
            callback(ok, result)
        self.call("acquire_control", {}, complete)

    def disconnect(self, release: bool = True) -> None:
        self._submit(self._begin_disconnect(release))

    async def _begin_disconnect(self, release: bool) -> None:
        self._generation += 1
        self._closing = True
        await self._disconnect(release=release)

    async def _disconnect(self, release: bool, cancel_receiver: bool = True) -> None:
        if release and self.connected:
            try:
                await asyncio.wait_for(self.request("release_control", {}), timeout=3.0)
            except Exception:
                pass
        current = asyncio.current_task()
        if cancel_receiver and self._receive_task and self._receive_task is not current:
            self._receive_task.cancel()
        for future in self._pending.values():
            if not future.done():
                future.set_exception(RemoteError("Jetson connection closed"))
        self._pending.clear()
        if self._writer:
            self._writer.close()
        if self._listener:
            self._listener.close()
        if self._ssh:
            self._ssh.close()
        self._reader = self._writer = self._listener = self._ssh = None
        self.session_id = self.controller_id = None

    def deploy(
        self, alias: str, source_root: Path, config: dict[str, Any],
        callback: Callable[[bool, str], None],
    ) -> None:
        future = self._submit(self._deploy(alias, source_root, config))
        future.add_done_callback(
            lambda result: callback(True, result.result()) if not result.exception()
            else callback(False, str(result.exception())))

    async def _deploy(self, alias: str, source_root: Path, config: dict[str, Any]) -> str:
        try:
            import asyncssh
        except ImportError as exc:
            raise RemoteError("AsyncSSH is not installed; reinstall the GUI dependencies") from exc
        async with asyncssh.connect(alias) as connection:
            home_result = await connection.run("printf %s \"$HOME\"", check=True)
            home = PurePosixPath(home_result.stdout)
            active = await connection.run(
                "systemctl --user is-active dracoviloc-host.service", check=False)
            if active.exit_status == 0:
                raise RemoteError("stop the active Jetson run before updating the agent")
            version = "1.0.0"
            install_dir = home / ".local/share/dracoviloc-agent" / version
            config_dir = home / ".config/dracoviloc-agent"
            service_dir = home / ".config/systemd/user"
            recordings = str(config.get("recordings_root", "~/DracoViLoc/runs"))
            if recordings.startswith("~/"):
                recordings = str(home / recordings[2:])
            await connection.run(
                "mkdir -p " + " ".join(shlex.quote(str(path)) for path in (
                    install_dir / "dracoviloc_agent", config_dir, service_dir,
                    PurePosixPath(recordings))), check=True)
            async with connection.start_sftp_client() as sftp:
                for local in (source_root / "dracoviloc_agent").glob("*.py"):
                    await sftp.put(str(local), str(install_dir / "dracoviloc_agent" / local.name))
                remote_config = config_dir / "config.json"
                async with sftp.open(str(remote_config), "w") as handle:
                    await handle.write(json.dumps(config, indent=2))
                project_root = str(config.get("project_root", "~/DracoViLoc"))
                if project_root.startswith("~/"):
                    project_root = str(home / project_root[2:])
                setup = PurePosixPath(project_root) / "install/setup.bash"
                service = (
                    "[Unit]\nDescription=DracoViLoc remote-control agent\nAfter=network.target\n\n"
                    "[Service]\nType=simple\nRestart=on-failure\nRestartSec=2\n"
                    "TimeoutStopSec=8\nKillMode=control-group\n"
                    f"Environment=PYTHONPATH={install_dir}\n"
                    "Environment=ROS_DOMAIN_ID=42\n"
                    "Environment=FASTDDS_BUILTIN_TRANSPORTS=UDPv4\n"
                    "Environment=CUDA_MODULE_LOADING=LAZY\n"
                    f"ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source {setup} && exec /usr/bin/python3 "
                    f"-m dracoviloc_agent.main --config {remote_config}'\n\n"
                    "[Install]\nWantedBy=default.target\n")
                async with sftp.open(str(service_dir / "dracoviloc-agent.service"), "w") as handle:
                    await handle.write(service)
            result = await connection.run(
                "systemctl --user daemon-reload && "
                "systemctl --user enable dracoviloc-agent.service && "
                "systemctl --user restart dracoviloc-agent.service",
                check=False)
            if result.exit_status:
                raise RemoteError(result.stderr.strip() or "could not start the user service")
            linger = await connection.run("loginctl show-user \"$USER\" -p Linger --value", check=False)
            warning = ""
            if linger.stdout.strip().lower() != "yes":
                warning = " Warning: enable persistence once with: sudo loginctl enable-linger $USER"
            return "Jetson agent deployed and started." + warning

    def download_recording(
        self, remote_path: str, destination_root: Path,
        callback: Callable[[bool, str], None],
    ) -> None:
        future = self._submit(self._download_recording(remote_path, destination_root))
        future.add_done_callback(
            lambda result: callback(True, result.result()) if not result.exception()
            else callback(False, str(result.exception())))

    async def _download_recording(self, remote_path: str, destination_root: Path) -> str:
        if not self._ssh:
            raise RemoteError("Jetson is not connected")
        destination_root.mkdir(parents=True, exist_ok=True)
        final = destination_root / Path(remote_path).name
        partial = destination_root / (final.name + ".part")
        if final.exists() or partial.exists():
            raise RemoteError(f"download target already exists: {final}")
        partial.mkdir()
        expected_size = 0
        try:
            import asyncssh
            async with self._ssh.start_sftp_client() as sftp:
                async def copy_tree(remote_dir: PurePosixPath, local_dir: Path) -> None:
                    nonlocal expected_size
                    async for entry in sftp.scandir(str(remote_dir)):
                        name = entry.filename.decode() if isinstance(entry.filename, bytes) else entry.filename
                        if name in {".", ".."}:
                            continue
                        remote_item = remote_dir / name
                        local_item = local_dir / name
                        if entry.attrs.type == asyncssh.FILEXFER_TYPE_DIRECTORY:
                            local_item.mkdir()
                            await copy_tree(remote_item, local_item)
                        elif entry.attrs.type == asyncssh.FILEXFER_TYPE_REGULAR:
                            expected_size += int(entry.attrs.size or 0)
                            await sftp.get(str(remote_item), str(local_item))
                await copy_tree(PurePosixPath(remote_path), partial)
            actual_size = sum(path.stat().st_size for path in partial.rglob("*") if path.is_file())
            if actual_size != expected_size:
                raise RemoteError(
                    f"download verification failed ({actual_size} of {expected_size} bytes)")
            partial.rename(final)
            return str(final)
        except Exception:
            # Keep the .part directory for diagnosis or an operator-approved retry.
            raise
