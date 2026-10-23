from __future__ import annotations

import asyncio
import contextlib
import time
import uuid
from dataclasses import dataclass
from typing import Any

from . import AGENT_VERSION
from .config import AgentConfig
from .protocol import PROTOCOL_VERSION, ProtocolError, read_message, write_message
from .runtime import AgentRuntime


@dataclass(eq=False)
class Session:
    session_id: str
    writer: asyncio.StreamWriter
    name: str = "unknown"
    connected_at: float = 0.0


class AgentServer:
    """Loopback-only service. SSH is the authentication and encryption boundary."""

    CONTROL_COMMANDS = {"start", "home", "set_tracking", "stop_all", "force_stop_all"}

    def __init__(self, config: AgentConfig) -> None:
        self.config = config.expanded()
        self.loop: asyncio.AbstractEventLoop | None = None
        self.sessions: set[Session] = set()
        self.controller_id: str | None = None
        self._server: asyncio.AbstractServer | None = None
        self.runtime = AgentRuntime(
            self.config, self._threadsafe_status, self._threadsafe_log,
            self._threadsafe_camera)

    async def start(self) -> None:
        self.loop = asyncio.get_running_loop()
        if self.config.listen_host not in {"127.0.0.1", "::1", "localhost"}:
            raise ValueError("the remote agent may only listen on loopback")
        self.runtime.start()
        self._server = await asyncio.start_server(
            self._accept, self.config.listen_host, self.config.listen_port)

    async def serve_forever(self) -> None:
        await self.start()
        assert self._server is not None
        async with self._server:
            await self._server.serve_forever()

    async def close(self) -> None:
        if self._server:
            self._server.close()
            await self._server.wait_closed()
        self.runtime.stop()
        for session in tuple(self.sessions):
            session.writer.close()

    def _schedule(self, coroutine) -> None:
        if self.loop and not self.loop.is_closed():
            self.loop.call_soon_threadsafe(asyncio.create_task, coroutine)

    def _threadsafe_status(self, status: dict[str, Any]) -> None:
        self._schedule(self.broadcast({"type": "event", "event": "status", "data": status}))

    def _threadsafe_log(self, subsystem: str, severity: str, message: str) -> None:
        self._schedule(self.broadcast({
            "type": "event", "event": "log", "data": {
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "subsystem": subsystem, "severity": severity, "message": message,
            }}))

    def _threadsafe_camera(self, metadata: dict[str, Any], body: bytes) -> None:
        self._schedule(self.broadcast(
            {"type": "event", "event": "camera", "data": metadata}, body))

    async def broadcast(self, header: dict[str, Any], body: bytes = b"") -> None:
        for session in tuple(self.sessions):
            try:
                await write_message(session.writer, header, body)
            except (ConnectionError, asyncio.CancelledError):
                pass

    async def _accept(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        session = Session(str(uuid.uuid4()), writer, connected_at=time.time())
        try:
            hello, _ = await asyncio.wait_for(read_message(reader), timeout=8.0)
            if hello.get("type") != "hello" or hello.get("version") != PROTOCOL_VERSION:
                await write_message(writer, {
                    "type": "hello_error", "error": "protocol version mismatch",
                    "agent_version": AGENT_VERSION})
                return
            session.name = str(hello.get("client_name", "DracoViLoc GUI"))[:80]
            await write_message(writer, {
                "type": "hello", "agent_version": AGENT_VERSION,
                "session_id": session.session_id, "controller_id": self.controller_id,
                "capabilities": ["control", "status", "camera_jpeg", "recordings"],
            })
            await write_message(writer, {
                "type": "event", "event": "status", "data": self.runtime.status})
            # Do not expose a socket to broadcasts until its ordered hello and
            # initial status have been written. Otherwise a periodic status
            # broadcast can overtake the hello and make a valid connection
            # look like a failed handshake to the GUI.
            self.sessions.add(session)
            while True:
                request, _ = await read_message(reader)
                if request.get("type") != "request":
                    continue
                asyncio.create_task(self._handle(session, request))
        except (asyncio.IncompleteReadError, ConnectionError, ProtocolError, asyncio.TimeoutError):
            pass
        finally:
            self.sessions.discard(session)
            writer.close()
            with contextlib.suppress(Exception):
                await writer.wait_closed()
            if self.controller_id == session.session_id:
                await asyncio.sleep(5.0)
                if self.controller_id == session.session_id:
                    self.controller_id = None
                    await self._control_event()

    async def _control_event(self) -> None:
        await self.broadcast({
            "type": "event", "event": "control",
            "data": {"controller_id": self.controller_id}})

    async def _handle(self, session: Session, request: dict[str, Any]) -> None:
        request_id = request.get("id")
        command = request.get("command")
        args = request.get("args") or {}
        try:
            if command == "acquire_control":
                if self.controller_id in {None, session.session_id}:
                    self.controller_id = session.session_id
                    result: Any = {"controller": True}
                    await self._control_event()
                else:
                    raise PermissionError("another GUI currently owns control")
            elif command == "release_control":
                if self.controller_id == session.session_id:
                    self.controller_id = None
                    await self._control_event()
                result = {"controller": False}
            elif command in self.CONTROL_COMMANDS and self.controller_id != session.session_id:
                raise PermissionError("this client is read-only; acquire control first")
            elif command == "get_status":
                result = self.runtime.status
            elif command == "preflight":
                result = self.runtime.preflight(args.get("profile", {}))
            elif command == "start":
                ok, message = await asyncio.to_thread(
                    self.runtime.start_run, args.get("profile", {}))
                if not ok:
                    raise RuntimeError(message)
                result = {"message": message}
            elif command == "home":
                ok, message = await asyncio.to_thread(self.runtime.home)
                if not ok:
                    raise RuntimeError(message)
                result = {"message": message}
            elif command == "set_tracking":
                ok, message = await asyncio.to_thread(
                    self.runtime.set_tracking, bool(args.get("enabled")))
                if not ok:
                    raise RuntimeError(message)
                result = {"message": message}
            elif command == "stop_all":
                ok, message = await asyncio.to_thread(self.runtime.stop_all)
                if not ok:
                    raise RuntimeError(message)
                result = {"message": message}
            elif command == "force_stop_all":
                ok, message = await asyncio.to_thread(self.runtime.force_stop_all)
                if not ok:
                    raise RuntimeError(message)
                result = {"message": message}
            elif command == "list_recordings":
                result = await asyncio.to_thread(self.runtime.list_recordings)
            else:
                raise ValueError(f"unknown command: {command}")
            response = {"type": "response", "id": request_id, "ok": True, "result": result}
        except Exception as exc:
            response = {"type": "response", "id": request_id, "ok": False, "error": str(exc)}
        with contextlib.suppress(ConnectionError):
            await write_message(session.writer, response)
