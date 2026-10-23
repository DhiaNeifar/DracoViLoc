"""Small dependency-free framed protocol used by the Jetson agent."""

from __future__ import annotations

import asyncio
import json
import struct
from typing import Any

PROTOCOL_VERSION = 1
AGENT_PORT = 48721
MAX_HEADER = 1024 * 1024
MAX_BODY = 8 * 1024 * 1024


class ProtocolError(RuntimeError):
    pass


def encode_message(header: dict[str, Any], body: bytes = b"") -> bytes:
    envelope = dict(header)
    envelope.setdefault("version", PROTOCOL_VERSION)
    envelope["body_length"] = len(body)
    encoded = json.dumps(envelope, separators=(",", ":")).encode("utf-8")
    if len(encoded) > MAX_HEADER or len(body) > MAX_BODY:
        raise ProtocolError("remote message exceeds size limit")
    return struct.pack("!I", len(encoded)) + encoded + body


async def read_message(reader: asyncio.StreamReader) -> tuple[dict[str, Any], bytes]:
    raw_length = await reader.readexactly(4)
    header_length = struct.unpack("!I", raw_length)[0]
    if not 0 < header_length <= MAX_HEADER:
        raise ProtocolError("invalid header length")
    try:
        header = json.loads((await reader.readexactly(header_length)).decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError("invalid JSON header") from exc
    body_length = header.get("body_length", 0)
    if not isinstance(body_length, int) or not 0 <= body_length <= MAX_BODY:
        raise ProtocolError("invalid body length")
    return header, await reader.readexactly(body_length) if body_length else b""


async def write_message(writer: asyncio.StreamWriter, header: dict[str, Any], body: bytes = b"") -> None:
    writer.write(encode_message(header, body))
    await writer.drain()
