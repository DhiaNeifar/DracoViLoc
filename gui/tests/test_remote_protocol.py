import asyncio
import json
import struct

import pytest

from dracoviloc_gui.remote_protocol import (
    MAX_BODY, ProtocolError, encode_message, read_message,
)


def decode(payload: bytes):
    async def work():
        reader = asyncio.StreamReader()
        for offset in range(0, len(payload), 3):
            reader.feed_data(payload[offset:offset + 3])
        reader.feed_eof()
        return await read_message(reader)
    return asyncio.run(work())


def test_round_trip_handles_fragmented_binary_frame():
    header, body = decode(encode_message({"type": "event", "event": "camera"}, b"jpeg"))
    assert header["version"] == 1
    assert header["body_length"] == 4
    assert body == b"jpeg"


def test_invalid_json_is_rejected():
    with pytest.raises(ProtocolError, match="JSON"):
        decode(struct.pack("!I", 1) + b"{")


def test_oversize_body_is_rejected_before_allocation():
    header = json.dumps({"body_length": MAX_BODY + 1}).encode()
    with pytest.raises(ProtocolError, match="body length"):
        decode(struct.pack("!I", len(header)) + header)
