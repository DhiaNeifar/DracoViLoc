from __future__ import annotations

import argparse
import asyncio
from pathlib import Path

from .config import AgentConfig
from .server import AgentServer


def main() -> int:
    parser = argparse.ArgumentParser(description="DracoViLoc Jetson remote agent")
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    try:
        asyncio.run(AgentServer(AgentConfig.load(args.config)).serve_forever())
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
