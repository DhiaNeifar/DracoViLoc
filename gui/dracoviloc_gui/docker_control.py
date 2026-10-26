"""Docker Engine API access (no CLI parsing) for the YOLO container."""
from __future__ import annotations

import warnings

with warnings.catch_warnings():
    # docker-py imports the system paramiko, which warns about the newer
    # cryptography package; SSH transport is not used here.
    warnings.simplefilter("ignore")
    import docker
    from docker.errors import DockerException, NotFound

YOLO_CONTAINER = "isaac_yolo"


def _client():
    return docker.from_env(timeout=15)


def container_state(name: str = YOLO_CONTAINER) -> str | None:
    """Return the container status ('running', 'exited', ...) or None if absent."""
    try:
        return _client().containers.get(name).status
    except NotFound:
        return None


def remove_if_stopped(name: str = YOLO_CONTAINER) -> bool:
    """Remove a leftover stopped container. Returns True if one was removed."""
    try:
        container = _client().containers.get(name)
    except NotFound:
        return False
    if container.status == "running":
        raise RuntimeError(f"Container {name} is already running")
    container.remove()
    return True


def stop(name: str = YOLO_CONTAINER, timeout_s: int = 10) -> bool:
    """Stop the container if it is still running. Returns True if it was running."""
    try:
        container = _client().containers.get(name)
    except NotFound:
        return False
    if container.status != "running":
        return False
    container.stop(timeout=timeout_s)
    return True


def image_exists(image: str) -> bool:
    try:
        _client().images.get(image)
        return True
    except NotFound:
        return False


__all__ = ["DockerException", "YOLO_CONTAINER", "container_state", "image_exists",
           "remove_if_stopped", "stop"]
