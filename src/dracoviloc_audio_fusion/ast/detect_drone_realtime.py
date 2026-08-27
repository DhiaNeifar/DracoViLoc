#!/usr/bin/env python3
"""Détection de drone en temps réel via microphone et moteur TensorRT.

Supporte les microphones simples et les matrices multi-micros (ex. 16 micros).
Modes de canaux (--channel) :
    - all  : Analyse tous les canaux actifs de la matrice et rapporte le micro crête (par défaut)
    - mix  : Moyenne (downmix) de tous les canaux vers un signal composite
    - auto : Analyse dynamique du canal le plus fort à chaque fenêtre
    - 0..N-1 : Canal fixe spécifique

Exemples:
    python3 detect_drone_realtime.py --list-devices
    python3 detect_drone_realtime.py --channel all
    python3 detect_drone_realtime.py --device 2 --channel auto
    python3 detect_drone_realtime.py --device "Microphone Array" --channel 0
"""
from __future__ import annotations

import argparse
import inspect
import json
import queue
import sys
import time
from collections import deque
from fractions import Fraction
from pathlib import Path

import numpy as np
import sounddevice as sd

# pycuda, tensorrt, transformers et scipy sont importés à la demande dans
# main()/TRTEngine afin que --help et --list-devices fonctionnent sans CUDA.

# Paramètres par défaut
WINDOW_SECONDS = 1.0
HOP_SECONDS = 0.5
THRESHOLD = 0.5
CONSECUTIVE_WINDOWS = 3
SILENCE_DBFS = -55.0

# Couleurs ANSI
RESET = "\033[0m"
BOLD = "\033[1m"
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
DIM = "\033[2m"


class TRTEngine:
    def __init__(self, engine_path: str):
        import pycuda.driver as cuda
        import tensorrt as trt

        self._cuda = cuda  # conservé pour infer()

        with open(engine_path, "rb") as f, \
                trt.Runtime(trt.Logger(trt.Logger.WARNING)) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()

        self.input_name = self.engine.get_tensor_name(0)
        self.output_name = self.engine.get_tensor_name(1)
        self.input_shape = tuple(self.engine.get_tensor_shape(self.input_name))

        self.context.set_input_shape(self.input_name, self.input_shape)
        self.output_shape = tuple(self.context.get_tensor_shape(self.output_name))

        self.d_input = cuda.mem_alloc(int(np.prod(self.input_shape)) * 4)
        self.d_output = cuda.mem_alloc(int(np.prod(self.output_shape)) * 4)
        self.context.set_tensor_address(self.input_name, int(self.d_input))
        self.context.set_tensor_address(self.output_name, int(self.d_output))

    def infer(self, input_array: np.ndarray) -> np.ndarray:
        cuda = self._cuda
        output = np.empty(self.output_shape, dtype=np.float32)
        cuda.memcpy_htod_async(self.d_input, np.ascontiguousarray(input_array), self.stream)
        self.context.execute_async_v3(self.stream.handle)
        cuda.memcpy_dtoh_async(output, self.d_output, self.stream)
        self.stream.synchronize()
        return output


def find_drone_class(id2label: dict) -> int:
    for index, label in id2label.items():
        normalized = str(label).lower().replace("-", "_").replace(" ", "_")
        if "drone" in normalized and not normalized.startswith(("no", "non")):
            return int(index)
    raise ValueError(f"Aucun label 'drone' trouvé : {id2label}")


def dbfs(audio: np.ndarray) -> float:
    rms = float(np.sqrt(np.mean(np.square(audio, dtype=np.float64)) + 1e-12))
    return 20.0 * np.log10(rms)


def verify_model(engine: TRTEngine, extractor, sample_rate: int,
                 drone_class: int) -> None:
    """Vérifie que le moteur TensorRT est correctement chargé :
    forme des tenseurs, logits finis, softmax valide, et sorties qui
    réagissent réellement au signal d'entrée."""
    t = np.arange(sample_rate, dtype=np.float32) / sample_rate
    tone = (0.05 * np.sin(2 * np.pi * 440 * t)
            + 0.03 * np.sin(2 * np.pi * 2500 * t)).astype(np.float32)
    signals = {
        "silence": np.zeros(sample_rate, dtype=np.float32),
        "tonalité": tone,
    }
    scores = {}
    for name, signal in signals.items():
        feats = extractor(signal, sampling_rate=sample_rate, return_tensors="np")
        input_values = feats["input_values"].astype(np.float32)

        if tuple(input_values.shape) != tuple(engine.input_shape):
            raise RuntimeError(
                f"Forme d'entrée incompatible : features {tuple(input_values.shape)} "
                f"vs moteur {tuple(engine.input_shape)}."
            )
        logits = engine.infer(input_values)
        if not np.all(np.isfinite(logits)):
            raise RuntimeError(f"Logits non finis sur « {name} » : {logits}")

        logits_shifted = logits - np.max(logits)
        exp_logits = np.exp(logits_shifted)
        probs = exp_logits / np.sum(exp_logits)
        if abs(float(np.sum(probs)) - 1.0) > 1e-3:
            raise RuntimeError(f"Softmax invalide sur « {name} » : somme = "
                               f"{float(np.sum(probs)):.4f}")
        scores[name] = float(probs[0, drone_class])
        print(f"   - {name:<12} -> logits OK, P(drone) = {scores[name] * 100:.1f}%")

    if abs(scores["silence"] - scores["tonalité"]) < 1e-6:
        raise RuntimeError("Le moteur renvoie des sorties identiques quel que soit "
                           "l'entrée : modèle probablement mal chargé.")


def bar(value: float, width: int = 20) -> str:
    """Barre de progression texte pour une valeur entre 0 et 1."""
    filled = int(round(max(0.0, min(1.0, value)) * width))
    return "█" * filled + "░" * (width - filled)


# ---------------------------------------------------------------------------
# Sélection du périphérique audio
# ---------------------------------------------------------------------------

def input_devices() -> list[dict]:
    """Retourne la liste des périphériques ayant au moins un canal d'entrée."""
    devices = sd.query_devices()
    return [dict(d, index=i) for i, d in enumerate(devices) if d["max_input_channels"] > 0]


PROBE_MAX_CHANNELS = 16
PROBE_MIN_DBFS = -90.0


def probe_device(index: int, duration: float = 0.4) -> dict:
    """Teste un périphérique : capture d'un court extrait et mesure par canal."""
    info = sd.query_devices(index)
    n_channels = min(int(info["max_input_channels"]), PROBE_MAX_CHANNELS)
    rate = int(info["default_samplerate"])
    result = {"ok": False, "error": None, "channels": n_channels,
              "levels": [], "active_channels": 0, "rate": rate}
    try:
        data = sd.rec(int(duration * rate), samplerate=rate, channels=n_channels,
                      dtype="float32", device=index, blocking=True)
    except Exception as e:
        result["error"] = str(e).splitlines()[0] if str(e) else type(e).__name__
        return result

    if data.size == 0:
        result["error"] = "aucune donnée captée"
        return result

    levels = [dbfs(data[:, c]) for c in range(data.shape[1])]
    result["levels"] = levels
    result["active_channels"] = sum(1 for lv in levels if lv > PROBE_MIN_DBFS)
    result["ok"] = result["active_channels"] > 0
    if not result["ok"]:
        result["error"] = f"silence total ({max(levels):.1f} dBFS max)"
    return result


def working_devices(probe: bool = True, duration: float = 0.4,
                    verbose: bool = True) -> list[dict]:
    """Retourne uniquement les périphériques d'entrée qui captent réellement du son."""
    devices = input_devices()
    working, skipped = [], 0
    for d in devices:
        if verbose:
            print(f"\r  Test du périphérique [{d['index']:2d}] "
                  f"{d['name'][:45]:<45}...", end="", flush=True)
        r = probe_device(d["index"], duration) if probe else {"ok": True}
        if r["ok"]:
            working.append(dict(d, probe=r))
        else:
            skipped += 1
    if verbose:
        print(f"\r{' ' * 80}\r", end="")
        if skipped:
            print(f"{DIM}({skipped} périphérique(s) ignoré(s) : "
                  f"erreur d'ouverture ou silence total){RESET}")
    return working


def list_input_devices(probe: bool = False) -> None:
    """Liste les périphériques d'entrée."""
    if not probe:
        devices = input_devices()
        if not devices:
            print("Aucun périphérique d'entrée trouvé.")
            return
        default_in = sd.default.device[0]
        print(f"{BOLD}Périphériques d'entrée disponibles :{RESET}\n")
        for d in devices:
            marker = f" {GREEN}(défaut){RESET}" if d["index"] == default_in else ""
            print(
                f"  [{d['index']:2d}] {d['name']}{marker}\n"
                f"        canaux d'entrée : {d['max_input_channels']} | "
                f"taux natif : {int(d['default_samplerate'])} Hz"
            )
        print("\nOptions de canaux : --channel all (tous), mix (moyenne), auto (crête), ou 0..N-1."
              "\nAjoutez --test-mics pour ne lister que les micros qui captent du son.")
        return

    print(f"{BOLD}Test des microphones (seuls ceux qui captent du son "
          f"sont listés)...{RESET}")
    devices = working_devices()
    if not devices:
        print("Aucun microphone fonctionnel trouvé.")
        return
    default_in = sd.default.device[0]
    print(f"\n{BOLD}Microphones fonctionnels :{RESET}\n")
    for d in devices:
        marker = f" {GREEN}(défaut){RESET}" if d["index"] == default_in else ""
        r = d["probe"]
        levels = r["levels"]
        best = max(levels) if levels else -120.0
        active = f"{r['active_channels']}/{r['channels']} canaux actifs"
        print(
            f"  [{d['index']:2d}] {d['name']}{marker}\n"
            f"        {d['max_input_channels']} canaux | "
            f"taux natif : {int(d['default_samplerate'])} Hz | "
            f"{GREEN}{active}{RESET} | niveau max : {best:.1f} dBFS"
        )
        if 1 < len(levels) <= PROBE_MAX_CHANNELS:
            detail = " ".join(
                f"{i}:{GREEN if lv > PROBE_MIN_DBFS else RED}{lv:.0f}{RESET}"
                for i, lv in enumerate(levels)
            )
            print(f"        canaux (dBFS) : {detail}")
    print("\nOptions de canaux : --channel all (tous), mix (moyenne), auto (crête), ou 0..N-1.")


def resolve_device(arg: str | None) -> int:
    """Résout l'ID du périphérique : argument, choix interactif, ou défaut."""
    devices = input_devices()
    if not devices:
        raise RuntimeError("Aucun périphérique d'entrée audio disponible.")

    if arg is not None:
        if arg.isdigit():
            index = int(arg)
            match = next((d for d in devices if d["index"] == index), None)
            if match is None:
                raise RuntimeError(f"Périphérique #{index} introuvable ou sans entrée. "
                                   f"Voir --list-devices.")
            return index
        matches = [d for d in devices if arg.lower() in d["name"].lower()]
        if not matches:
            raise RuntimeError(f"Aucun périphérique ne correspond à « {arg} ». "
                               f"Voir --list-devices.")
        if len(matches) > 1:
            names = ", ".join(f"[{d['index']}] {d['name']}" for d in matches)
            raise RuntimeError(f"« {arg} » est ambigu : {names}")
        return matches[0]["index"]

    if sys.stdin.isatty():
        list_input_devices()
        raw = input(f"\nNuméro du périphérique [{sd.default.device[0]}] : ").strip()
        if raw:
            return resolve_device(raw)
    default = sd.default.device[0]
    if default is None or default < 0:
        return devices[0]["index"]
    return default


def resolve_channel(device_index: int, arg: str | None) -> str | int:
    """Résout le mode/canal à utiliser :
    - 'all'  : tous les canaux avec détection séquentielle & spatiale (défaut pour matrices)
    - 'mix'  : moyenne de tous les canaux vers mono
    - 'auto' : sélection dynamique du canal le plus fort par fenêtre
    - int (0..N-1) : canal individuel fixe
    """
    info = sd.query_devices(device_index)
    n_channels = int(info["max_input_channels"])

    if arg is not None:
        arg_clean = str(arg).strip().lower()
        if arg_clean in ("all", "mix", "auto"):
            return arg_clean
        if arg_clean.isdigit():
            ch = int(arg_clean)
            if not 0 <= ch < n_channels:
                raise RuntimeError(
                    f"Canal {ch} invalide : « {info['name']} » a {n_channels} "
                    f"canal/canaux (0..{n_channels - 1})."
                )
            return ch
        raise RuntimeError(
            f"Option de canal « {arg} » invalide. Choisissez 'all', 'mix', 'auto' ou un index (0..{n_channels - 1})."
        )

    if n_channels == 1:
        return 0

    if sys.stdin.isatty():
        print(f"\n« {info['name']} » possède {n_channels} canaux d'entrée. "
              f"Mesure des canaux...")
        r = probe_device(device_index)
        if r["ok"]:
            detail = " ".join(
                f"{i}:{GREEN if lv > PROBE_MIN_DBFS else RED}{lv:.0f}{RESET}"
                for i, lv in enumerate(r["levels"])
            )
            print(f"  niveaux par canal (dBFS) : {detail}")
            best = int(np.argmax(r["levels"]))
            print(f"  {GREEN}{r['active_channels']}/{r['channels']} canaux actifs{RESET} "
                  f"(canal le plus fort : {best})")
        else:
            print(f"  {YELLOW}aucun signal détecté sur les canaux testés.{RESET}")

        print(f"  Modes possibles :\n"
              f"    - 'all'  : Analyse tous les micros indépendamment (recommandé)\n"
              f"    - 'mix'  : Moyenne tous les canaux vers mono\n"
              f"    - 'auto' : Analyse le micro le plus fort en temps réel\n"
              f"    - 0..{n_channels - 1}  : Sélection d'un canal unique")
        raw = input(f"Mode/canal à utiliser [all] : ").strip()
        if raw:
            return resolve_channel(device_index, raw)
        return "all"

    return "all"


# ---------------------------------------------------------------------------
# Affichage temps réel
# ---------------------------------------------------------------------------

def print_status(level_db: float, score: float, threshold: float,
                 windows: int, latency_ms: float, peak_info: str = "") -> None:
    # Niveau : échelle de -60 dBFS (silence) à 0 dBFS (max)
    level_ratio = (level_db + 60.0) / 60.0
    score_color = RED if score >= threshold else (YELLOW if score >= threshold * 0.6 else GREEN)
    info_tag = f" | {CYAN}{peak_info}{RESET}" if peak_info else ""
    line = (
        f"\r{DIM}Niveau{RESET} [{bar(level_ratio, 15)}] {level_db:6.1f} dBFS | "
        f"{DIM}Drone{RESET} [{score_color}{bar(score, 15)}{RESET}] "
        f"{score_color}{score * 100:5.1f}%{RESET}{info_tag} | "
        f"{DIM}fenêtres {windows} | inf {latency_ms:5.1f} ms{RESET}   "
    )
    print(line, end="", flush=True)


def print_event(message: str, color: str) -> None:
    """Affiche un événement sur sa propre ligne (sans écraser la ligne de statut)."""
    print(f"\r{' ' * 100}\r[{time.strftime('%H:%M:%S')}] {color}{BOLD}{message}{RESET}")


# ---------------------------------------------------------------------------
# Arguments
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--engine", type=Path, default="drone_ast.engine",
                        help="Chemin vers le moteur TensorRT")
    parser.add_argument("--model", type=Path, default="model",
                        help="Chemin vers le dossier du modèle (config.json, etc.)")
    parser.add_argument("--threshold", type=float, default=THRESHOLD,
                        help="Seuil de probabilité pour une détection")
    parser.add_argument("--window-seconds", type=float, default=WINDOW_SECONDS,
                        help="Durée de la fenêtre d'analyse")
    parser.add_argument("--hop-seconds", type=float, default=HOP_SECONDS,
                        help="Pas de chevauchement entre les fenêtres")
    parser.add_argument("--consecutive", type=int, default=CONSECUTIVE_WINDOWS,
                        help="Nombre de fenêtres consécutives requises")
    parser.add_argument("--silence-dbfs", type=float, default=SILENCE_DBFS,
                        help="Seuil de silence en dBFS pour ignorer l'inférence")
    parser.add_argument("--device", type=str, default=None,
                        help="Périphérique d'entrée : index ou partie du nom")
    parser.add_argument("--channel", type=str, default=None,
                        help="Canal ou mode : 'all' (tous les canaux avec localisation), "
                             "'mix' (moyenne), 'auto' (crête dynamique), ou index 0..N-1")
    parser.add_argument("--list-devices", action="store_true",
                        help="Liste tous les périphériques d'entrée puis quitte "
                             "(sans test)")
    parser.add_argument("--test-mics", action="store_true",
                        help="Avec --list-devices : teste chaque micro par capture "
                             "réelle et ne liste que ceux qui captent du son")
    parser.add_argument("--no-status", action="store_true",
                        help="Désactive la ligne de statut temps réel")
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Programme principal
# ---------------------------------------------------------------------------

def main() -> int:
    args = parse_args()

    if args.list_devices:
        list_input_devices(probe=args.test_mics)
        return 0

    import pycuda.autoinit  # noqa: F401  (initialise le contexte CUDA)
    import ast_patch  # noqa: F401  (numpy shim for AST fbank)
    from transformers import AutoFeatureExtractor

    # 1. Chargement du modèle et du moteur
    config_path = args.model / "config.json"
    if not config_path.exists():
        print(f"Erreur : fichier de configuration introuvable à {config_path}", file=sys.stderr)
        return 1

    config = json.loads(config_path.read_text())
    drone_class = find_drone_class(config["id2label"])

    print("Chargement du feature extractor...")
    extractor = AutoFeatureExtractor.from_pretrained(args.model)
    sample_rate = int(extractor.sampling_rate)

    print(f"Chargement du moteur TensorRT depuis {args.engine}...")
    engine = TRTEngine(str(args.engine))

    # Vérification que le modèle est correctement chargé
    print(f"Vérification du modèle (entrée {engine.input_shape}, "
          f"sortie {engine.output_shape})...")
    try:
        verify_model(engine, extractor, sample_rate, drone_class)
    except RuntimeError as e:
        print(f"❌ Vérification du modèle échouée : {e}", file=sys.stderr)
        return 1
    print(f"{GREEN}✅ Modèle chargé et fonctionnel.{RESET}")

    # 2. Sélection du périphérique et du mode/canal
    try:
        device_index = resolve_device(args.device)
        channel_mode = resolve_channel(device_index, args.channel)
    except (RuntimeError, ValueError) as e:
        print(f"Erreur : {e}", file=sys.stderr)
        return 1

    device_info = sd.query_devices(device_index)
    native_rate = int(device_info["default_samplerate"])
    n_channels = int(device_info["max_input_channels"])

    # Normalisation du mode
    is_single = isinstance(channel_mode, int)
    target_channel = channel_mode if is_single else 0
    mode_display = (
        f"Canal {target_channel + 1}/{n_channels}" if is_single
        else f"Matrice complète ({channel_mode.upper()} sur {n_channels} canaux)"
    )

    print(f"\n🎤 {BOLD}Configuration microphone{RESET}")
    print(f"   - Périphérique : [{device_index}] {device_info['name']}")
    print(f"   - Mode audio   : {mode_display}")
    print(f"   - Taux cible   : {sample_rate} Hz")
    print(f"   - Fenêtre      : {args.window_seconds}s (hop {args.hop_seconds}s)")
    print(f"   - Détection    : seuil {args.threshold * 100:.0f}% sur "
          f"{args.consecutive} fenêtres consécutives\n")

    # 3. Configuration du flux audio
    use_mapping = is_single and ("mapping" in inspect.signature(sd.InputStream.__init__).parameters)
    capture_channels = 1 if use_mapping else n_channels
    if not use_mapping and is_single and n_channels > 1:
        print(f"ℹ️  Capture multi-canal ({n_channels} canaux) avec sélection logicielle du canal {target_channel + 1}.")

    audio_queue: queue.Queue[np.ndarray] = queue.Queue()

    def audio_callback(indata: np.ndarray, frames: int, time_info, status: sd.CallbackFlags):
        if status:
            print(f"\n⚠️  Status audio : {status}", file=sys.stderr)
        if use_mapping:
            audio_queue.put(indata[:, 0].copy())
        elif is_single:
            audio_queue.put(indata[:, target_channel].copy())
        else:
            audio_queue.put(indata.copy())

    def open_stream(rate: int) -> sd.InputStream:
        kwargs = dict(
            samplerate=rate,
            channels=capture_channels,
            dtype="float32",
            blocksize=max(512, int(round(args.hop_seconds * rate)) // 4),
            callback=audio_callback,
            device=device_index,
        )
        if use_mapping:
            kwargs["mapping"] = [target_channel + 1]
        return sd.InputStream(**kwargs)

    capture_rate = sample_rate
    try:
        stream = open_stream(capture_rate)
        stream.close()
    except sd.PortAudioError:
        capture_rate = native_rate
        print(f"ℹ️  {sample_rate} Hz non supporté, capture à {capture_rate} Hz "
              f"avec ré-échantillonnage logiciel.")

    resample_poly = None
    if capture_rate != sample_rate:
        from scipy.signal import resample_poly as _resample_poly

        ratio = Fraction(sample_rate, capture_rate).limit_denominator(1000)
        resample_ratio = (ratio.numerator, ratio.denominator)
        resample_poly = lambda x: _resample_poly(x, *resample_ratio, axis=0).astype(np.float32)

    window_size = int(round(args.window_seconds * sample_rate))
    hop_size = int(round(args.hop_seconds * sample_rate))

    # Statistiques de session
    n_windows = 0
    n_inferences = 0
    n_detections = 0
    latency_ms = 0.0
    last_level = -120.0
    start_time = time.time()

    try:
        with open_stream(capture_rate):
            # Test initial du micro
            print("Vérification du signal micro...", end="", flush=True)
            check_chunks = []
            check_samples = 0
            deadline = time.time() + 2.0
            target_check_samples = int(0.7 * capture_rate)

            while check_samples < target_check_samples and time.time() < deadline:
                while not audio_queue.empty():
                    c = audio_queue.get()
                    check_chunks.append(c)
                    check_samples += len(c)
                time.sleep(0.01)

            if not check_chunks:
                print(f"\n❌ Aucune donnée reçue du micro (périphérique [{device_index}]).", file=sys.stderr)
                return 1

            check = np.concatenate(check_chunks, axis=0)
            if check.ndim > 1:
                check_levels = [dbfs(check[:, c]) for c in range(check.shape[1])]
                check_level = max(check_levels)
            else:
                check_level = dbfs(check)

            if check_level < PROBE_MIN_DBFS:
                print(f"\r{' ' * 60}\r{YELLOW}⚠️  Signal micro très faible ({check_level:.1f} dBFS) "
                      f"— vérifiez les branchements.{RESET}")
            else:
                print(f"\r{' ' * 60}\r{GREEN}✅ Signal micro OK ({check_level:.1f} dBFS).{RESET}")

            print("✅ Écoute en cours... (Ctrl+C pour arrêter)\n")

            buffer = np.empty((0, n_channels) if (not is_single and not use_mapping) else (0,), dtype=np.float32)
            recent_scores: deque[float] = deque(maxlen=args.consecutive)
            is_detected = False
            last_peak_label = ""

            while True:
                while not audio_queue.empty():
                    chunk = audio_queue.get()
                    if resample_poly is not None:
                        chunk = resample_poly(chunk)
                    if buffer.size == 0:
                        buffer = chunk
                    else:
                        buffer = np.concatenate((buffer, chunk), axis=0)

                while len(buffer) >= window_size:
                    window = buffer[:window_size]
                    buffer = buffer[hop_size:]
                    n_windows += 1

                    # 1. Mode MIX (moyenne des canaux)
                    if channel_mode == "mix":
                        mono = window.mean(axis=1) if window.ndim > 1 else window
                        last_level = dbfs(mono)
                        if last_level < args.silence_dbfs:
                            score = 0.0
                        else:
                            feats = extractor(mono, sampling_rate=sample_rate, return_tensors="np")
                            input_values = feats["input_values"].astype(np.float32)
                            t0 = time.perf_counter()
                            logits = engine.infer(input_values)
                            latency_ms = (time.perf_counter() - t0) * 1000.0
                            n_inferences += 1
                            logits_shifted = logits - np.max(logits)
                            exp_logits = np.exp(logits_shifted)
                            probs = exp_logits / np.sum(exp_logits)
                            score = float(probs[0, drone_class])
                        peak_label = "Mix"

                    # 2. Mode AUTO (canal le plus fort)
                    elif channel_mode == "auto":
                        if window.ndim > 1:
                            ch_levels = [dbfs(window[:, c]) for c in range(window.shape[1])]
                            best_c = int(np.argmax(ch_levels))
                            mono = window[:, best_c]
                            last_level = ch_levels[best_c]
                        else:
                            best_c = 0
                            mono = window
                            last_level = dbfs(mono)

                        if last_level < args.silence_dbfs:
                            score = 0.0
                        else:
                            feats = extractor(mono, sampling_rate=sample_rate, return_tensors="np")
                            input_values = feats["input_values"].astype(np.float32)
                            t0 = time.perf_counter()
                            logits = engine.infer(input_values)
                            latency_ms = (time.perf_counter() - t0) * 1000.0
                            n_inferences += 1
                            logits_shifted = logits - np.max(logits)
                            exp_logits = np.exp(logits_shifted)
                            probs = exp_logits / np.sum(exp_logits)
                            score = float(probs[0, drone_class])
                        peak_label = f"Ch {best_c + 1}"

                    # 3. Mode CANAL UNIQUE FIXE
                    elif is_single:
                        mono = window[:, target_channel] if (window.ndim > 1 and not use_mapping) else window
                        last_level = dbfs(mono)
                        if last_level < args.silence_dbfs:
                            score = 0.0
                        else:
                            feats = extractor(mono, sampling_rate=sample_rate, return_tensors="np")
                            input_values = feats["input_values"].astype(np.float32)
                            t0 = time.perf_counter()
                            logits = engine.infer(input_values)
                            latency_ms = (time.perf_counter() - t0) * 1000.0
                            n_inferences += 1
                            logits_shifted = logits - np.max(logits)
                            exp_logits = np.exp(logits_shifted)
                            probs = exp_logits / np.sum(exp_logits)
                            score = float(probs[0, drone_class])
                        peak_label = f"Ch {target_channel + 1}"

                    # 4. Mode ALL (scan multi-micros avec détection spatiale)
                    else:
                        n_ch = window.shape[1] if window.ndim > 1 else 1
                        ch_levels = [dbfs(window[:, c]) if window.ndim > 1 else dbfs(window) for c in range(n_ch)]
                        last_level = max(ch_levels)
                        ch_scores = [0.0] * n_ch
                        t0 = time.perf_counter()
                        inf_done = 0

                        for c in range(n_ch):
                            if ch_levels[c] >= args.silence_dbfs:
                                mono = window[:, c] if window.ndim > 1 else window
                                feats = extractor(mono, sampling_rate=sample_rate, return_tensors="np")
                                input_values = feats["input_values"].astype(np.float32)
                                logits = engine.infer(input_values)
                                logits_shifted = logits - np.max(logits)
                                exp_logits = np.exp(logits_shifted)
                                probs = exp_logits / np.sum(exp_logits)
                                ch_scores[c] = float(probs[0, drone_class])
                                inf_done += 1

                        if inf_done > 0:
                            latency_ms = (time.perf_counter() - t0) * 1000.0
                            n_inferences += inf_done

                        score = max(ch_scores)
                        best_c = int(np.argmax(ch_scores)) if score > 0 else int(np.argmax(ch_levels))
                        peak_label = f"Mic {best_c + 1}" if n_ch > 1 else ""

                    recent_scores.append(score)
                    last_peak_label = peak_label

                    # Vérification de l'état de détection
                    if len(recent_scores) == args.consecutive:
                        median_score = float(np.median(recent_scores))
                        if median_score >= args.threshold and not is_detected:
                            n_detections += 1
                            tag = f" ({peak_label})" if peak_label else ""
                            print_event(f"🚨 DRONE DÉTECTÉ !{tag} (confiance médiane : {median_score * 100:.1f}%)", RED)
                            is_detected = True
                        elif median_score < args.threshold and is_detected:
                            print_event("✅ Signal de drone perdu.", GREEN)
                            is_detected = False

                if not args.no_status:
                    print_status(last_level, recent_scores[-1] if recent_scores else 0.0,
                                 args.threshold, n_windows, latency_ms, last_peak_label)
                time.sleep(0.01)

    except KeyboardInterrupt:
        duration = time.time() - start_time
        print(f"\n\n⏹️  {BOLD}Arrêt — résumé de session{RESET}")
        print(f"   Durée        : {duration:.1f} s")
        print(f"   Fenêtres     : {n_windows} (inférences : {n_inferences}, "
              f"silencieuses : {n_windows - n_inferences})")
        print(f"   Détections   : {n_detections}")
    except sd.PortAudioError as e:
        print(f"\n❌ Erreur audio : {e}", file=sys.stderr)
        print("Vérifiez que le microphone est connecté et autorisé, "
              "et le canal choisi avec --list-devices.", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

