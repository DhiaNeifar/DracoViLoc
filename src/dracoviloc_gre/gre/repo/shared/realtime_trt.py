"""DRACOVILOC - detecteur temps reel via moteur TensorRT, SANS PyTorch (pour le Jetson, ou
TensorRT est deja fourni par JetPack mais PyTorch n'est pas installe).

Ce module NE PEUT PAS importer `realtime.py` (qui importe torch) -- la machine a etats de
decision (lissage causal + hysteresis) est donc reproduite ici a l'identique plutot
qu'importee. Toute modification de la logique de decision dans `utils.py`/`realtime.py` doit
etre reportee ici manuellement -- c'est le compromis accepte pour eliminer la dependance
PyTorch sur la cible (cf. conversation, TensorRT deja installe sur le Jetson, PyTorch
difficile a installer).

L'extraction de features (`StreamingFeatureExtractor`) reste, elle, importee depuis `utils.py`
sans duplication -- ce module est deja sans dependance PyTorch (torch n'est importe qu'a
l'interieur de `build_gru_model`, jamais appele ici).

Prerequis sur la cible : TensorRT (bindings Python `tensorrt`, fournis par JetPack via apt)
et `pycuda` (pip, se compile localement).

Fichiers attendus (produits par `scripts/export_onnx.py` puis `trtexec`, voir ce script) :
    model_<feature_set>.engine        -- moteur TensorRT construit SUR la cible
    model_<feature_set>_meta.json     -- feat_mean, feat_std, decision, n_features, hidden_size
"""

from __future__ import annotations

import json
from collections import deque
from pathlib import Path

import numpy as np
import pycuda.autoinit  # noqa: F401 -- initialise le contexte CUDA courant, necessaire avant tout usage GPU
import pycuda.driver as cuda
import tensorrt as trt

import utils

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)


class TensorRTDroneDetector:
    """Equivalent de `realtime.RealtimeDroneDetector`, meme interface publique
    (`push_audio(chunk) -> list[dict]`), meme resultats numeriques (verifies a 1e-5 pres par
    `scripts/export_onnx.py` avant deploiement), mais l'inference GRU passe par un moteur
    TensorRT au lieu d'un `torch.nn.Module`.
    """

    def __init__(self, engine_path: str | Path, meta_path: str | Path, config: dict):
        self.config = config
        meta = json.loads(Path(meta_path).read_text(encoding="utf-8"))
        self.feature_set = meta["feature_set"]
        self.n_features = meta["n_features"]
        self.hidden_size = meta["hidden_size"]
        self.feat_mean = np.array(meta["feat_mean"], dtype=np.float32)
        self.feat_std = np.array(meta["feat_std"], dtype=np.float32)

        dcfg = meta["decision"]
        self.threshold_on = dcfg["threshold_on"]
        self.threshold_off = dcfg["threshold_off"]
        self.min_presence_s = dcfg["min_presence_s"]
        self.min_absence_s = dcfg["min_absence_s"]

        pcfg = config["preprocessing"]
        self.sr = pcfg["sample_rate"]
        self.frame_length = int(round(pcfg["frame_duration_s"] * self.sr))
        self.hop_length = int(round(pcfg["frame_hop_s"] * self.sr))
        smoothing_frames = max(1, int(round(config["decision"]["smoothing_s"] / pcfg["frame_hop_s"])))
        self._smoothing_window: deque[float] = deque(maxlen=smoothing_frames)

        self.extractor = utils.StreamingFeatureExtractor(self.sr, config, self.feature_set)

        self._sample_buffer = np.zeros(0, dtype=np.float32)
        self._n_samples_seen = 0
        self._decision_state = False
        self._candidate_since: float | None = None
        self._hidden = np.zeros((1, 1, self.hidden_size), dtype=np.float32)

        self._load_engine(str(engine_path))

    # --- moteur TensorRT -----------------------------------------------------------------
    def _load_engine(self, engine_path: str) -> None:
        runtime = trt.Runtime(TRT_LOGGER)
        with open(engine_path, "rb") as f:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()

        # buffers hote (pinned) + device pour x, h0 (entrees) et score, h_n (sorties) --
        # formes STATIQUES fixees a l'export ONNX (batch=1, seq=1)
        self._host = {}
        self._dev = {}
        shapes = {
            "x": (1, 1, self.n_features), "h0": (1, 1, self.hidden_size),
            "score": (1, 1), "h_n": (1, 1, self.hidden_size),
        }
        for name, shape in shapes.items():
            nbytes = int(np.prod(shape)) * 4  # float32
            self._host[name] = cuda.pagelocked_empty(shape, dtype=np.float32)
            self._dev[name] = cuda.mem_alloc(nbytes)
            self.context.set_tensor_address(name, int(self._dev[name]))
            if name in ("x", "h0"):
                self.context.set_input_shape(name, shape)

    def _run_engine(self, x: np.ndarray, h0: np.ndarray) -> tuple[float, np.ndarray]:
        self._host["x"][:] = x
        self._host["h0"][:] = h0
        cuda.memcpy_htod_async(self._dev["x"], self._host["x"], self.stream)
        cuda.memcpy_htod_async(self._dev["h0"], self._host["h0"], self.stream)
        self.context.execute_async_v3(self.stream.handle)
        cuda.memcpy_dtoh_async(self._host["score"], self._dev["score"], self.stream)
        cuda.memcpy_dtoh_async(self._host["h_n"], self._dev["h_n"], self.stream)
        self.stream.synchronize()
        return float(self._host["score"][0, 0]), self._host["h_n"].copy()

    # --- meme logique que RealtimeDroneDetector (dupliquee, voir docstring du module) -----
    def push_audio(self, chunk: np.ndarray) -> list[dict]:
        chunk = np.asarray(chunk, dtype=np.float32).reshape(-1)
        self._sample_buffer = np.concatenate([self._sample_buffer, chunk])

        results = []
        while len(self._sample_buffer) >= self.frame_length:
            frame = self._sample_buffer[: self.frame_length]
            self._n_samples_seen += self.hop_length
            self._sample_buffer = self._sample_buffer[self.hop_length :]

            t_s = (self._n_samples_seen - self.hop_length + self.frame_length) / self.sr
            results.append(self._process_frame(frame, t_s))
        return results

    def _process_frame(self, frame: np.ndarray, t_s: float) -> dict:
        feat = self.extractor.process_frame(frame)
        feat_norm = ((feat - self.feat_mean) / self.feat_std).astype(np.float32)
        x = feat_norm.reshape(1, 1, -1)

        score, self._hidden = self._run_engine(x, self._hidden)

        self._smoothing_window.append(score)
        smoothed = float(np.mean(self._smoothing_window))

        event_start, event_end = False, False
        if not self._decision_state:
            if smoothed >= self.threshold_on:
                if self._candidate_since is None:
                    self._candidate_since = t_s
                if t_s - self._candidate_since >= self.min_presence_s:
                    self._decision_state = True
                    self._candidate_since = None
                    event_start = True
            else:
                self._candidate_since = None
        else:
            if smoothed <= self.threshold_off:
                if self._candidate_since is None:
                    self._candidate_since = t_s
                if t_s - self._candidate_since >= self.min_absence_s:
                    self._decision_state = False
                    self._candidate_since = None
                    event_end = True
            else:
                self._candidate_since = None

        return {
            "t_s": t_s, "score": score, "smoothed_score": smoothed,
            "decision": self._decision_state, "event_start": event_start, "event_end": event_end,
        }
