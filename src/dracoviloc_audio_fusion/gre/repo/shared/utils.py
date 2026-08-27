"""DRACOVILOC - fonctions partagees entre les notebooks (CLAUDE.md Section 1.3).

Toute fonction utilisee par plus d'un notebook doit vivre ici. Un notebook qui
redefinit une fonction deja presente ici est une erreur (regle du projet).

Organisation :
    1. Configuration et reproductibilite
    2. E/S audio et fabrication de scenes (SNR controle)
    3. Pretraitement causal (filtre IIR, framing)
    4. Extraction de features (lfcc_hf / mfcc / wideband) + test de causalite + extracteur streaming
    5. Baseline energie 3-9 kHz
    6. Modele GRU causal
    7. Decision temporelle (hysteresis + persistance)
    8. Evaluation evenementielle
"""

from __future__ import annotations

import random
from collections import deque
from math import gcd
from pathlib import Path

import numpy as np
import pandas as pd
import scipy.signal as sig
import soundfile as sf
import yaml

EPS = 1e-10


# =============================================================================
# 1. Configuration et reproductibilite
# =============================================================================

def load_config(path: str | Path = "config/default.yaml") -> dict:
    """Charge la configuration YAML -- source unique de verite des hyperparametres."""
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def set_seed(seed: int) -> None:
    """Fixe la graine pour numpy, random, et torch si disponible (reproductibilite)."""
    random.seed(seed)
    np.random.seed(seed)
    try:
        import torch

        torch.manual_seed(seed)
        torch.use_deterministic_algorithms(True, warn_only=True)
    except ImportError:
        pass


# =============================================================================
# 2. E/S audio et fabrication de scenes
# =============================================================================

def load_audio(path: str | Path, sr: int) -> np.ndarray:
    """Charge un fichier audio, le convertit en mono et le reechantillonne a `sr`.

    Reechantillonnage polyphase (scipy.signal.resample_poly), deterministe.
    Le reechantillonnage complet d'un enregistrement pre-existant n'est pas une
    operation "en ligne" -- il prepare des donnees d'entrainement/test hors-ligne,
    ce n'est pas le chemin d'inference causal (voir stream_apply_causal_filter).
    """
    audio, file_sr = sf.read(str(path), always_2d=False)
    if audio.ndim > 1:
        audio = audio.mean(axis=1)
    audio = audio.astype(np.float64)
    if file_sr != sr:
        g = gcd(int(file_sr), int(sr))
        up, down = int(sr) // g, int(file_sr) // g
        audio = sig.resample_poly(audio, up, down)
    return audio.astype(np.float32)


def save_audio(path: str | Path, audio: np.ndarray, sr: int) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    sf.write(str(path), audio.astype(np.float32), sr)


def rms(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(x, dtype=np.float64)) + EPS))


def mix_at_snr(target: np.ndarray, noise: np.ndarray, snr_db: float, seed: int | None = None) -> np.ndarray:
    """Melange `target` (signal drone) et `noise` (bruit) au SNR impose (en dB).

    `noise` est boucle/tronque a la longueur de `target`. Le gain applique au bruit
    est calcule pour atteindre exactement le SNR demande en RMS.
    """
    rng = np.random.default_rng(seed)
    n_target = len(target)
    if len(noise) < n_target:
        n_repeat = int(np.ceil(n_target / len(noise)))
        noise = np.tile(noise, n_repeat)
    start = int(rng.integers(0, max(1, len(noise) - n_target + 1)))
    noise = noise[start:start + n_target]

    target_rms = rms(target)
    noise_rms = rms(noise)
    target_snr_linear = 10 ** (snr_db / 20.0)
    desired_noise_rms = target_rms / target_snr_linear
    gain = desired_noise_rms / (noise_rms + EPS)
    mixed = target + gain * noise

    peak = np.max(np.abs(mixed)) + EPS
    if peak > 0.99:
        mixed = mixed * (0.99 / peak)
    return mixed.astype(np.float32)


def concat_with_crossfade(segments: list[np.ndarray], sr: int, crossfade_s: float = 0.05) -> np.ndarray:
    """Concatene des segments audio avec un fondu enchaine court pour eviter les clics.

    Usage hors-ligne (fabrication de scenes dans le notebook 1) -- n'affecte pas la
    causalite du chemin d'inference.
    """
    if not segments:
        return np.array([], dtype=np.float32)
    n_fade = int(round(crossfade_s * sr))
    out = segments[0].astype(np.float64).copy()
    for seg in segments[1:]:
        seg = seg.astype(np.float64)
        if n_fade > 0 and len(out) >= n_fade and len(seg) >= n_fade:
            fade_out = np.linspace(1.0, 0.0, n_fade)
            fade_in = np.linspace(0.0, 1.0, n_fade)
            overlap = out[-n_fade:] * fade_out + seg[:n_fade] * fade_in
            out = np.concatenate([out[:-n_fade], overlap, seg[n_fade:]])
        else:
            out = np.concatenate([out, seg])
    return out.astype(np.float32)


# =============================================================================
# 3. Pretraitement causal (CLAUDE.md Section 4)
# =============================================================================

def design_causal_bandpass(low_hz: float, high_hz: float, sr: int, order: int = 4):
    """Filtre IIR passe-bande causal (Butterworth, sections biquad -- SOS).

    A appliquer uniquement avec `sosfilt` / `stream_apply_causal_filter`.
    JAMAIS `sosfiltfilt`/`filtfilt` : ces fonctions filtrent le signal en aller-retour
    (une passe avant, une passe arriere) pour annuler le dephasage -- la passe arriere
    utilise donc des echantillons FUTURS. Incompatible avec un detecteur causal.
    """
    nyq = sr / 2.0
    sos = sig.butter(order, [low_hz / nyq, high_hz / nyq], btype="bandpass", output="sos")
    return sos


def apply_causal_filter_offline(sos: np.ndarray, audio: np.ndarray) -> np.ndarray:
    """Filtrage causal d'un signal complet (usage notebooks 1/2, hors-ligne).

    Note de causalite : meme applique en un seul appel sur tout le tableau, `sosfilt`
    reste mathematiquement causal -- output[n] ne depend que de input[0..n]. Le
    decoupage en trames avec etat `zi` explicite (voir `stream_apply_causal_filter`)
    n'est necessaire que pour le VRAI streaming (donnees qui arrivent morceau par
    morceau au fil du temps), pas pour la correction de la causalite elle-meme.
    """
    return sig.sosfilt(sos, audio).astype(np.float32)


def stream_apply_causal_filter(sos: np.ndarray, chunk: np.ndarray, zi: np.ndarray | None = None):
    """Filtrage causal trame par trame avec etat persistant (usage notebook 3).

    Retourne (sortie_filtree, etat_final). `zi` doit etre reinjecte a l'appel suivant.
    """
    if zi is None:
        zi = sig.sosfilt_zi(sos) * (chunk[0] if len(chunk) else 0.0)
    y, zf = sig.sosfilt(sos, chunk, zi=zi)
    return y.astype(np.float32), zf


# -----------------------------------------------------------------------------
# Framing causal
# -----------------------------------------------------------------------------

def frame_signal(audio: np.ndarray, sr: int, frame_duration_s: float, frame_hop_s: float):
    """Decoupe `audio` en trames causales.

    La trame i couvre les echantillons [i*hop, i*hop+frame_length) ; son horodatage
    de reference est la FIN de la trame (dernier echantillon), jamais le centre.
    Aucun padding centre ("reflect") n'est utilise : c'est la source la plus frequente
    de fuite du futur dans les bibliotheques audio standards (torchaudio/librosa avec
    center=True par defaut regardent n_fft/2 echantillons dans le futur).

    Retourne (frames [n_frames, frame_length], frame_end_s [n_frames], frame_length, hop_length).
    """
    frame_length = int(round(frame_duration_s * sr))
    hop_length = int(round(frame_hop_s * sr))
    if len(audio) < frame_length:
        return (
            np.zeros((0, frame_length), dtype=np.float64),
            np.zeros((0,), dtype=np.float64),
            frame_length,
            hop_length,
        )
    n_frames = 1 + (len(audio) - frame_length) // hop_length
    frames = np.empty((n_frames, frame_length), dtype=np.float64)
    frame_end_s = np.empty((n_frames,), dtype=np.float64)
    for i in range(n_frames):
        start = i * hop_length
        frames[i] = audio[start:start + frame_length]
        frame_end_s[i] = (start + frame_length) / sr
    return frames, frame_end_s, frame_length, hop_length


# =============================================================================
# 4. Extraction de features (CLAUDE.md Section 5)
# =============================================================================

def linear_filterbank(sr: int, n_fft: int, n_filters: int, fmin: float, fmax: float) -> np.ndarray:
    """Banc de filtres triangulaires a resolution LINEAIRE entre fmin et fmax.

    Matrice statique (ne depend que de sr/n_fft/n_filters/fmin/fmax) -- sa
    construction ne "regarde" jamais le signal, donc n'affecte pas la causalite.
    """
    n_bins = n_fft // 2 + 1
    fft_freqs = np.fft.rfftfreq(n_fft, d=1.0 / sr)
    edges_hz = np.linspace(fmin, fmax, n_filters + 2)
    edges_bin = np.clip(np.floor((n_fft + 1) * edges_hz / sr).astype(int), 0, n_bins - 1)

    fb = np.zeros((n_filters, n_bins), dtype=np.float64)
    for m in range(1, n_filters + 1):
        left, center, right = edges_bin[m - 1], edges_bin[m], edges_bin[m + 1]
        if center > left:
            k = np.arange(left, center)
            fb[m - 1, k] = (k - left) / (center - left)
        if right > center:
            k = np.arange(center, right)
            fb[m - 1, k] = (right - k) / (right - center)
    del fft_freqs
    return fb


def mel_filterbank(sr: int, n_fft: int, n_filters: int, fmin: float, fmax: float) -> np.ndarray:
    """Banc de filtres triangulaires a resolution MEL (reference MFCC, CLAUDE.md 5.4).

    Implementation maison (evite une dependance a librosa.filters.mel pour ce calcul
    purement geometrique, statique, sans lien avec la causalite).
    """
    def hz_to_mel(f):
        return 2595.0 * np.log10(1.0 + f / 700.0)

    def mel_to_hz(m):
        return 700.0 * (10 ** (m / 2595.0) - 1.0)

    n_bins = n_fft // 2 + 1
    edges_mel = np.linspace(hz_to_mel(fmin), hz_to_mel(fmax), n_filters + 2)
    edges_hz = mel_to_hz(edges_mel)
    edges_bin = np.clip(np.floor((n_fft + 1) * edges_hz / sr).astype(int), 0, n_bins - 1)

    fb = np.zeros((n_filters, n_bins), dtype=np.float64)
    for m in range(1, n_filters + 1):
        left, center, right = edges_bin[m - 1], edges_bin[m], edges_bin[m + 1]
        if center > left:
            k = np.arange(left, center)
            fb[m - 1, k] = (k - left) / (center - left)
        if right > center:
            k = np.arange(center, right)
            fb[m - 1, k] = (right - k) / (right - center)
    return fb


def _dct2_orthonormal(x: np.ndarray, n_out: int) -> np.ndarray:
    """DCT-II orthonormale le long du dernier axe (evite la dependance scipy.fftpack)."""
    n_in = x.shape[-1]
    k = np.arange(n_out)[:, None]
    n = np.arange(n_in)[None, :]
    basis = np.cos(np.pi / n_in * (n + 0.5) * k)
    basis[0, :] *= 1.0 / np.sqrt(2.0)
    basis *= np.sqrt(2.0 / n_in)
    return x @ basis.T


def _power_spectrum_frames(frames: np.ndarray, n_fft: int) -> np.ndarray:
    """Spectre de puissance par trame. Fenetrage de Hamming APPLIQUE DANS LA TRAME
    (pas de padding centre) -- chaque trame ne voit que ses propres echantillons.
    """
    window = np.hamming(frames.shape[1])
    windowed = frames * window[None, :]
    spectrum = np.fft.rfft(windowed, n=n_fft, axis=1)
    return (np.abs(spectrum) ** 2) / n_fft


def spectral_descriptors(power_spec: np.ndarray, freqs: np.ndarray, band: tuple[float, float],
                          noise_floor_window_frames: int = 200, rolloff_ratio: float = 0.85) -> np.ndarray:
    """Calcule les 5 descripteurs spectraux complementaires (CLAUDE.md Section 5.2),
    trame par trame, de facon strictement causale.

    Les CINQ descripteurs sont calcules exclusivement DANS `band` (pas seulement le SNR --
    correction audit #1 : centroide/roll-off/flatness/flux etaient calcules sur le spectre
    ENTIER, ce qui laissait un offset DC ou un ronflement secteur hors-bande dominer ces
    features et provoquer un score aberrant). Pour `feature_set="wideband"/"mfcc"`, `band`
    couvre deja tout le spectre audible (0, nyquist) : aucun changement de comportement pour
    ces deux pistes, la restriction ne change quelque chose que pour `lfcc_hf` (3-9 kHz).

    - energie relative / SNR adaptatif dans `band` (plancher de bruit = minimum
      GLISSANT sur les `noise_floor_window_frames` trames PASSEES uniquement --
      fenetre glissante "trailing", jamais centree) ;
    - centroide spectral ;
    - roll-off spectral (frequence sous laquelle se trouve `rolloff_ratio` de l'energie) ;
    - flatness spectrale (moyenne geometrique / moyenne arithmetique) ;
    - flux spectral (norme L2 de la difference entre spectres normalises consecutifs --
      ne depend que de la trame courante et de la PRECEDENTE).

    Retourne un tableau [n_frames, 5].
    """
    n_frames = power_spec.shape[0]

    band_mask = (freqs >= band[0]) & (freqs <= band[1])
    band_power = power_spec[:, band_mask]
    band_freqs = freqs[band_mask]
    band_energy = band_power.sum(axis=1)
    total_energy = band_energy + EPS

    noise_floor = (
        pd.Series(band_energy)
        .rolling(window=noise_floor_window_frames, min_periods=1)
        .quantile(0.10)
        .to_numpy()
    )
    snr_adaptive_db = 10.0 * np.log10((band_energy + EPS) / (noise_floor + EPS))

    centroid = (band_power @ band_freqs) / total_energy

    cumulative = np.cumsum(band_power, axis=1)
    rolloff_target = rolloff_ratio * total_energy
    rolloff_idx = np.array([
        int(np.searchsorted(cumulative[i], rolloff_target[i])) for i in range(n_frames)
    ])
    rolloff_idx = np.clip(rolloff_idx, 0, len(band_freqs) - 1)
    rolloff = band_freqs[rolloff_idx]

    geo_mean = np.exp(np.mean(np.log(band_power + EPS), axis=1))
    arith_mean = np.mean(band_power, axis=1) + EPS
    flatness = geo_mean / arith_mean

    normalized = band_power / total_energy[:, None]
    flux = np.zeros(n_frames, dtype=np.float64)
    if n_frames > 1:
        flux[1:] = np.linalg.norm(normalized[1:] - normalized[:-1], axis=1)

    return np.stack([snr_adaptive_db, centroid, rolloff, flatness, flux], axis=1)


FEATURE_SETS = ("lfcc_hf", "mfcc", "wideband", "logmel")


def extract_features(audio: np.ndarray, sr: int, config: dict, feature_set: str | None = None):
    """Extraction de features deterministe et causale, paremetrable par `feature_set`
    (CLAUDE.md Section 5.4) : "lfcc_hf" (principal), "mfcc" (reference litterature),
    "wideband" (reference bande large), "logmel" (experimental -- meme banc de filtres mel
    que mfcc mais SANS DCT ni troncature, hypothese que la DCT supprime le "peigne"
    harmonique discriminant d'un drone -- voir reports/PERFORMANCE_LOGMEL.md).

    Causalite : chaque trame n'utilise que ses propres echantillons (fenetrage sans
    padding centre) ; le flux spectral et le plancher de bruit ne regardent que les
    trames PASSEES. Aucune etape ne depend d'un echantillon futur.

    Retourne (features [n_frames, n_features], frame_end_s [n_frames]).
    """
    feature_set = feature_set or config["features"]["feature_set"]
    assert feature_set in FEATURE_SETS, f"feature_set inconnu: {feature_set}"

    fcfg = config["features"]
    pcfg = config["preprocessing"]
    n_cep = fcfg["n_cepstral_coeffs"]
    include_c0 = fcfg["include_c0"]

    frames, frame_end_s, frame_length, _ = frame_signal(
        audio, sr, pcfg["frame_duration_s"], pcfg["frame_hop_s"]
    )
    if frames.shape[0] == 0:
        n_descr = 5
        n_out = fcfg["n_logmel_bands"] if feature_set == "logmel" else n_cep
        return np.zeros((0, n_out + n_descr), dtype=np.float32), frame_end_s

    n_fft = int(2 ** np.ceil(np.log2(frame_length)))
    power_spec = _power_spectrum_frames(frames, n_fft)
    freqs = np.fft.rfftfreq(n_fft, d=1.0 / sr)
    nyquist = sr / 2.0

    hf_band = (fcfg["band_hf_low_hz"], fcfg["band_hf_high_hz"])
    full_band = (0.0, nyquist)

    if feature_set == "lfcc_hf":
        fb = linear_filterbank(sr, n_fft, n_cep + 2, hf_band[0], hf_band[1])
        descr_band = hf_band
    elif feature_set == "wideband":
        fb = linear_filterbank(sr, n_fft, n_cep + 2, full_band[0], full_band[1])
        descr_band = full_band
    elif feature_set == "logmel":
        fb = mel_filterbank(sr, n_fft, fcfg["n_logmel_bands"], full_band[0], full_band[1])
        descr_band = full_band
    else:  # mfcc
        fb = mel_filterbank(sr, n_fft, n_cep + 2, full_band[0], full_band[1])
        descr_band = full_band

    filter_energy = power_spec @ fb.T
    log_filter_energy = np.log(filter_energy + EPS)
    if feature_set == "logmel":
        spectral_feats = log_filter_energy  # PAS de DCT (hypothese testee) -- log-mel brut
    else:
        cepstral = _dct2_orthonormal(log_filter_energy, n_cep + 1)
        spectral_feats = cepstral[:, : n_cep + 1] if include_c0 else cepstral[:, 1 : n_cep + 1]

    descriptors = spectral_descriptors(power_spec, freqs, descr_band)

    features = np.concatenate([spectral_feats, descriptors], axis=1).astype(np.float32)
    return features, frame_end_s


def check_causality(audio: np.ndarray, sr: int, config: dict, feature_set: str,
                     perturb_after_s: float, rng_seed: int = 0, atol: float = 1e-5) -> dict:
    """Test de causalite (CLAUDE.md Section 4, cellule bloquante du notebook 2).

    Remplace les echantillons APRES `perturb_after_s` par du bruit, recalcule les
    features, et verifie qu'aucune feature dont l'horodatage de fin <= perturb_after_s
    n'a change. Retourne un rapport {"passed": bool, "n_checked": int, "max_abs_diff": float}.
    """
    rng = np.random.default_rng(rng_seed)
    perturb_idx = int(round(perturb_after_s * sr))

    audio_perturbed = audio.copy()
    if perturb_idx < len(audio_perturbed):
        audio_perturbed[perturb_idx:] = rng.uniform(-1.0, 1.0, size=len(audio) - perturb_idx)

    feats_a, times_a = extract_features(audio, sr, config, feature_set)
    feats_b, times_b = extract_features(audio_perturbed, sr, config, feature_set)

    n_common = min(len(times_a), len(times_b))
    safe_mask = times_a[:n_common] <= perturb_after_s
    n_checked = int(safe_mask.sum())

    if n_checked == 0:
        return {"passed": True, "n_checked": 0, "max_abs_diff": 0.0, "warning": "aucune trame anterieure a la perturbation"}

    diff = np.abs(feats_a[:n_common][safe_mask] - feats_b[:n_common][safe_mask])
    max_abs_diff = float(diff.max())
    return {"passed": bool(max_abs_diff <= atol), "n_checked": n_checked, "max_abs_diff": max_abs_diff}


class StreamingFeatureExtractor:
    """Extraction causale des memes features qu'`extract_features`, mais TRAME PAR TRAME
    avec etat persistant (plancher de bruit glissant, spectre precedent pour le flux) --
    utilisee par le notebook 3 pour simuler le flux temps reel via un buffer circulaire.

    Doit produire, trame par trame, exactement les memes valeurs que `extract_features`
    appelee en une fois sur l'audio complet (verifie dans le notebook 3).
    """

    def __init__(self, sr: int, config: dict, feature_set: str, noise_floor_window_frames: int = 200, rolloff_ratio: float = 0.85):
        fcfg = config["features"]
        pcfg = config["preprocessing"]
        self.sr = sr
        self.frame_length = int(round(pcfg["frame_duration_s"] * sr))
        self.n_fft = int(2 ** np.ceil(np.log2(self.frame_length)))
        self.freqs = np.fft.rfftfreq(self.n_fft, d=1.0 / sr)
        self.window = np.hamming(self.frame_length)
        self.feature_set = feature_set
        self.n_cep = fcfg["n_cepstral_coeffs"]
        self.include_c0 = fcfg["include_c0"]
        self.rolloff_ratio = rolloff_ratio

        nyquist = sr / 2.0
        hf_band = (fcfg["band_hf_low_hz"], fcfg["band_hf_high_hz"])
        full_band = (0.0, nyquist)
        if feature_set == "lfcc_hf":
            self.fb = linear_filterbank(sr, self.n_fft, self.n_cep + 2, *hf_band)
            descr_band = hf_band
        elif feature_set == "wideband":
            self.fb = linear_filterbank(sr, self.n_fft, self.n_cep + 2, *full_band)
            descr_band = full_band
        elif feature_set == "logmel":
            self.fb = mel_filterbank(sr, self.n_fft, fcfg["n_logmel_bands"], *full_band)
            descr_band = full_band
        elif feature_set == "mfcc":
            self.fb = mel_filterbank(sr, self.n_fft, self.n_cep + 2, *full_band)
            descr_band = full_band
        else:
            raise ValueError(feature_set)
        self.band_mask = (self.freqs >= descr_band[0]) & (self.freqs <= descr_band[1])
        self.band_freqs = self.freqs[self.band_mask]

        self._band_energy_history: deque[float] = deque(maxlen=noise_floor_window_frames)
        self._prev_normalized_spectrum: np.ndarray | None = None

    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        """Traite UNE trame de `self.frame_length` echantillons -> vecteur de features.

        Ne lit que `frame` (les echantillons de cette trame) et l'etat interne accumule
        a partir des trames PASSEES -- aucune dependance a des echantillons futurs.
        """
        windowed = frame * self.window
        spectrum = np.fft.rfft(windowed, n=self.n_fft)
        power = (np.abs(spectrum) ** 2) / self.n_fft

        filter_energy = power @ self.fb.T
        log_filter_energy = np.log(filter_energy + EPS)
        if self.feature_set == "logmel":
            spectral_feats = log_filter_energy  # PAS de DCT (hypothese testee) -- log-mel brut
        else:
            cepstral_full = _dct2_orthonormal(log_filter_energy[None, :], self.n_cep + 1)[0]
            spectral_feats = cepstral_full[: self.n_cep + 1] if self.include_c0 else cepstral_full[1 : self.n_cep + 1]

        # descripteurs restreints a `self.band_mask` (correction audit #1 -- auparavant seul
        # le SNR l'etait, les 4 autres voyaient le spectre entier, DC compris)
        band_power = power[self.band_mask]
        band_energy = float(band_power.sum())
        total_energy = band_energy + EPS
        self._band_energy_history.append(band_energy)
        noise_floor = float(np.quantile(np.fromiter(self._band_energy_history, dtype=np.float64), 0.10))
        snr_db = 10.0 * np.log10((band_energy + EPS) / (noise_floor + EPS))

        centroid = float(band_power @ self.band_freqs) / total_energy
        cumulative = np.cumsum(band_power)
        rolloff_idx = min(int(np.searchsorted(cumulative, self.rolloff_ratio * total_energy)), len(self.band_freqs) - 1)
        rolloff = self.band_freqs[rolloff_idx]

        geo_mean = np.exp(np.mean(np.log(band_power + EPS)))
        arith_mean = np.mean(band_power) + EPS
        flatness = geo_mean / arith_mean

        normalized = band_power / total_energy
        if self._prev_normalized_spectrum is None:
            flux = 0.0
        else:
            flux = float(np.linalg.norm(normalized - self._prev_normalized_spectrum))
        self._prev_normalized_spectrum = normalized

        descriptors = np.array([snr_db, centroid, rolloff, flatness, flux], dtype=np.float32)
        return np.concatenate([spectral_feats, descriptors]).astype(np.float32)


# =============================================================================
# 5. Baseline energie 3-9 kHz (metre-etalon, CLAUDE.md Section 9)
# =============================================================================

def baseline_energy_score(audio: np.ndarray, sr: int, config: dict) -> tuple[np.ndarray, np.ndarray]:
    """Score baseline causal : energie relative dans la bande 3-9 kHz, normalisee
    en score [0, 1] par une sigmoide sur le SNR adaptatif en dB (meme plancher de
    bruit glissant-causal que les descripteurs de `spectral_descriptors`).

    Retourne (score [n_frames] dans [0, 1], frame_end_s [n_frames]).
    """
    bcfg = config["baseline"]
    pcfg = config["preprocessing"]

    frames, frame_end_s, frame_length, _ = frame_signal(
        audio, sr, pcfg["frame_duration_s"], pcfg["frame_hop_s"]
    )
    if frames.shape[0] == 0:
        return np.zeros((0,), dtype=np.float32), frame_end_s

    n_fft = int(2 ** np.ceil(np.log2(frame_length)))
    power_spec = _power_spectrum_frames(frames, n_fft)
    freqs = np.fft.rfftfreq(n_fft, d=1.0 / sr)
    descr = spectral_descriptors(power_spec, freqs, (bcfg["band_low_hz"], bcfg["band_high_hz"]))
    snr_db = descr[:, 0]

    score = 1.0 / (1.0 + np.exp(-(snr_db - 6.0) / 3.0))
    return score.astype(np.float32), frame_end_s


# =============================================================================
# 6. Modele GRU causal (CLAUDE.md Section 6)
# =============================================================================

def get_device():
    """GPU CUDA si disponible, sinon CPU. Import torch differe (meme raison que
    build_gru_model : ce module reste importable sans torch installe).

    A utiliser pour l'ENTRAINEMENT et le score par lot (sequences longues, le GPU apporte un
    gain reel). Ne PAS l'utiliser par defaut pour l'inference streaming trame-par-trame
    (RealtimeDroneDetector) : sur un modele aussi petit (~5700 parametres), le cout de
    transfert CPU<->GPU par appel depasse largement le calcul lui-meme et ralentirait le
    temps reel au lieu de l'accelerer -- le streaming reste delibbrement sur CPU.
    """
    import torch

    return torch.device("cuda" if torch.cuda.is_available() else "cpu")


def build_gru_model(input_size: int, config: dict):
    """Construit le GRU causal leger. Import torch differe pour que ce module reste
    important sans torch installe (utile pour le notebook 1, qui n'en a pas besoin).
    """
    import torch
    import torch.nn as nn

    mcfg = config["model"]
    assert mcfg["bidirectional"] is False, "CLAUDE.md #6 [CONTRAINTE] : jamais bidirectionnel"

    class CausalGRUDetector(nn.Module):
        """GRU unidirectionnel + tete lineaire -> score drone par pas de temps.

        L'etat cache `h` peut etre fourni/recupere explicitement pour l'inference
        streaming (notebook 3) : le score au pas t ne depend que des pas <= t.
        """

        def __init__(self):
            super().__init__()
            self.gru = nn.GRU(
                input_size=input_size,
                hidden_size=mcfg["hidden_size"],
                num_layers=mcfg["num_layers"],
                batch_first=True,
                bidirectional=False,
                dropout=mcfg["dropout"] if mcfg["num_layers"] > 1 else 0.0,
            )
            self.head = nn.Linear(mcfg["hidden_size"], 1)

        def forward(self, x, h0=None):
            out, h_n = self.gru(x, h0)
            score = torch.sigmoid(self.head(out)).squeeze(-1)
            return score, h_n

    return CausalGRUDetector()


# =============================================================================
# 7. Decision temporelle : hysteresis + persistance (CLAUDE.md Section 8)
# =============================================================================

def smooth_causal(score: np.ndarray, frame_hop_s: float, smoothing_s: float) -> np.ndarray:
    """Lissage causal court : moyenne glissante TRAILING (jamais centree)."""
    window = max(1, int(round(smoothing_s / frame_hop_s)))
    return pd.Series(score).rolling(window=window, min_periods=1).mean().to_numpy()


def apply_hysteresis(score: np.ndarray, times_s: np.ndarray, config: dict) -> tuple[np.ndarray, list[dict]]:
    """Chaine de decision : lissage -> seuil haut/bas -> duree min presence/absence.

    Retourne (decision_binaire [n_frames], liste d'evenements [{start_s, end_s}]).
    """
    dcfg = config["decision"]
    pcfg = config["preprocessing"]
    smoothed = smooth_causal(score, pcfg["frame_hop_s"], dcfg["smoothing_s"])

    state = False  # False = non-drone, True = drone
    decision = np.zeros(len(smoothed), dtype=bool)
    candidate_since = None  # instant depuis lequel le score franchit le seuil oppose

    for i, (s, t) in enumerate(zip(smoothed, times_s)):
        if not state:
            if s >= dcfg["threshold_on"]:
                if candidate_since is None:
                    candidate_since = t
                if t - candidate_since >= dcfg["min_presence_s"]:
                    state = True
                    candidate_since = None
            else:
                candidate_since = None
        else:
            if s <= dcfg["threshold_off"]:
                if candidate_since is None:
                    candidate_since = t
                if t - candidate_since >= dcfg["min_absence_s"]:
                    state = False
                    candidate_since = None
            else:
                candidate_since = None
        decision[i] = state

    events = []
    in_event = False
    ev_start = None
    for d, t in zip(decision, times_s):
        if d and not in_event:
            in_event = True
            ev_start = t
        elif not d and in_event:
            in_event = False
            events.append({"start_s": ev_start, "end_s": t})
    if in_event:
        events.append({"start_s": ev_start, "end_s": times_s[-1]})

    return decision, events


# =============================================================================
# 8. Evaluation evenementielle (CLAUDE.md Section 9)
# =============================================================================

def _intervals_from_annotations(annotations: pd.DataFrame, recording_id: str, label: str = "drone") -> list[tuple[float, float]]:
    rows = annotations[(annotations["recording_id"] == recording_id) & (annotations["label"] == label)]
    return list(zip(rows["start_s"], rows["end_s"]))


def evaluate_events(predicted_events: list[dict], ground_truth: list[tuple[float, float]],
                     recording_duration_s: float, min_overlap_s: float = 0.1) -> dict:
    """Metriques evenementielles (CLAUDE.md Section 9) pour UN enregistrement.

    - appariement UN-A-UN glouton par recouvrement decroissant : un evenement predit et un
      evenement verite-terrain ne peuvent chacun etre revendiques qu'une seule fois (correction
      audit #2 -- auparavant plusieurs evenements predits pouvaient tous "matcher" le meme
      evenement reel sans penalite, ce qui gonflait artificiellement precision/F1 des que la
      decision fragmentait une presence drone continue en plusieurs blocs) ;
    - un evenement predit "matche" un evenement verite-terrain s'ils se chevauchent d'au moins
      `min_overlap_s` ET si aucun des deux n'est deja revendique par une paire de meilleur
      recouvrement ;
    - rappel = evenements verite-terrain matches / total verite-terrain ;
    - precision = evenements predits matches / total predits ;
    - fausses alarmes = evenements predits non matches (sans chevauchement suffisant, OU
      chevauchant un evenement reel deja revendique par un meilleur match -- fragment/insertion
      redondante, comptee comme fausse alarme plutot que comme un match gratuit) ;
    - latence d'apparition = start_predit - start_verite (pour les matches, >=0 si en retard) ;
    - latence de disparition = end_predit - end_verite (pour les matches) ;
    - duree des detections parasites = duree totale des faux positifs.
    """
    def overlap(a, b):
        return max(0.0, min(a[1], b[1]) - max(a[0], b[0]))

    pairs = []  # (recouvrement, indice_predit, indice_reel), pour toutes les paires eligibles
    for pi, pe in enumerate(predicted_events):
        p_interval = (pe["start_s"], pe["end_s"])
        for gi, ge in enumerate(ground_truth):
            ov = overlap(p_interval, ge)
            if ov >= min_overlap_s:
                pairs.append((ov, pi, gi))
    pairs.sort(key=lambda x: x[0], reverse=True)

    gt_matched = [False] * len(ground_truth)
    pred_matched = [False] * len(predicted_events)
    onset_latencies, offset_latencies = [], []

    for ov, pi, gi in pairs:
        if pred_matched[pi] or gt_matched[gi]:
            continue
        pred_matched[pi] = True
        gt_matched[gi] = True
        onset_latencies.append(predicted_events[pi]["start_s"] - ground_truth[gi][0])
        offset_latencies.append(predicted_events[pi]["end_s"] - ground_truth[gi][1])

    n_gt = len(ground_truth)
    n_pred = len(predicted_events)
    n_gt_matched = sum(gt_matched)
    n_pred_matched = sum(pred_matched)

    recall = n_gt_matched / n_gt if n_gt > 0 else float("nan")
    precision = n_pred_matched / n_pred if n_pred > 0 else float("nan")
    f1 = (2 * precision * recall / (precision + recall)) if (precision and recall and precision + recall > 0) else float("nan")

    false_alarms = [predicted_events[i] for i in range(n_pred) if not pred_matched[i]]
    duration_min = recording_duration_s / 60.0
    fa_per_min = len(false_alarms) / duration_min if duration_min > 0 else float("nan")
    spurious_duration_s = sum(fa["end_s"] - fa["start_s"] for fa in false_alarms)

    return {
        "n_ground_truth_events": n_gt,
        "n_predicted_events": n_pred,
        "n_false_alarms": len(false_alarms),
        "recording_duration_s": recording_duration_s,
        "recall": recall,
        "precision": precision,
        "f1": f1,
        "false_alarms_per_min": fa_per_min,
        "onset_latency_s_mean": float(np.mean(onset_latencies)) if onset_latencies else float("nan"),
        "offset_latency_s_mean": float(np.mean(offset_latencies)) if offset_latencies else float("nan"),
        "spurious_detection_duration_s": spurious_duration_s,
    }
