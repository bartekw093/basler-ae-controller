"""
basler_auto_exposure.py

Plug-and-play kontroler auto-exposure / auto-gain dla kamer Basler (pypylon),
zaprojektowany pod stałe FPS (np. 25fps) z ograniczeniem czasowym na ExposureTime.

Użycie minimalne:

    from pypylon import pylon
    from basler_auto_exposure import AutoExposureController, AEConfig

    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    camera.Open()

    ae = AutoExposureController(camera, AEConfig(target_fps=25))
    ae.start()  # wybiera automatycznie native AE Baslera albo software loop

    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    while camera.IsGrabbing():
        grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if grab_result.GrabSucceeded():
            img = grab_result.Array
            ae.update(img)          # no-op jeśli używasz native AE
            ... # dalsze przetwarzanie
        grab_result.Release()

Działa z dowolną aplikacją bo:
- nie zakłada żadnego konkretnego pipeline'u przetwarzania obrazu
- automatycznie wykrywa czy kamera ma ExposureAuto/GainAuto i preferuje native AE
- fallback na software PI-loop jeśli native AE nie jest dostępny lub explicite wyłączony
- wszystkie parametry konfigurowalne przez dataclass AEConfig
- bezpieczne wobec błędów GenICam (nie wywala pipeline'u przy okazjonalnym rzucie wyjątku)
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional, Literal

import numpy as np

try:
    from pypylon import genicam
except ImportError:
    genicam = None  # pozwala importować moduł bez pypylon (np. do testów typowania)

logger = logging.getLogger("basler_auto_exposure")


@dataclass
class AEConfig:
    """Konfiguracja kontrolera auto-exposure/gain."""

    # --- Cel regulacji ---
    target_fps: float = 25.0
    target_brightness: float = 128.0      # cel w skali 0-255 (jeśli obraz 16-bit, normalizujemy wewnętrznie)
    tolerance: float = 5.0                # martwa strefa wokół celu - zapobiega "drganiu"
    brightness_metric: Literal["mean", "percentile"] = "mean"
    percentile: float = 95.0              # używane gdy brightness_metric == "percentile"

    # --- Limity exposure (w mikrosekundach) ---
    exposure_safety_margin_us: float = 2000.0  # margines na readout/transfer poniżej okresu ramki
    min_exposure_us: float = 100.0
    max_exposure_us: Optional[float] = None     # None = wyliczane automatycznie z target_fps

    # --- Limity gain (jednostki zależne od kamery - zwykle dB) ---
    min_gain: float = 0.0
    max_gain: Optional[float] = None      # None = czyta z kamery (GainMax)

    # --- Tryb pracy ---
    prefer_native_ae: bool = True         # jeśli kamera wspiera ExposureAuto/GainAuto - użyj go
    # Różne modele Baslera używają różnych stringów dla AutoFunctionProfile -
    # próbujemy po kolei aż któryś zostanie zaakceptowany. "MinimizeGain" oznacza
    # priorytet exposure nad gainem (mniej szumu).
    native_auto_function_profiles: tuple = ("MinimizeGain", "Gain", "GainMinimum")

    # Twarde wymuszenie FPS - bez tego Basler niekoniecznie utrzymuje stałą
    # częstotliwość klatek; ograniczenie samego exposure to za mało.
    enforce_frame_rate: bool = True

    # --- Parametry software PI-loop (używane tylko gdy native AE niedostępny) ---
    kp_exposure: float = 60.0             # wzmocnienie proporcjonalne -> us na jednostkę błędu jasności
    kp_gain: float = 0.08                 # wzmocnienie proporcjonalne -> dB na jednostkę błędu jasności
    ki: float = 0.0                       # człon całkujący (0 = czysty P, włącz jeśli zostaje stały offset)
    update_every_n_frames: int = 3        # nie regulować co frame - tłumi drgania
    max_step_fraction: float = 0.25       # max zmiana exposure/gain na krok jako % zakresu (anti-windup/szarpanie)

    # --- Pomiar jasności / głębia bitowa ---
    bit_depth: Optional[int] = None       # None = autodetekcja z PixelSize (fallback 8-bit)
    brightness_subsample: int = 4         # podpróbkowanie przy pomiarze jasności (wydajność)

    # --- Odporność na błędy ---
    catch_genicam_errors: bool = True


class AutoExposureController:
    """
    Kontroler auto-exposure/gain dla kamery Basler (pypylon).

    Automatycznie wybiera między natywnym AE kamery (ExposureAuto/GainAuto)
    a software PI-loop, w zależności od dostępności node'ów i konfiguracji.
    """

    def __init__(self, camera, config: Optional[AEConfig] = None):
        self.camera = camera
        self.cfg = config or AEConfig()
        self._mode: Literal["native", "software", "disabled"] = "disabled"
        self._frame_count = 0
        self._integral_error = 0.0
        self._max_exposure_us = self.cfg.max_exposure_us
        self._max_gain = self.cfg.max_gain
        self._bit_depth_scale: Optional[float] = None  # przelicznik do skali 0-255, ustalany w start()

    # ------------------------------------------------------------------ #
    # Setup / start
    # ------------------------------------------------------------------ #

    def start(self):
        """Inicjalizuje kontroler: wylicza limity, głębię bitową i wybiera tryb pracy."""
        self._compute_exposure_ceiling()
        self._compute_gain_ceiling()
        self._resolve_bit_depth_scale()
        if self.cfg.enforce_frame_rate:
            self._enforce_frame_rate()

        if self.cfg.prefer_native_ae and self._native_ae_available():
            self._start_native_ae()
            self._mode = "native"
            logger.info("AutoExposureController: tryb NATIVE (kamera Basler AE/AGC)")
        else:
            self._start_software_loop_prereqs()
            self._mode = "software"
            logger.info("AutoExposureController: tryb SOFTWARE (PI-loop)")

        return self._mode

    def stop(self):
        """Wyłącza auto-exposure (przydatne np. przed zmianą ROI/PixelFormat)."""
        if self._mode == "native":
            self._safe_set(self.camera.ExposureAuto, "Off")
            self._safe_set(self.camera.GainAuto, "Off")
        self._mode = "disabled"

    # ------------------------------------------------------------------ #
    # Główna metoda wywoływana w pętli grabbingu
    # ------------------------------------------------------------------ #

    def update(self, img_array: np.ndarray):
        """
        Wywołuj raz na każdy odebrany frame. W trybie native jest to no-op
        (kamera reguluje się sama w firmware). W trybie software wykonuje
        krok regulacji PI co `update_every_n_frames` ramek.
        """
        if self._mode != "software":
            return  # native AE robi to sam w hardware; disabled -> nic nie robimy

        self._frame_count += 1
        if self._frame_count % self.cfg.update_every_n_frames != 0:
            return

        try:
            self._software_step(img_array)
        except Exception as exc:  # noqa: BLE001 - chcemy łapać też genicam.GenericException
            if self.cfg.catch_genicam_errors:
                logger.warning("AutoExposureController: błąd podczas regulacji (ignorowany): %s", exc)
            else:
                raise

    # ------------------------------------------------------------------ #
    # Native AE (Basler firmware)
    # ------------------------------------------------------------------ #

    def _native_ae_available(self) -> bool:
        cam = self.camera
        return (
            hasattr(cam, "ExposureAuto")
            and hasattr(cam, "GainAuto")
            and genicam is not None
            and genicam.IsWritable(cam.ExposureAuto)
            and genicam.IsWritable(cam.GainAuto)
        )

    def _start_native_ae(self):
        cam = self.camera

        # Próbujemy kolejnych wartości profilu aż któraś się przyjmie (różne modele).
        if hasattr(cam, "AutoFunctionProfile"):
            for profile in self.cfg.native_auto_function_profiles:
                if self._try_set(cam.AutoFunctionProfile, profile):
                    break

        # Limit górny exposure - krytyczne dla utrzymania FPS
        if hasattr(cam, "ExposureAutoUpperLimit"):
            self._safe_set(cam.ExposureAutoUpperLimit, self._max_exposure_us)
        if hasattr(cam, "ExposureAutoLowerLimit"):
            self._safe_set(cam.ExposureAutoLowerLimit, self.cfg.min_exposure_us)

        if hasattr(cam, "GainAutoUpperLimit"):
            self._safe_set(cam.GainAutoUpperLimit, self._max_gain)
        if hasattr(cam, "GainAutoLowerLimit"):
            self._safe_set(cam.GainAutoLowerLimit, self.cfg.min_gain)

        # Target brightness - różne kamery różnie nazywają/skalują ten node
        target_normalized = self.cfg.target_brightness / 255.0
        if hasattr(cam, "AutoTargetBrightness"):
            self._safe_set(cam.AutoTargetBrightness, target_normalized)
        elif hasattr(cam, "AutoTargetValue"):
            # starsze modele - zwykle skala 0-255 a nie 0-1
            self._safe_set(cam.AutoTargetValue, int(self.cfg.target_brightness))

        self._safe_set(cam.ExposureAuto, "Continuous")
        self._safe_set(cam.GainAuto, "Continuous")

    # ------------------------------------------------------------------ #
    # Software PI-loop
    # ------------------------------------------------------------------ #

    def _start_software_loop_prereqs(self):
        cam = self.camera
        # Wyłącz native auto jeśli istnieje, żeby się nie gryzły
        if hasattr(cam, "ExposureAuto"):
            self._safe_set(cam.ExposureAuto, "Off")
        if hasattr(cam, "GainAuto"):
            self._safe_set(cam.GainAuto, "Off")

        # Start od bezpiecznych wartości
        self._safe_set(cam.ExposureTime, min(self._max_exposure_us, 10000.0))
        if hasattr(cam, "Gain"):
            self._safe_set(cam.Gain, self.cfg.min_gain)

    def _software_step(self, img_array: np.ndarray):
        cam = self.camera
        brightness = self._measure_brightness(img_array)
        error = self.cfg.target_brightness - brightness

        if abs(error) < self.cfg.tolerance:
            self._integral_error = 0.0  # reset całki w martwej strefie
            return

        self._integral_error += error

        current_exp = cam.ExposureTime.GetValue()
        current_gain = cam.Gain.GetValue() if hasattr(cam, "Gain") else 0.0

        exp_correction = self.cfg.kp_exposure * error + self.cfg.ki * self._integral_error
        exp_step_limit = self.cfg.max_step_fraction * self._max_exposure_us
        exp_correction = float(np.clip(exp_correction, -exp_step_limit, exp_step_limit))

        new_exp = current_exp + exp_correction
        new_exp_clamped = float(np.clip(new_exp, self.cfg.min_exposure_us, self._max_exposure_us))
        self._safe_set(cam.ExposureTime, new_exp_clamped)

        # Gain: podbijamy gdy exposure jest przy suficie (ciemno), a redukujemy gdy
        # exposure ma zapas a gain jest niezerowy (scena się rozjaśniła) -> mniej szumu.
        at_exposure_ceiling = new_exp_clamped >= self._max_exposure_us - 1.0
        has_headroom = new_exp_clamped <= self.cfg.min_exposure_us + 1.0

        if hasattr(cam, "Gain") and (at_exposure_ceiling or (has_headroom and current_gain > self.cfg.min_gain)):
            gain_correction = self.cfg.kp_gain * error
            gain_step_limit = self.cfg.max_step_fraction * (self._max_gain - self.cfg.min_gain)
            gain_correction = float(np.clip(gain_correction, -gain_step_limit, gain_step_limit))
            new_gain = float(np.clip(current_gain + gain_correction, self.cfg.min_gain, self._max_gain))
            self._safe_set(cam.Gain, new_gain)

    def _measure_brightness(self, img_array: np.ndarray) -> float:
        # Podpróbkowanie - do regulacji AE w zupełności wystarcza, znacznie taniej.
        step = max(1, self.cfg.brightness_subsample)
        sub = img_array[::step, ::step]
        arr = sub.astype(np.float32) * (self._bit_depth_scale or 1.0)

        if self.cfg.brightness_metric == "mean":
            return float(np.mean(arr))
        return float(np.percentile(arr, self.cfg.percentile))

    # ------------------------------------------------------------------ #
    # Limity / helpers
    # ------------------------------------------------------------------ #

    def _compute_exposure_ceiling(self):
        if self.cfg.max_exposure_us is not None:
            self._max_exposure_us = self.cfg.max_exposure_us
            return
        frame_period_us = 1_000_000.0 / self.cfg.target_fps
        self._max_exposure_us = max(
            self.cfg.min_exposure_us,
            frame_period_us - self.cfg.exposure_safety_margin_us,
        )
        logger.info(
            "AutoExposureController: max_exposure_us wyliczone automatycznie = %.1f us (FPS=%.1f)",
            self._max_exposure_us,
            self.cfg.target_fps,
        )

    def _compute_gain_ceiling(self):
        if self.cfg.max_gain is not None:
            self._max_gain = self.cfg.max_gain
            return
        try:
            self._max_gain = self.camera.Gain.GetMax()
        except Exception:  # noqa: BLE001
            self._max_gain = 24.0  # bezpieczna wartość domyślna jeśli node niedostępny

    def _resolve_bit_depth_scale(self):
        """
        Ustala przelicznik do skali 0-255 RAZ, na podstawie konfiguracji kamery
        (PixelSize), a nie zawartości obrazu - dzięki temu ciemna pierwsza klatka
        kamery 10/12-bit nie zafałszuje skalowania na cały czas pracy.
        """
        bits = self.cfg.bit_depth
        if bits is None:
            try:
                # PixelSize zwykle ma postać "Bpp8" / "Bpp10" / "Bpp12"
                raw = str(self.camera.PixelSize.GetValue())
                bits = int("".join(ch for ch in raw if ch.isdigit()))
            except Exception:  # noqa: BLE001
                bits = None

        if bits and bits > 8:
            self._bit_depth_scale = 255.0 / (2 ** bits - 1)
        else:
            self._bit_depth_scale = 1.0

    def _enforce_frame_rate(self):
        """
        Faktyczne zablokowanie FPS. Ograniczenie samego ExposureTime nie gwarantuje
        stałej częstotliwości - trzeba włączyć AcquisitionFrameRateEnable i ustawić
        AcquisitionFrameRate (starsze modele: AcquisitionFrameRateAbs).
        """
        cam = self.camera
        if hasattr(cam, "AcquisitionFrameRateEnable"):
            self._safe_set(cam.AcquisitionFrameRateEnable, True)
        rate_node = getattr(cam, "AcquisitionFrameRate",
                            getattr(cam, "AcquisitionFrameRateAbs", None))
        if rate_node is not None:
            self._safe_set(rate_node, self.cfg.target_fps)

    def _try_set(self, node, value) -> bool:
        """Jak _safe_set, ale zwraca True/False czy się udało (do próbowania wartości enum)."""
        try:
            if genicam is not None and not genicam.IsWritable(node):
                return False
            node.SetValue(value)
            return True
        except Exception:  # noqa: BLE001
            return False

    def _safe_set(self, node, value):
        """Ustawia node GenICam, łapiąc błędy access-mode / zakresu, jeśli skonfigurowano."""
        try:
            if genicam is not None and not genicam.IsWritable(node):
                logger.debug("Node nie jest writable w tym stanie kamery - pomijam")
                return
            node.SetValue(value)
        except Exception as exc:  # noqa: BLE001
            if self.cfg.catch_genicam_errors:
                logger.warning("Nie udało się ustawić wartości %s: %s", value, exc)
            else:
                raise
