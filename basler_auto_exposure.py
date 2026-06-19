"""
basler_auto_exposure.py

Plug-and-play kontroler auto-exposure / auto-gain dla kamer Basler (pypylon),
zaprojektowany pod stałe FPS (np. 25fps) z ograniczeniem czasowym na ExposureTime.

Zoptymalizowany pod pracę NA ZEWNĄTRZ 24/7: od nocy (długa ekspozycja + gain)
po ostre słońce (krótka ekspozycja, gain 0) i prześwietlenie dużej części kadru.

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

Dla trudnego outdooru zalecany jest tryb SOFTWARE (prefer_native_ae=False),
bo regulator software ma ochronę highlightów - native AE Baslera celuje w
średnią i potrafi spalić niebo albo zrobić noc zbyt ciemną.

Działa z dowolną aplikacją bo:
- nie zakłada żadnego konkretnego pipeline'u przetwarzania obrazu
- automatycznie wykrywa czy kamera ma ExposureAuto/GainAuto i preferuje native AE
- fallback na software loop (regulacja multiplikatywna w domenie log + ochrona highlightów)
- wszystkie parametry konfigurowalne przez dataclass AEConfig
- bezpieczne wobec błędów GenICam (nie wywala pipeline'u przy okazjonalnym rzucie wyjątku)
"""

from __future__ import annotations

import math
import logging
from dataclasses import dataclass
from typing import Optional, Literal, Tuple

import numpy as np

try:
    from pypylon import genicam
except ImportError:
    genicam = None  # pozwala importować moduł bez pypylon (np. do testów typowania)

logger = logging.getLogger("basler_auto_exposure")

_LN2 = math.log(2.0)


@dataclass
class AEConfig:
    """Konfiguracja kontrolera auto-exposure/gain."""

    # --- Cel regulacji ---
    target_fps: float = 25.0
    target_brightness: float = 110.0      # cel w skali 0-255; nieco poniżej środka -> bezpieczniej dla highlightów
    tolerance: float = 6.0                # martwa strefa wokół celu (skala 0-255) - zapobiega "drganiu"
    brightness_metric: Literal["mean", "percentile"] = "mean"
    percentile: float = 50.0              # używane gdy brightness_metric == "percentile" (mediana = odporna na clipping)

    # --- Limity exposure (w mikrosekundach) ---
    exposure_safety_margin_us: float = 2000.0  # margines na readout/transfer poniżej okresu ramki
    min_exposure_us: float = 50.0         # niżej -> obrona przed prześwietleniem w ostrym słońcu
    max_exposure_us: Optional[float] = None     # None = wyliczane automatycznie z target_fps

    # --- Limity gain (jednostki zależne od kamery - zwykle dB) ---
    min_gain: float = 0.0
    max_gain: Optional[float] = None      # None = czyta z kamery (GainMax)
    gain_is_db: bool = True               # True: Gain w dB (typowe Basler ace); False: jednostki liniowe

    # --- Tryb pracy ---
    prefer_native_ae: bool = True         # jeśli kamera wspiera ExposureAuto/GainAuto - użyj go
    # Różne modele Baslera używają różnych stringów dla AutoFunctionProfile -
    # próbujemy po kolei aż któryś zostanie zaakceptowany. "MinimizeGain" = priorytet exposure (mniej szumu).
    native_auto_function_profiles: tuple = ("MinimizeGain", "Gain", "GainMinimum")

    # Twarde wymuszenie FPS - bez tego Basler niekoniecznie utrzymuje stałą częstotliwość.
    enforce_frame_rate: bool = True

    # --- Regulator software (używany gdy native AE niedostępny / wyłączony) ---
    update_every_n_frames: int = 2        # co ile ramek liczyć krok regulacji
    ema_alpha: float = 0.4                # wygładzanie pomiaru jasności (0-1; mniej = gładziej, wolniej)
    max_step_ev_up: float = 0.7           # max krok rozjaśniania na iterację [EV/stops]
    max_step_ev_down: float = 2.0         # max krok ciemnienia [EV] - większy, bo highlighty są groźniejsze

    # --- Metering: balans highlightów i cieni (sceny o wysokim kontraście) ---
    # "global"   = regulacja do średniej + ochrona highlightów
    # "balanced" = szukanie równowagi między spalonymi highlightami a zatopionymi cieniami
    #              (np. jednocześnie słońce i głęboki cień w jednym kadrze)
    metering: Literal["global", "balanced"] = "balanced"

    # Highlighty (spalone niebo / słońce)
    saturation_threshold: float = 250.0   # piksel >= tej wartości (skala 0-255) liczony jako nasycony
    max_saturated_fraction: float = 0.02  # dopuszczalny udział nasyconych pikseli (2%)
    highlight_recovery_gain: float = 6.0  # jak agresywnie ciemnieć przy nadmiarze highlightów

    # Cienie (zatopiona czerń)
    shadow_threshold: float = 16.0        # piksel <= tej wartości liczony jako zatopiony cień
    max_shadow_fraction: float = 0.10     # dopuszczalny udział pikseli w cieniu (10%)
    shadow_recovery_gain: float = 4.0     # jak agresywnie rozjaśniać przy nadmiarze cieni

    # Przy nieuniknionym dwustronnym clippingu (HDR) - ile razy ważniejsza jest
    # ochrona highlightów niż cieni przy szukaniu kompromisu.
    highlight_priority: float = 2.5

    # --- Pomiar jasności / głębia bitowa ---
    bit_depth: Optional[int] = None       # None = autodetekcja z PixelSize (fallback 8-bit)
    brightness_subsample: int = 4         # podpróbkowanie przy pomiarze jasności (wydajność)

    # --- Odporność na błędy ---
    catch_genicam_errors: bool = True


class AutoExposureController:
    """
    Kontroler auto-exposure/gain dla kamery Basler (pypylon).

    Automatycznie wybiera między natywnym AE kamery (ExposureAuto/GainAuto)
    a software loop, w zależności od dostępności node'ów i konfiguracji.

    Software loop reguluje multiplikatywnie w domenie logarytmicznej: traktuje
    iloczyn (czas_ekspozycji * gain_liniowy) jako wspólny "budżet światła",
    skaluje go przez (target / zmierzona_jasność) i rozkłada z powrotem -
    najpierw na czas (mało szumu), nadwyżkę na gain (noc).
    """

    def __init__(self, camera, config: Optional[AEConfig] = None):
        self.camera = camera
        self.cfg = config or AEConfig()
        self._mode: Literal["native", "software", "disabled"] = "disabled"
        self._frame_count = 0
        self._brightness_ema: Optional[float] = None
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
            logger.info("AutoExposureController: tryb SOFTWARE (regulacja log + ochrona highlightów)")

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
        krok regulacji co `update_every_n_frames` ramek.
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
    # Software loop (regulacja multiplikatywna w domenie log)
    # ------------------------------------------------------------------ #

    def _start_software_loop_prereqs(self):
        cam = self.camera
        # Wyłącz native auto jeśli istnieje, żeby się nie gryzły
        if hasattr(cam, "ExposureAuto"):
            self._safe_set(cam.ExposureAuto, "Off")
        if hasattr(cam, "GainAuto"):
            self._safe_set(cam.GainAuto, "Off")

        # Start od bezpiecznych wartości
        self._safe_set(cam.ExposureTime, min(self._max_exposure_us, 5000.0))
        if hasattr(cam, "Gain"):
            self._safe_set(cam.Gain, self.cfg.min_gain)

    def _software_step(self, img_array: np.ndarray):
        cam = self.camera
        brightness, hi_fraction, lo_fraction = self._measure(img_array)

        # Wygładzanie czasowe - tłumi migotanie (reflektory, przejeżdżające auta).
        if self._brightness_ema is None:
            self._brightness_ema = brightness
        else:
            a = self.cfg.ema_alpha
            self._brightness_ema = a * brightness + (1.0 - a) * self._brightness_ema
        smoothed = self._brightness_ema

        hi_excess = max(0.0, hi_fraction - self.cfg.max_saturated_fraction)
        lo_excess = max(0.0, lo_fraction - self.cfg.max_shadow_fraction)
        clipping_active = (hi_excess > 0.0) or (lo_excess > 0.0)

        # Martwa strefa: blisko celu i bez nadmiaru clippingu -> nie ruszamy nic.
        if not clipping_active and abs(smoothed - self.cfg.target_brightness) < self.cfg.tolerance:
            return

        # Pożądana korekta budżetu światła w domenie log (EV w jednostkach naturalnych).
        log_ratio = self._compute_log_ratio(smoothed, hi_excess, lo_excess)

        # Ograniczenie kroku w domenie log (EV). Asymetryczne: szybciej ciemniej.
        max_up = self.cfg.max_step_ev_up * _LN2
        max_down = self.cfg.max_step_ev_down * _LN2
        log_ratio = max(-max_down, min(max_up, log_ratio))
        ratio = math.exp(log_ratio)

        # Budżet światła = czas * gain_liniowy.
        has_gain = hasattr(cam, "Gain")
        current_exp = cam.ExposureTime.GetValue()
        current_gain = cam.Gain.GetValue() if has_gain else self.cfg.min_gain
        lin_gain_min = self._gain_to_linear(self.cfg.min_gain)
        lin_gain_max = self._gain_to_linear(self._max_gain) if has_gain else lin_gain_min
        current_total = current_exp * self._gain_to_linear(current_gain)
        new_total = current_total * ratio

        # Rozkład budżetu: najpierw maksymalizuj czas przy minimalnym gainie
        # (najmniej szumu), nadwyżkę dopiero przerzuć na gain (noc).
        desired_exp = new_total / lin_gain_min
        new_exp = float(np.clip(desired_exp, self.cfg.min_exposure_us, self._max_exposure_us))
        self._safe_set(cam.ExposureTime, new_exp)

        if has_gain:
            needed_lin_gain = new_total / max(new_exp, 1e-6)
            new_lin_gain = float(np.clip(needed_lin_gain, lin_gain_min, lin_gain_max))
            new_gain = self._linear_to_gain(new_lin_gain)
            self._safe_set(cam.Gain, new_gain)

    def _compute_log_ratio(self, smoothed: float, hi_excess: float, lo_excess: float) -> float:
        """
        Wylicza pożądaną korektę budżetu światła (log naturalny; >0 = jaśniej, <0 = ciemniej).

        Idea balansu:
        - bez nadmiernego clippingu -> klasyczna regulacja do średniej (target_brightness),
        - nadmiar highlightów -> górne ograniczenie: WYMUSZ co najmniej tyle ciemnienia,
        - nadmiar cieni       -> dolne ograniczenie: WYMUSZ co najmniej tyle rozjaśnienia,
        - oba naraz (HDR, clipping nieunikniony) -> żądania są sprzeczne, więc bierzemy
          kompromis ważony priorytetem highlightów. To właśnie "znalezienie balansu":
          ekspozycja siada między spaleniem bieli a zatopieniem czerni.
        """
        cfg = self.cfg
        base = math.log(cfg.target_brightness / max(smoothed, 1.0))

        if cfg.metering == "global":
            # Tylko ochrona highlightów (bez balansu cieni).
            if hi_excess > 0.0:
                base = min(base, -cfg.highlight_recovery_gain * hi_excess)
            return base

        # metering == "balanced"
        upper = math.inf   # górny limit log_ratio (highlighty pchają w dół)
        lower = -math.inf  # dolny limit log_ratio (cienie pchają w górę)
        if hi_excess > 0.0:
            upper = -cfg.highlight_recovery_gain * hi_excess
        if lo_excess > 0.0:
            lower = cfg.shadow_recovery_gain * lo_excess

        if lower > upper:
            # Sprzeczność (dwustronny clipping) -> kompromis ważony.
            w = cfg.highlight_priority
            return (w * upper + lower) / (w + 1.0)

        return float(np.clip(base, lower, upper))

    def _measure(self, img_array: np.ndarray) -> Tuple[float, float, float]:
        """Zwraca (jasność 0-255, udział highlightów 0-1, udział cieni 0-1)."""
        step = max(1, self.cfg.brightness_subsample)
        sub = img_array[::step, ::step]
        arr = sub.astype(np.float32) * (self._bit_depth_scale or 1.0)

        if self.cfg.brightness_metric == "mean":
            brightness = float(np.mean(arr))
        else:
            brightness = float(np.percentile(arr, self.cfg.percentile))

        hi_fraction = float(np.mean(arr >= self.cfg.saturation_threshold))
        lo_fraction = float(np.mean(arr <= self.cfg.shadow_threshold))
        return brightness, hi_fraction, lo_fraction

    # ------------------------------------------------------------------ #
    # Konwersje gain
    # ------------------------------------------------------------------ #

    def _gain_to_linear(self, gain: float) -> float:
        if self.cfg.gain_is_db:
            return 10.0 ** (gain / 20.0)
        return max(gain, 1e-6)

    def _linear_to_gain(self, lin: float) -> float:
        if self.cfg.gain_is_db:
            return 20.0 * math.log10(max(lin, 1e-6))
        return lin

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
