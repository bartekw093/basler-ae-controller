# basler-ae-controller

Plug-and-play kontroler **auto-exposure / auto-gain** dla kamer **Basler** (pypylon),
zaprojektowany pod **stałe FPS** (np. 25 fps) z ograniczeniem czasowym na `ExposureTime`.

Moduł działa z dowolną aplikacją, ponieważ:

- nie zakłada żadnego konkretnego pipeline'u przetwarzania obrazu,
- automatycznie wykrywa, czy kamera ma `ExposureAuto`/`GainAuto` i **preferuje natywne AE** (firmware),
- ma **fallback na software PI-loop**, gdy natywne AE jest niedostępne lub wyłączone,
- wszystkie parametry są konfigurowalne przez dataclass `AEConfig`,
- jest **odporny na błędy GenICam** – okazjonalny wyjątek nie wywala pipeline'u.

## Wymagania

- Python 3.9+
- [`pypylon`](https://github.com/basler/pypylon)
- `numpy`

```bash
pip install pypylon numpy
```

> `numpy` jest twardą zależnością. `pypylon` jest wymagane do pracy z kamerą,
> ale sam moduł można zaimportować bez niego (np. do testów typowania).

## Użycie

```python
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
        ...                     # dalsze przetwarzanie
    grab_result.Release()
```

## Jak to działa

Przy `start()`:

1. wyliczany jest górny limit ekspozycji z `target_fps`
   (`max_exposure_us = 1e6 / fps − margines_bezpieczeństwa`),
2. ustalana jest głębia bitowa (z `PixelSize`) do normalizacji jasności do skali 0–255,
3. (opcjonalnie) wymuszany jest stały FPS przez `AcquisitionFrameRateEnable` + `AcquisitionFrameRate`,
4. wybierany jest tryb pracy:
   - **NATIVE** – ustawia `ExposureAuto`/`GainAuto = Continuous` wraz z limitami i targetem jasności;
     regulacją zajmuje się firmware kamery (`update()` jest wtedy no-op),
   - **SOFTWARE** – prosty regulator PI: koryguje `ExposureTime`, a `Gain` rusza dopiero,
     gdy ekspozycja jest przy suficie (scena ciemna) – i redukuje gain, gdy ekspozycja ma zapas
     (mniej szumu).

## Kluczowe parametry `AEConfig`

| Parametr | Domyślnie | Opis |
|---|---|---|
| `target_fps` | `25.0` | docelowa liczba klatek/s (z niej liczony jest limit ekspozycji) |
| `target_brightness` | `128.0` | docelowa jasność w skali 0–255 |
| `brightness_metric` | `"mean"` | `"mean"` lub `"percentile"` |
| `enforce_frame_rate` | `True` | twarde ustawienie `AcquisitionFrameRate` na kamerze |
| `prefer_native_ae` | `True` | preferuj natywne AE kamery, gdy dostępne |
| `min_exposure_us` / `max_exposure_us` | `100` / auto | limity ekspozycji (µs); `None` = auto z FPS |
| `min_gain` / `max_gain` | `0` / auto | limity gainu; `None` = z kamery (`GainMax`) |
| `bit_depth` | `None` | wymuszenie głębi bitowej; `None` = autodetekcja |

Pełna lista pól znajduje się w `AEConfig` w pliku
[`basler_auto_exposure.py`](basler_auto_exposure.py).

## Uwagi sprzętowe

- Nazwy/zakresy node'ów GenICam różnią się między modelami Baslera. Moduł próbuje kilku
  wariantów (np. `AutoFunctionProfile`, `AcquisitionFrameRate` vs `AcquisitionFrameRateAbs`)
  i bezpiecznie pomija te niedostępne.
- `enforce_frame_rate=True` modyfikuje konfigurację kamery w `start()`. Ustaw `False`,
  jeśli częstotliwością klatek ma zarządzać aplikacja hosta.
