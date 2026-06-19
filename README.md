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

## Praca na zewnątrz 24/7

Moduł jest projektowany pod kamerę pracującą na zewnątrz przez całą dobę – od nocy
po ostre słońce. Regulator software (zalecany dla trudnych scen: `prefer_native_ae=False`)
radzi sobie z tym dzięki:

- **regulacji multiplikatywnej w domenie log (EV)** – światło zmienia się wykładniczo,
  więc korekta jest proporcjonalna („o ile stopów"), a nie addytywna. To samo prawo
  regulacji obejmuje cały zakres od nocy po słońce;
- **wspólnemu budżetowi światła** `czas × gain_liniowy` – najpierw maksymalizowany jest
  czas ekspozycji (najmniej szumu), a dopiero nadwyżkę przejmuje gain (noc);
- **ochronie highlightów** – udział spalonych pikseli jest mierzony i ograniczany
  (spalone niebo niszczy dane, choć nie podbija średniej);
- **balansowi highlightów i cieni** dla scen o wysokim kontraście (patrz niżej);
- **asymetrycznym krokom** – ciemnienie jest szybsze niż rozjaśnianie (prześwietlenie
  jest groźniejsze niż chwilowe niedoświetlenie);
- **wygładzaniu czasowemu (EMA)** – tłumi migotanie od reflektorów / przejeżdżających aut.

### Sceny o wysokim zakresie dynamicznym (jednocześnie słońce i głęboki cień)

Gdy w jednym kadrze jest naraz spalony fragment i zatopiony cień, **żadna ekspozycja
nie naświetli idealnie obu stref** – to fizyczne ograniczenie zakresu dynamiki sensora.
W trybie `metering="balanced"` (domyślny) regulator zamiast trzymać samą średnią mierzy
**jednocześnie udział highlightów i udział cieni** i ustawia ekspozycję w punkcie równowagi:

- nadmiar highlightów → wymusza minimalne ciemnienie,
- nadmiar cieni → wymusza minimalne rozjaśnienie,
- gdy oba naraz (clipping nieunikniony) → bierze **kompromis ważony** parametrem
  `highlight_priority` (domyślnie highlighty są ~2,5× ważniejsze, bo utrata danych
  w bieli jest nieodwracalna).

Tryb `metering="global"` wyłącza balans cieni i reguluje do średniej z samą ochroną highlightów.

## Jak to działa

Przy `start()`:

1. wyliczany jest górny limit ekspozycji z `target_fps`
   (`max_exposure_us = 1e6 / fps − margines_bezpieczeństwa`),
2. ustalana jest głębia bitowa (z `PixelSize`) do normalizacji jasności do skali 0–255,
3. (opcjonalnie) wymuszany jest stały FPS przez `AcquisitionFrameRateEnable` + `AcquisitionFrameRate`,
4. wybierany jest tryb pracy:
   - **NATIVE** – ustawia `ExposureAuto`/`GainAuto = Continuous` wraz z limitami i targetem jasności;
     regulacją zajmuje się firmware kamery (`update()` jest wtedy no-op),
   - **SOFTWARE** – regulator opisany wyżej (log-domena + budżet światła + balans highlightów/cieni).

### Przykładowa konfiguracja pod trudny outdoor

```python
cfg = AEConfig(
    target_fps=25,
    prefer_native_ae=False,     # użyj software loop z ochroną highlightów
    metering="balanced",
    target_brightness=110,
    highlight_priority=2.5,     # mocniej chroń biele niż czernie
    max_saturated_fraction=0.02,
    max_shadow_fraction=0.10,
)
```

## Kluczowe parametry `AEConfig`

| Parametr | Domyślnie | Opis |
|---|---|---|
| `target_fps` | `25.0` | docelowa liczba klatek/s (z niej liczony jest limit ekspozycji) |
| `target_brightness` | `128.0` | docelowa jasność w skali 0–255 |
| `brightness_metric` | `"mean"` | `"mean"` lub `"percentile"` |
| `metering` | `"balanced"` | `"balanced"` (balans highlightów/cieni) lub `"global"` |
| `highlight_priority` | `2.5` | ile razy ważniejsza ochrona bieli niż czerni przy HDR |
| `max_saturated_fraction` | `0.02` | dopuszczalny udział spalonych highlightów |
| `max_shadow_fraction` | `0.10` | dopuszczalny udział zatopionych cieni |
| `enforce_frame_rate` | `True` | twarde ustawienie `AcquisitionFrameRate` na kamerze |
| `prefer_native_ae` | `True` | preferuj natywne AE kamery, gdy dostępne |
| `gain_is_db` | `True` | czy `Gain` jest w dB (typowe Basler) czy liniowy |
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
