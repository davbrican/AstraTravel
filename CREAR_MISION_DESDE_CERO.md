# Crear una mision desde cero

Esta guia explica como construir una mision nueva dentro de `v3_spaceship_missions`: crear una nave, colocarla en la Tierra, decidir cuando acelerar, hacia donde apuntar, como consumir combustible, como cambia la masa y como avanzar la simulacion.

El proyecto ya tiene implementadas las piezas fisicas principales. Tu trabajo al crear una mision es conectarlas y decidir la logica de vuelo.

## Idea general

Una mision necesita estas piezas:

1. Un sistema solar inicial.
2. Una nave con etapas, tanques y motores.
3. Una posicion inicial, por ejemplo en la superficie de la Tierra.
4. Un sistema de decision:
   - por eventos fijos en el tiempo, usando `MissionTimeline`
   - o por estado fisico, usando un controlador tipo `TransLunarMissionController`
5. Un motor de simulacion que avance el tiempo.
6. Diagnosticos para saber altitud, velocidad, combustible, masa y fase.

Flujo minimo:

```text
crear sistema solar
crear nave
colocar nave en la Tierra
crear controlador o timeline
crear reloj
repetir:
    avanzar simulacion
    actualizar guiado
    aplicar gravedad
    aplicar empuje
    consumir combustible
    actualizar masa
    leer diagnosticos
```

## Archivos importantes

### `spacecraft_models.py`

Aqui estan:

- `Spacecraft`
- `Stage`
- `Tank`
- `Engine`
- `GuidanceMode`

Este archivo se encarga de:

- calcular masa total de la nave
- calcular combustible restante
- encender y apagar motores
- separar etapas
- calcular empuje
- calcular aceleracion por empuje
- consumir propelente
- orientar la nave hacia un vector objetivo

### `mission_engine.py`

Motor especifico para lanzamiento desde la Tierra.

Usa:

- gravedad N-body
- empuje
- consumo de combustible
- drag atmosferico
- controlador de mision

Funcion principal:

```python
step_launch_mission_simulation(...)
```

Es el motor recomendado si la nave sale desde la Tierra.

### `launch_mission_profiles.py`

Aqui esta `TransLunarMissionController`, que es un ejemplo de controlador automatico por fases.

Tambien contiene:

- `MissionPhase`
- `LaunchVehicleProgram`
- `compute_earth_relative_snapshot`

Este archivo muestra como tomar decisiones segun:

- altitud
- velocidad
- velocidad circular
- apoapsis
- combustible restante
- distancia a la Luna
- fase actual

### `mission_timeline.py`

Sistema generico de eventos programados.

Sirve para decir:

```text
t = 0 s: encender motor
t = 60 s: cambiar direccion
t = 120 s: apagar motor
t = 125 s: separar etapa
```

Es facil de entender, pero menos robusto que un controlador por estado fisico.

### `mission_scenarios.py`

Monta escenarios completos. Es buen sitio para mirar ejemplos de vehiculos:

- Apollo-like
- Artemis-like

Tambien muestra como se crea una mision completa con:

```python
create_translunar_launch_scenario(...)
```

## Conceptos fisicos ya implementados

### Empuje

Cada motor tiene un empuje maximo:

```python
Engine(
    name="MainEngine",
    max_thrust_newtons=7_000_000.0,
    specific_impulse_seconds=360.0,
    tank_names=["MainTank"],
)
```

Si el motor esta activo, produce:

```text
thrust = max_thrust * throttle
```

### Consumo de combustible

El consumo se calcula con:

```text
mass_flow = thrust / (Isp * g0)
```

Donde:

- `thrust` esta en newtons
- `Isp` es el impulso especifico en segundos
- `g0` es 9.80665 m/s^2
- `mass_flow` sale en kg/s

Esto ya esta implementado en:

```python
Engine.current_mass_flow_kg_s()
Spacecraft.consume_propellant(dt_seconds)
```

### Masa variable

La nave no tiene una masa fija. Su masa total se calcula dinamicamente:

```python
spacecraft.total_mass_kg
```

Esa masa depende de:

- etapas no separadas
- masa seca de cada etapa
- combustible restante en cada tanque

Cuando se consume combustible, baja `current_mass_kg` de los tanques. Como `total_mass_kg` se calcula en cada lectura, la masa total baja automaticamente.

Cuando se separa una etapa:

```python
spacecraft.jettison_stage("Stage1")
```

esa etapa deja de contar para la masa total.

### Aceleracion por empuje

La aceleracion sale de:

```text
acceleration = thrust / current_mass
```

En el codigo:

```python
Spacecraft.compute_thrust_acceleration_km_s2()
```

Como la masa baja al consumir combustible, la misma fuerza genera cada vez mas aceleracion.

### Orientacion de la nave

La nave puede apuntar usando `GuidanceMode`.

Modos utiles:

```python
GuidanceMode.TARGET_VECTOR
GuidanceMode.PROGRADE
GuidanceMode.RETROGRADE
GuidanceMode.RADIAL_OUT
GuidanceMode.RADIAL_IN
GuidanceMode.MANUAL
```

Ejemplo:

```python
spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, Vector3(1.0, 0.0, 0.0))
spacecraft.update_guidance()
```

Importante: la orientacion esta idealizada. La nave rota instantaneamente hacia el vector objetivo. No hay dinamica real de actitud, torque, inercia ni limits de giro.

## Crear una nave

Una nave se crea con etapas. Cada etapa tiene:

- masa seca
- tanques
- motores

Ejemplo de una nave simple de una etapa:

```python
from spacecraft_models import Engine, PropellantType, Spacecraft, Stage, Tank


def create_simple_vehicle(parent):
    stage1 = Stage(
        name="Stage1",
        dry_mass_kg=50_000.0,
        tanks=[
            Tank(
                name="MainTank",
                propellant_type=PropellantType.LH2,
                capacity_kg=400_000.0,
                current_mass_kg=400_000.0,
            )
        ],
        engines=[
            Engine(
                name="MainEngine",
                max_thrust_newtons=7_000_000.0,
                specific_impulse_seconds=360.0,
                tank_names=["MainTank"],
            )
        ],
    )

    return Spacecraft(
        name="MyVehicle",
        stages=[stage1],
        dry_radius_km=5.0,
        color_hex="#FFAA55",
        parent=parent,
        metadata={
            "drag_coefficient": 0.45,
            "reference_area_m2": 80.0,
        },
    )
```

### Nave de varias etapas

Ejemplo:

```python
def create_two_stage_vehicle(parent):
    stage1 = Stage(
        name="Stage1",
        dry_mass_kg=80_000.0,
        tanks=[
            Tank("Stage1Tank", PropellantType.RP1, 600_000.0, 600_000.0),
        ],
        engines=[
            Engine("Stage1Engine", 9_000_000.0, 300.0, ["Stage1Tank"]),
        ],
    )

    stage2 = Stage(
        name="Stage2",
        dry_mass_kg=15_000.0,
        tanks=[
            Tank("Stage2Tank", PropellantType.LH2, 120_000.0, 120_000.0),
        ],
        engines=[
            Engine("Stage2Engine", 1_200_000.0, 440.0, ["Stage2Tank"]),
        ],
    )

    return Spacecraft(
        name="MyTwoStageVehicle",
        stages=[stage1, stage2],
        dry_radius_km=5.0,
        color_hex="#FFAA55",
        parent=parent,
        metadata={
            "drag_coefficient": 0.45,
            "reference_area_m2": 90.0,
        },
    )
```

## Crear el sistema solar

```python
from solar_system_factory_nbody import create_default_solar_system_nbody

scene = create_default_solar_system_nbody()
root = scene.root
body_index = scene.body_index

earth = body_index["Earth"]
moon = body_index["Moon"]
```

El `root` es el nodo principal de la simulacion. La nave debe colgar de `root` para participar correctamente en el motor N-body actual.

## Colocar la nave en la Tierra

La forma recomendada es usar el metodo ya existente:

```python
controller.place_on_launch_pad(spacecraft, earth, moon)
```

Ese metodo:

1. Calcula un plano de lanzamiento usando la Tierra y la Luna.
2. Coloca la nave justo encima de la superficie terrestre.
3. Le da la velocidad heliocentrica de la Tierra.
4. Anade velocidad tangencial aproximada por rotacion terrestre.
5. Orienta la nave hacia arriba.

Ejemplo:

```python
from launch_mission_profiles import LaunchVehicleProgram, TransLunarMissionController

program = LaunchVehicleProgram(
    ascent_stage_names=["Stage1"],
    ascent_engine_names=["MainEngine"],
    parking_stage_name=None,
    parking_engine_name=None,
    service_stage_name=None,
    service_engine_name=None,
)

controller = TransLunarMissionController(
    spacecraft_name=spacecraft.name,
    program=program,
)

controller.place_on_launch_pad(spacecraft, earth, moon)
```

### Colocacion manual

Si quieres hacerlo manualmente:

```python
from spacecraft_models import GuidanceMode
from space_simulation_models import Vector3

radial = Vector3(1.0, 0.0, 0.0)
tangential = Vector3(0.0, 1.0, 0.0)

launch_altitude_km = 0.1
earth_rotation_speed_km_s = 0.465

spacecraft.set_local_position(
    earth.global_position + radial * (earth.radius_km + launch_altitude_km)
)

spacecraft.local_velocity_km_s = (
    earth.local_velocity_km_s + tangential * earth_rotation_speed_km_s
)

spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, radial)
spacecraft.update_guidance()
```

La colocacion manual es util para pruebas, pero para misiones reales dentro de este proyecto conviene reutilizar `place_on_launch_pad`.

## Decidir hacia donde acelerar

Durante un lanzamiento desde la Tierra, necesitas dos direcciones:

```text
radial     = hacia arriba desde el centro de la Tierra
tangential = horizontal, direccion de avance orbital
```

El gravity turn mezcla ambas.

Al inicio:

```text
100% radial, 0% tangential
```

Luego:

```text
70% radial, 30% tangential
50% radial, 50% tangential
20% radial, 80% tangential
0% radial, 100% tangential
```

En codigo:

```python
target = radial * 0.7 + tangential * 0.3
spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, target.normalized())
spacecraft.update_guidance()
```

El controlador actual lo hace con:

```python
_gravity_turn_target(...)
```

en `launch_mission_profiles.py`.

## Opcion A: mision por timeline

Usa `MissionTimeline` si quieres programar acciones por tiempo.

Ejemplo:

```python
from mission_timeline import (
    JettisonStageAction,
    MissionTimeline,
    ScheduledMissionEvent,
    SetGuidanceAction,
    StartBurnAction,
    StopBurnAction,
)
from spacecraft_models import GuidanceMode
from space_simulation_models import Vector3


timeline = MissionTimeline()

timeline.add_event(
    ScheduledMissionEvent(
        trigger_time_seconds=0.0,
        name="liftoff",
        action=StartBurnAction(
            spacecraft_name="MyVehicle",
            engine_name="MainEngine",
            throttle=1.0,
            guidance_mode=GuidanceMode.TARGET_VECTOR,
            target_vector=Vector3(1.0, 0.0, 0.0),
        ),
    )
)

timeline.add_event(
    ScheduledMissionEvent(
        trigger_time_seconds=60.0,
        name="pitch_program",
        action=SetGuidanceAction(
            spacecraft_name="MyVehicle",
            guidance_mode=GuidanceMode.TARGET_VECTOR,
            target_vector=Vector3(0.7, 0.7, 0.0),
        ),
    )
)

timeline.add_event(
    ScheduledMissionEvent(
        trigger_time_seconds=150.0,
        name="main_engine_cutoff",
        action=StopBurnAction(
            spacecraft_name="MyVehicle",
            engine_name="MainEngine",
        ),
    )
)

timeline.add_event(
    ScheduledMissionEvent(
        trigger_time_seconds=155.0,
        name="stage_sep",
        action=JettisonStageAction(
            spacecraft_name="MyVehicle",
            stage_name="Stage1",
        ),
    )
)
```

Luego avanzas con:

```python
from mission_simulation_engine import step_mission_simulation
from nbody_engine import SimulationClock

clock = SimulationClock(current_time_seconds=0.0, time_scale=1.0)
spacecraft_index = {spacecraft.name: spacecraft}

while clock.current_time_seconds < 1000.0:
    diagnostics = step_mission_simulation(
        root=root,
        clock=clock,
        timeline=timeline,
        spacecraft_index=spacecraft_index,
        real_delta_seconds=1.0,
    )
```

### Ventajas del timeline

- Es facil de entender.
- Es bueno para pruebas.
- Es bueno para maniobras sencillas.
- Es bueno cuando ya sabes los tiempos exactos.

### Limitaciones del timeline

El timeline no mira si la nave alcanzo la altitud correcta o si va demasiado lenta. Ejecuta acciones por tiempo, aunque la fisica haya salido distinta.

Por ejemplo:

```text
t=150s apagar motor
```

se ejecuta siempre a los 150 segundos, incluso si la nave todavia no llego a orbita.

## Opcion B: mision con controlador por estado fisico

Para una mision seria de lanzamiento, lo mejor es un controlador por fases.

La idea:

```text
si fase == ASCENT_STAGE_1:
    apuntar segun gravity turn
    si etapa 1 sin combustible:
        apagar motor
        separar etapa
        encender etapa 2
        cambiar a ASCENT_STAGE_2

si fase == ASCENT_STAGE_2:
    apuntar mas horizontal
    si altitud y velocidad suficientes:
        apagar motor
        cambiar a PARKING_COAST

si fase == PARKING_COAST:
    esperar
    si toca TLI:
        apuntar hacia Luna futura
        encender motor
        cambiar a TLI_BURN
```

Este enfoque es mejor porque el "cuando" depende del estado fisico:

- cuando altitud >= objetivo
- cuando velocidad >= objetivo
- cuando apoapsis >= objetivo
- cuando combustible <= limite
- cuando distancia a Luna <= limite
- cuando velocidad radial sea pequena

El ejemplo ya implementado esta en:

```python
TransLunarMissionController.update(...)
```

en `launch_mission_profiles.py`.

## Leer el estado orbital

Para decidir si apagar, redirigir o cambiar fase, usa:

```python
from launch_mission_profiles import compute_earth_relative_snapshot

snapshot = compute_earth_relative_snapshot(spacecraft, earth)
```

Te da:

```python
snapshot.altitude_km
snapshot.speed_km_s
snapshot.radial_speed_km_s
snapshot.circular_speed_km_s
snapshot.specific_energy_km2_s2
snapshot.eccentricity
snapshot.semimajor_axis_km
snapshot.periapsis_altitude_km
snapshot.apoapsis_altitude_km
```

Ejemplo de regla:

```python
reached_parking = (
    snapshot.altitude_km >= 180.0
    and snapshot.speed_km_s >= snapshot.circular_speed_km_s * 0.97
    and abs(snapshot.radial_speed_km_s) <= 0.35
)

if reached_parking:
    spacecraft.shutdown_all_engines()
```

## Crear un programa de lanzamiento

`LaunchVehicleProgram` describe que etapas y motores se usan para cada parte de la mision.

Ejemplo para una nave de dos etapas:

```python
from launch_mission_profiles import LaunchVehicleProgram

program = LaunchVehicleProgram(
    ascent_stage_names=["Stage1"],
    ascent_engine_names=["Stage1Engine"],
    parking_stage_name="Stage2",
    parking_engine_name="Stage2Engine",
    service_stage_name=None,
    service_engine_name=None,
    parking_orbit_target_altitude_km=185.0,
    parking_coast_duration_seconds=20.0 * 60.0,
    translunar_transfer_time_seconds=72.0 * 3600.0,
    target_lunar_distance_km=384_400.0,
)
```

Significado:

- `ascent_stage_names`: etapas usadas durante ascenso inicial.
- `ascent_engine_names`: motores de esas etapas.
- `parking_stage_name`: etapa usada para completar/iniciar orbita de aparcamiento.
- `parking_engine_name`: motor de esa etapa.
- `service_stage_name`: etapa de servicio para maniobras posteriores.
- `service_engine_name`: motor de servicio.
- `parking_orbit_target_altitude_km`: altitud objetivo de orbita baja.
- `parking_coast_duration_seconds`: cuanto esperar antes de TLI.
- `translunar_transfer_time_seconds`: tiempo aproximado de transferencia lunar.
- `target_lunar_distance_km`: distancia objetivo para TLI.

## Usar el motor de lanzamiento

Para misiones que salen desde la Tierra, usa:

```python
from atmosphere_models import EarthExponentialAtmosphere
from mission_engine import step_launch_mission_simulation
from nbody_engine import SimulationClock

clock = SimulationClock(current_time_seconds=0.0, time_scale=1.0)
atmosphere = EarthExponentialAtmosphere()

while clock.current_time_seconds < 3600.0:
    diagnostics = step_launch_mission_simulation(
        root=root,
        clock=clock,
        controller=controller,
        atmosphere=atmosphere,
        real_delta_seconds=1.0,
    )

    print(
        f"t={clock.current_time_seconds:.1f}s "
        f"phase={diagnostics.phase_name} "
        f"alt={diagnostics.altitude_km:.1f}km "
        f"v={diagnostics.speed_km_s:.3f}km/s "
        f"prop={diagnostics.propellant_mass_kg:.1f}kg "
        f"mass={diagnostics.total_mass_kg:.1f}kg"
    )
```

`step_launch_mission_simulation` devuelve:

- `simulated_delta_seconds`
- `phase_name`
- `altitude_km`
- `speed_km_s`
- `propellant_mass_kg`
- `total_mass_kg`

## Plantilla completa

Puedes crear un archivo nuevo, por ejemplo:

```text
v3_spaceship_missions/my_custom_mission.py
```

Con este contenido base:

```python
from atmosphere_models import EarthExponentialAtmosphere
from launch_mission_profiles import LaunchVehicleProgram, TransLunarMissionController
from mission_engine import step_launch_mission_simulation
from nbody_engine import SimulationClock
from solar_system_factory_nbody import create_default_solar_system_nbody
from spacecraft_models import Engine, PropellantType, Spacecraft, Stage, Tank


def create_my_vehicle(parent):
    stage1 = Stage(
        name="Stage1",
        dry_mass_kg=80_000.0,
        tanks=[
            Tank("Stage1Tank", PropellantType.RP1, 600_000.0, 600_000.0),
        ],
        engines=[
            Engine("Stage1Engine", 9_000_000.0, 300.0, ["Stage1Tank"]),
        ],
    )

    stage2 = Stage(
        name="Stage2",
        dry_mass_kg=15_000.0,
        tanks=[
            Tank("Stage2Tank", PropellantType.LH2, 120_000.0, 120_000.0),
        ],
        engines=[
            Engine("Stage2Engine", 1_200_000.0, 440.0, ["Stage2Tank"]),
        ],
    )

    return Spacecraft(
        name="MyVehicle",
        stages=[stage1, stage2],
        dry_radius_km=5.0,
        color_hex="#FFAA55",
        parent=parent,
        metadata={
            "drag_coefficient": 0.45,
            "reference_area_m2": 90.0,
        },
    )


def create_my_mission():
    scene = create_default_solar_system_nbody()
    root = scene.root
    body_index = scene.body_index

    spacecraft = create_my_vehicle(parent=root)

    program = LaunchVehicleProgram(
        ascent_stage_names=["Stage1"],
        ascent_engine_names=["Stage1Engine"],
        parking_stage_name="Stage2",
        parking_engine_name="Stage2Engine",
        service_stage_name=None,
        service_engine_name=None,
        parking_orbit_target_altitude_km=185.0,
        parking_coast_duration_seconds=20.0 * 60.0,
        translunar_transfer_time_seconds=72.0 * 3600.0,
        target_lunar_distance_km=384_400.0,
    )

    controller = TransLunarMissionController(
        spacecraft_name=spacecraft.name,
        program=program,
    )

    controller.place_on_launch_pad(
        spacecraft,
        body_index["Earth"],
        body_index["Moon"],
    )

    clock = SimulationClock(current_time_seconds=0.0, time_scale=1.0)
    atmosphere = EarthExponentialAtmosphere()

    return root, clock, spacecraft, controller, atmosphere


def run():
    root, clock, spacecraft, controller, atmosphere = create_my_mission()

    while clock.current_time_seconds < 3600.0:
        diag = step_launch_mission_simulation(
            root=root,
            clock=clock,
            controller=controller,
            atmosphere=atmosphere,
            real_delta_seconds=1.0,
        )

        print(
            f"t={clock.current_time_seconds:.1f}s "
            f"phase={diag.phase_name} "
            f"alt={diag.altitude_km:.1f}km "
            f"v={diag.speed_km_s:.3f}km/s "
            f"prop={diag.propellant_mass_kg:.1f}kg "
            f"mass={diag.total_mass_kg:.1f}kg"
        )


if __name__ == "__main__":
    run()
```

Ejecutar:

```bash
python v3_spaceship_missions/my_custom_mission.py
```

## Generar una mision desde JSON

Tambien existe una funcion para crear misiones desde un JSON:

```python
from json_mission_generator import generate_mission
```

La funcion acepta:

- un `dict`
- un string JSON
- una ruta a un archivo `.json`

Devuelve un `LaunchMissionScenario`, compatible con:

```python
step_launch_mission_simulation(...)
```

### Ejemplo minimo

```python
from json_mission_generator import generate_mission
from mission_engine import step_launch_mission_simulation


mission_config = {
    "mission_name": "MiPrimeraMisionJson",
    "vehicle": {
        "name": "JsonRocket",
        "dry_radius_km": 5.0,
        "color_hex": "#FFAA55",
        "metadata": {
            "drag_coefficient": 0.45,
            "reference_area_m2": 80.0
        },
        "stages": [
            {
                "name": "Stage1",
                "dry_mass_kg": 50000.0,
                "tanks": [
                    {
                        "name": "MainTank",
                        "propellant_type": "lh2",
                        "capacity_kg": 400000.0,
                        "current_mass_kg": 400000.0
                    }
                ],
                "engines": [
                    {
                        "name": "MainEngine",
                        "max_thrust_newtons": 7000000.0,
                        "specific_impulse_seconds": 360.0,
                        "tank_names": ["MainTank"]
                    }
                ]
            }
        ]
    },
    "start": "DEFAULT",
    "steps": [
        {
            "at": 0.0,
            "type": "burn",
            "name": "liftoff",
            "engine": "MainEngine",
            "force_newtons": 3500000.0,
            "duration": 20.0,
            "direction": "radial_out"
        },
        {
            "at": 20.0,
            "type": "burn",
            "name": "pitch_over",
            "engine": "MainEngine",
            "throttle": 1.0,
            "duration": 80.0,
            "direction": "tangential"
        }
    ]
}

scenario = generate_mission(mission_config)

while scenario.clock.current_time_seconds < 120.0:
    diag = step_launch_mission_simulation(
        scenario.root,
        scenario.clock,
        scenario.controller,
        scenario.atmosphere,
        real_delta_seconds=1.0,
    )

    print(
        f"t={scenario.clock.current_time_seconds:.1f}s "
        f"phase={diag.phase_name} "
        f"alt={diag.altitude_km:.1f}km "
        f"v={diag.speed_km_s:.3f}km/s "
        f"prop={diag.propellant_mass_kg:.1f}kg "
        f"mass={diag.total_mass_kg:.1f}kg"
    )
```

### Ejemplo como archivo JSON

Puedes guardar esto como:

```text
v3_spaceship_missions/missions/mi_mision.json
```

```json
{
  "mission_name": "MiMisionDesdeArchivo",
  "vehicle": "artemis",
  "start": "DEFAULT",
  "steps": [
    {
      "at": 0,
      "type": "burn",
      "name": "liftoff",
      "engine": "SLS_LIFTOFF_CLUSTER",
      "force_newtons": 39000000,
      "duration": 120,
      "direction": "radial_out"
    },
    {
      "at": 120,
      "type": "set_guidance",
      "name": "turn_downrange",
      "direction": "tangential"
    },
    {
      "at": 180,
      "type": "shutdown",
      "name": "main_engine_cutoff",
      "engine": "SLS_LIFTOFF_CLUSTER"
    }
  ]
}
```

Y cargarlo asi:

```python
from json_mission_generator import generate_mission

scenario = generate_mission("v3_spaceship_missions/missions/mi_mision.json")
```

### Esquema del JSON

Campos principales:

```json
{
  "mission_name": "NombreDeLaMision",
  "time_scale": 1.0,
  "start_time_seconds": 0.0,
  "vehicle": "artemis",
  "start": "DEFAULT",
  "steps": []
}
```

`vehicle` puede ser:

- `"artemis"`
- `"apollo"`
- un objeto con definicion completa de nave

Ejemplo de vehiculo custom:

```json
{
  "name": "JsonRocket",
  "dry_radius_km": 5.0,
  "color_hex": "#FFAA55",
  "metadata": {
    "drag_coefficient": 0.45,
    "reference_area_m2": 80.0
  },
  "stages": [
    {
      "name": "Stage1",
      "dry_mass_kg": 50000,
      "tanks": [
        {
          "name": "MainTank",
          "propellant_type": "lh2",
          "capacity_kg": 400000,
          "current_mass_kg": 400000
        }
      ],
      "engines": [
        {
          "name": "MainEngine",
          "max_thrust_newtons": 7000000,
          "specific_impulse_seconds": 360,
          "tank_names": ["MainTank"]
        }
      ]
    }
  ]
}
```

### Punto de salida

`start` puede ser `"DEFAULT"`:

```json
"start": "DEFAULT"
```

Esto coloca la nave en una plataforma de lanzamiento calculada a partir de Tierra y Luna:

- posicion sobre la superficie terrestre
- velocidad heliocentrica de la Tierra
- velocidad tangencial aproximada por rotacion terrestre
- orientacion inicial radial hacia arriba

Tambien puedes indicar coordenadas absolutas:

```json
{
  "type": "coordinates",
  "frame": "absolute",
  "position_km": [1000, 2000, 3000],
  "velocity_km_s": [0, 0, 0],
  "direction": [1, 0, 0]
}
```

O coordenadas relativas a la Tierra:

```json
{
  "type": "coordinates",
  "frame": "earth_relative",
  "position_km": [6371.1, 0, 0],
  "velocity_km_s": [0, 0.465, 0],
  "direction": "radial_out"
}
```

Tambien puedes pedir superficie terrestre con tu propio radial:

```json
{
  "type": "earth_surface",
  "radial": [1, 0, 0],
  "tangential": [0, 1, 0],
  "altitude_km": 0.1,
  "earth_rotation_speed_km_s": 0.465
}
```

### Tipos de pasos

#### `burn`

Enciende un motor durante una duracion.

```json
{
  "at": 0,
  "type": "burn",
  "name": "liftoff",
  "engine": "MainEngine",
  "force_newtons": 3500000,
  "duration": 20,
  "direction": "radial_out"
}
```

Tambien puedes usar `throttle`:

```json
{
  "at": 30,
  "type": "burn",
  "name": "full_power",
  "engine": "MainEngine",
  "throttle": 1.0,
  "duration": 60,
  "direction": "tangential"
}
```

Si usas `force_newtons`, la funcion calcula:

```text
throttle = force_newtons / engine.max_thrust_newtons
```

Si la fuerza pedida supera lo que puede dar el motor, se lanza un error.

#### `set_guidance`

Cambia hacia donde apunta la nave.

```json
{
  "at": 60,
  "type": "set_guidance",
  "name": "point_tangential",
  "direction": "tangential"
}
```

#### `shutdown`

Apaga un motor o todos los motores.

```json
{
  "at": 150,
  "type": "shutdown",
  "name": "cutoff",
  "engine": "MainEngine"
}
```

Sin `engine`, apaga todos:

```json
{
  "at": 150,
  "type": "shutdown",
  "name": "all_cutoff"
}
```

#### `jettison_stage`

Separa una etapa.

```json
{
  "at": 155,
  "type": "jettison_stage",
  "name": "stage_sep",
  "stage": "Stage1"
}
```

### Direcciones soportadas

Las direcciones pueden ser strings:

- `"radial_out"`: hacia fuera desde el centro de la Tierra
- `"radial_in"`: hacia el centro de la Tierra
- `"prograde"`: direccion de la velocidad relativa a la Tierra
- `"retrograde"`: direccion contraria a la velocidad relativa a la Tierra
- `"tangential"`: horizontal aproximada para lanzamiento
- `"moon"`: hacia la Luna

Tambien puedes pasar un vector:

```json
"direction": [1, 0, 0]
```

O un objeto:

```json
{
  "vector": [1, 0, 0],
  "frame": "absolute"
}
```

Para lanzamientos desde Tierra, normalmente es mejor no usar vectores absolutos. Es mas robusto mezclar direcciones relativas al estado actual:

```json
{
  "radial_out": 0.75,
  "tangential": 0.25
}
```

Esto significa:

```text
75% hacia arriba desde la Tierra
25% horizontal/downrange
```

Ejemplo de gravity turn progresivo:

```json
{
  "at": 45,
  "type": "burn",
  "name": "early_gravity_turn",
  "engine": "SLS_LIFTOFF_CLUSTER",
  "throttle": 1.0,
  "duration": 75,
  "direction": {
    "radial_out": 0.75,
    "tangential": 0.25
  }
}
```

Evita usar algo como:

```json
"direction": {
  "vector": [0.65, 0.35, 0.0],
  "frame": "absolute"
}
```

para un lanzamiento salvo que sepas exactamente como estan orientados los ejes globales. Con `DEFAULT`, la plataforma se calcula a partir de Tierra-Luna, por lo que un vector absoluto puede apuntar parcialmente hacia dentro de la Tierra.

### Que hace internamente `generate_mission`

`generate_mission`:

1. Crea el sistema solar con `create_default_solar_system_nbody`.
2. Crea la nave desde `"artemis"`, `"apollo"` o un vehiculo custom.
3. Coloca la nave usando `start`.
4. Convierte pasos JSON en eventos internos.
5. Crea un `JsonMissionController`.
6. Crea `SimulationClock`.
7. Crea `EarthExponentialAtmosphere`.
8. Devuelve un `LaunchMissionScenario`.

La fisica no se calcula en el JSON. La fisica sigue saliendo de:

- `mission_engine.py`
- `spacecraft_models.py`
- `nbody_engine.py`
- `atmosphere_models.py`

## Como crear tu propio controlador

Si quieres control total, crea una clase nueva inspirada en `TransLunarMissionController`.

Estructura minima:

```python
from dataclasses import dataclass
from enum import Enum

from launch_mission_profiles import compute_earth_relative_snapshot
from spacecraft_models import GuidanceMode, Spacecraft
from space_simulation_models import CelestialBody, Vector3


class MyMissionPhase(str, Enum):
    PRELAUNCH = "prelaunch"
    ASCENT = "ascent"
    COAST = "coast"
    COMPLETE = "complete"


@dataclass
class MyMissionController:
    spacecraft_name: str
    phase: MyMissionPhase = MyMissionPhase.PRELAUNCH

    def place_on_launch_pad(self, spacecraft: Spacecraft, earth: CelestialBody) -> None:
        radial = Vector3(1.0, 0.0, 0.0)
        tangential = Vector3(0.0, 1.0, 0.0)

        spacecraft.set_local_position(
            earth.global_position + radial * (earth.radius_km + 0.1)
        )
        spacecraft.local_velocity_km_s = earth.local_velocity_km_s + tangential * 0.465
        spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, radial)
        spacecraft.update_guidance()

    def update(
        self,
        spacecraft: Spacecraft,
        earth: CelestialBody,
        moon: CelestialBody,
        current_time_seconds: float,
    ) -> None:
        snapshot = compute_earth_relative_snapshot(spacecraft, earth)

        radial = (spacecraft.global_position - earth.global_position).normalized()
        tangential = Vector3(-radial.y, radial.x, radial.z).normalized()

        if self.phase == MyMissionPhase.PRELAUNCH:
            spacecraft.arm_engine("Stage1Engine", throttle=1.0)
            spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, radial)
            self.phase = MyMissionPhase.ASCENT
            return

        if self.phase == MyMissionPhase.ASCENT:
            pitch_fraction = max(0.0, min(1.0, snapshot.altitude_km / 180.0))
            target = radial * (1.0 - pitch_fraction) + tangential * pitch_fraction
            spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, target.normalized())

            if snapshot.altitude_km > 180.0 and snapshot.speed_km_s > snapshot.circular_speed_km_s * 0.95:
                spacecraft.shutdown_all_engines()
                self.phase = MyMissionPhase.COAST
                return

        if self.phase == MyMissionPhase.COAST:
            spacecraft.set_guidance_mode(GuidanceMode.PROGRADE)
            if current_time_seconds > 2000.0:
                self.phase = MyMissionPhase.COMPLETE
                return
```

Para usarlo con `mission_engine.py`, el metodo debe llamarse `update(...)` y recibir:

```python
update(spacecraft, earth, moon, current_time_seconds)
```

El motor lo llamara automaticamente en cada subpaso.

## Reglas practicas para disenar una mision

### Despegue

Empieza con vector radial:

```python
spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, radial)
spacecraft.arm_engine("Stage1Engine", 1.0)
```

### Gravity turn

Haz que el vector pase gradualmente de radial a tangencial:

```python
pitch_fraction = max(0.0, min(1.0, (altitude_km - 1.0) / 180.0))
target = radial * (1.0 - pitch_fraction) + tangential * pitch_fraction
```

### Separacion de etapa

Cuando la etapa esta casi vacia:

```python
if spacecraft.get_stage("Stage1").propellant_mass_kg <= 1.0:
    spacecraft.shutdown_engine("Stage1Engine")
    spacecraft.jettison_stage("Stage1")
    spacecraft.arm_engine("Stage2Engine", 1.0)
```

### Orbita de aparcamiento

Condicion aproximada:

```python
reached_parking = (
    snapshot.altitude_km >= 180.0
    and snapshot.speed_km_s >= snapshot.circular_speed_km_s * 0.97
    and abs(snapshot.radial_speed_km_s) <= 0.35
)
```

### TLI

Para una inyeccion translunar simple:

```python
target = moon.global_position - spacecraft.global_position
spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, target.normalized())
spacecraft.arm_engine("TliEngine", 1.0)
```

Apagado aproximado:

```python
target_apoapsis_altitude = 384_400.0 - earth.radius_km

if snapshot.apoapsis_altitude_km and snapshot.apoapsis_altitude_km > target_apoapsis_altitude * 0.92:
    spacecraft.shutdown_all_engines()
```

## Como comprobar que la mision va bien

Durante la simulacion, imprime:

```python
print(spacecraft.total_mass_kg)
print(spacecraft.propellant_mass_kg)
print(spacecraft.active_engines())
print(spacecraft.available_delta_v_km_s)
```

Y el estado orbital:

```python
snapshot = compute_earth_relative_snapshot(spacecraft, earth)

print(snapshot.altitude_km)
print(snapshot.speed_km_s)
print(snapshot.circular_speed_km_s)
print(snapshot.apoapsis_altitude_km)
print(snapshot.periapsis_altitude_km)
```

Los sintomas tipicos:

- Si la nave cae: poco empuje, demasiado horizontal muy pronto, demasiado drag o poca velocidad.
- Si se escapa demasiado rapido: demasiado empuje o apagado tarde.
- Si no llega a orbita: falta delta-v, mal gravity turn o apagado temprano.
- Si consume combustible muy rapido: empuje alto, Isp bajo o tanques pequenos.
- Si la masa no cambia: los motores no estan activos o no estan conectados al tanque correcto.
- Si no hay aceleracion: motor apagado, throttle cero, sin combustible o vector de guiado no actualizado.

## Recomendacion de diseno

Para empezar, crea misiones en este orden:

1. Nave de una etapa.
2. Despegue vertical corto.
3. Gravity turn simple.
4. Apagado por tiempo.
5. Apagado por altitud y velocidad.
6. Segunda etapa.
7. Separacion de etapa.
8. Orbita de aparcamiento.
9. Coast.
10. TLI.

Primero usa reglas simples. Cuando funcionen, cambia los tiempos fijos por condiciones fisicas.

El objetivo no es escribir una lista perfecta de tiempos, sino una maquina de estados que reaccione al estado real de la nave.
