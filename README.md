# v3_spaceship_missions

Simulador espacial en Python centrado en dos ideas:

1. Simular un sistema solar N-body basico, donde Sol, planetas y Luna se atraen gravitatoriamente.
2. Simular una mision de lanzamiento Tierra-Luna con cohete por etapas, combustible, empuje, drag atmosferico, orbita de aparcamiento, TLI y sobrevuelo lunar.

No es un simulador orbital de precision tipo NASA/SPICE. Es un modelo fisico simplificado, pero estructurado para experimentar con dinamica orbital, vehiculos por etapas y misiones translunares.

## Arquitectura general

La carpeta esta organizada por capas:

1. Modelos base: vectores, cuaterniones, transforms, nodos jerarquicos y cuerpos celestes.
2. Sistema solar: crea Sol, planetas, Tierra y Luna con masas, radios, colores y estados iniciales.
3. Motor N-body: calcula gravedad entre todos los cuerpos y avanza la simulacion con Velocity Verlet.
4. Modelos de nave: etapas, tanques, motores, combustible, masa variable, orientacion y empuje.
5. Control de mision: decide encendidos, gravity turn, apagados, separacion de etapas, TLI y coast.
6. Escenarios: monta misiones completas Apollo-like o Artemis-like.
7. Visualizadores: usan Matplotlib 3D para ver el sistema solar o la mision de lanzamiento.

Flujo principal de la mision translunar:

```text
mission_viewer_3d.py
    -> mission_scenarios.py
        -> solar_system_factory_nbody.py
        -> spacecraft_models.py
        -> launch_mission_profiles.py
    -> mission_engine.py
        -> nbody_engine.py
        -> atmosphere_models.py
        -> spacecraft_models.py
```

## Unidades

El proyecto usa estas unidades por defecto:

- Distancia: kilometros (km)
- Velocidad: kilometros por segundo (km/s)
- Masa: kilogramos (kg)
- Tiempo: segundos (s)
- Empuje: newtons (N)
- Aceleracion interna: kilometros por segundo al cuadrado (km/s^2)
- Angulos orbitales: grados
- Rotaciones con quaternion/ejes: radianes

## Como ejecutar

Desde la raiz del repo:

```bash
python v3_spaceship_missions/mission_viewer_3d.py --vehicle artemis
```

o:

```bash
python v3_spaceship_missions/mission_viewer_3d.py --vehicle apollo
```

Para ver solo la simulacion N-body Sol-Tierra-Luna:

```bash
python v3_spaceship_missions/simulation_viewer_3d_nbody.py
```

Dependencias principales:

- Python 3
- Matplotlib

## Controles del visor de mision

`mission_viewer_3d.py` tiene tres modos:

- `1`: vista local de la Tierra
- `2`: vista Tierra-Luna
- `3`: vista local de la nave

Controles:

- `Space`: pausa
- `1`, `2`, `3`: cambiar vista
- `Up` / `Down`: cambiar escala de tiempo
- `+` / `-`: zoom
- `Left` / `Right`: azimut
- `W` / `S`: elevacion
- `A` / `D`: pan X
- `Q` / `E`: pan Y
- `Z` / `C`: pan Z
- `L`: etiquetas
- `T`: trails
- `H`: ayuda
- `R`: reset
- Rueda del raton: zoom
- Drag izquierdo: rotar
- Drag derecho o medio: mover/pan

El overlay muestra:

- vehiculo
- fase actual
- tiempo de mision
- altitud
- velocidad
- propelente restante
- masa total
- delta-v disponible
- distancia a la Luna
- motores activos
- etapas activas
- eventos recientes

## Archivos

### `space_simulation_models.py`

Base matematica y de escena.

Define:

- `Vector3`: vector 3D con suma, resta, producto escalar, producto vectorial, magnitud y normalizacion.
- `Quaternion`: rotaciones 3D.
- `Transform`: posicion y rotacion relativa a un padre.
- `Node`: nodo jerarquico con padre, hijos, transform local y transform global.
- `ReferenceFrame`: nodo usado como sistema de referencia.
- `CelestialBody`: cuerpo fisico con masa, radio, velocidad, rotacion, datos orbitales y propiedades visuales.
- `PhysicalProperties`, `RotationProperties`, `OrbitParameters`, `VisualProperties`: datos fisicos, rotacionales, orbitales y visuales.

La idea central es que todo cuelga de un arbol de nodos. Cada cuerpo tiene una posicion local, pero en v3 casi todos los cuerpos son hijos directos del root, asi que esa posicion local funciona practicamente como posicion global heliocentrica.

### `nbody_engine.py`

Motor gravitatorio N-body.

Define:

- `SimulationClock`: tiempo actual, escala de tiempo y pausa.
- `compute_accelerations`: calcula aceleracion gravitatoria de cada cuerpo por todos los demas.
- `velocity_verlet_step`: integra posicion y velocidad con Velocity Verlet.
- `step_simulation`: avanza el reloj y divide el avance en subpasos.
- `compute_diagnostics`: energia total, centro de masa y velocidad del centro de masa.

Este motor no usa orbitas keplerianas para propagar los planetas despues del estado inicial. Una vez arrancada la simulacion, el movimiento sale de la gravedad Newtoniana cuerpo a cuerpo.

Flujo simplificado:

```text
clock.advance()
para cada subpaso:
    calcular aceleraciones gravitatorias
    mover posiciones
    recalcular aceleraciones
    actualizar velocidades
    actualizar rotaciones axiales
```

### `solar_system_factory_nbody.py`

Construye el sistema solar inicial.

Contiene:

- constantes como `AU_KM` y `SECONDS_PER_DAY`
- `OrbitalElements`: elementos orbitales clasicos
- `BodySeed`: datos base de cada cuerpo
- `BODY_SEEDS`: Sol, Mercurio, Venus, Tierra, Luna, Marte, Jupiter, Saturno, Urano y Neptuno
- `create_default_solar_system_nbody`: crea la escena completa

Los planetas se inicializan con elementos orbitales aproximados. Despues, los mueve el motor N-body.

La Tierra y la Luna se inicializan de forma especial: primero se calcula un estado aproximado del baricentro Tierra-Luna y luego se colocan Tierra y Luna alrededor de ese baricentro. Esto evita que la Luna empiece de forma incoherente respecto a la Tierra.

### `spacecraft_models.py`

Modelo de naves y cohetes.

Define:

- `Tank`: tanque de propelente.
- `Engine`: motor con empuje, Isp, throttle, igniciones y consumo.
- `Stage`: etapa con masa seca, tanques y motores.
- `Spacecraft`: nave completa. Hereda de `CelestialBody` para participar en la gravedad N-body.
- `GuidanceMode`: modos de guiado como prograde, retrograde, radial, target vector y manual.

La masa de una `Spacecraft` no es fija. Se calcula sumando las etapas no separadas, incluyendo el propelente restante.

Funciones importantes:

- `arm_engine`: enciende un motor.
- `shutdown_engine`: apaga un motor.
- `shutdown_all_engines`: apaga todos los motores.
- `jettison_stage`: separa una etapa.
- `compute_thrust_acceleration_km_s2`: convierte empuje en aceleracion.
- `consume_propellant`: consume combustible segun el caudal masico.
- `update_guidance`: orienta la nave hacia el vector de guiado.

La orientacion esta idealizada: la nave puede girar instantaneamente hacia el vector deseado. No hay dinamica rotacional real.

### `atmosphere_models.py`

Modelo atmosferico terrestre simple.

Define:

- `EarthExponentialAtmosphere`
- `AtmosphereSample`

Se usa para calcular densidad, presion y temperatura segun altitud. En la mision de lanzamiento sirve para calcular drag atmosferico, normalmente hasta unos 180 km.

Modelo:

```text
density = rho0 * exp(-altitude / scale_height)
```

Es una primera aproximacion, no aerodinamica de alta fidelidad.

### `launch_mission_profiles.py`

Cerebro de la mision translunar.

Define:

- `MissionPhase`: fases de mision.
- `OrbitalSnapshot`: datos orbitales relativos a la Tierra.
- `LaunchVehicleProgram`: configuracion abstracta del cohete.
- `TransLunarMissionController`: controlador que decide que hacer en cada instante.

Fases principales:

```text
PRELAUNCH
ASCENT_STAGE_1
ASCENT_STAGE_2
ASCENT_STAGE_3
PARKING_COAST
TLI_BURN
TRANSLUNAR_COAST
LUNAR_FLYBY
RETURN_COAST
COMPLETE
```

El controlador:

- coloca el vehiculo en la plataforma
- inicia el primer motor
- hace gravity turn
- detecta agotamiento de etapas
- separa etapas con retraso
- detecta orbita de aparcamiento
- espera una fase de coast
- apunta hacia una posicion futura aproximada de la Luna
- enciende TLI
- apaga TLI cuando el apoapsis se acerca a distancia lunar
- detecta encuentro lunar

`compute_earth_relative_snapshot` calcula altitud, velocidad, velocidad circular, energia especifica, excentricidad, periapsis y apoapsis de la nave respecto a la Tierra.

Nota tecnica: el metodo `_schedule_stage_jettison` contiene una referencia a `self_spacecraft`, que no existe. Actualmente parece codigo muerto porque el flujo real usa `_queue_jettison`, pero conviene limpiarlo o corregirlo.

### `mission_engine.py`

Motor especifico de la mision de lanzamiento v2.

Combina:

- gravedad N-body
- empuje de la nave
- consumo de propelente
- drag atmosferico
- controlador translunar
- integracion Velocity Verlet

Funcion principal:

```python
step_launch_mission_simulation(...)
```

Esa funcion:

1. Avanza el reloj.
2. Divide el tiempo simulado en subpasos pequenos, por defecto de 2 s.
3. En cada subpaso llama al controlador de mision.
4. Calcula gravedad.
5. Anade empuje si la nave esta quemando.
6. Anade drag atmosferico cerca de la Tierra.
7. Integra posiciones y velocidades.
8. Consume propelente.
9. Devuelve diagnosticos: fase, altitud, velocidad, masa de propelente y masa total.

Este es el motor que usa `mission_viewer_3d.py`.

### `mission_scenarios.py`

Monta escenarios completos.

Define:

- `LaunchMissionScenario`
- `create_apollo_like_launch_vehicle`
- `create_artemis_like_launch_vehicle`
- `create_translunar_launch_scenario`

`create_translunar_launch_scenario("artemis")` hace:

```text
crear sistema solar
crear nave Artemis-like
crear controlador translunar
colocar nave en plataforma de lanzamiento
crear reloj
crear atmosfera
devolver todo en LaunchMissionScenario
```

Vehiculos incluidos:

- Apollo-like:
  - `S-IC`
  - `S-II`
  - `S-IVB`
  - `Apollo_CSM_Service`

- Artemis-like:
  - `SLS_Core_Boosters`
  - `ICPS`
  - `Orion_ESM`

Cada etapa tiene masa seca, tanques y motores con empuje e Isp.

### `mission_viewer_3d.py`

Visualizador 3D de la mision de lanzamiento a la Luna.

Usa Matplotlib y anima la simulacion llamando a:

```python
step_launch_mission_simulation(...)
```

Muestra Tierra, Luna, nave, trails, overlays de telemetria y eventos recientes. La clase principal se llama `MissionViewer3DV2`.

### `simulation_viewer_3d_nbody.py`

Visualizador del sistema solar N-body sin mision de cohete.

Tiene dos modos:

- `1`: vista global Sol-Tierra-Luna
- `2`: vista local Tierra-Luna

Llama a:

```python
step_simulation(...)
```

Sirve para comprobar que la Tierra y la Luna se mueven de manera coherente en el entorno gravitatorio.

### `mission_timeline.py`

Sistema generico de eventos programados.

Define acciones como:

- `StartBurnAction`
- `StopBurnAction`
- `SetGuidanceAction`
- `JettisonStageAction`
- `CallbackAction`

Y una `MissionTimeline` que ejecuta eventos cuando el tiempo simulado supera su `trigger_time_seconds`.

Este archivo pertenece a una ruta mas generica de mision. En la mision translunar actual, el flujo principal lo lleva `TransLunarMissionController`, no tanto `MissionTimeline`.

### `mission_simulation_engine.py`

Motor de mision generico anterior o paralelo.

Combina:

- N-body
- timeline de eventos
- guiado de naves
- empuje
- consumo de propelente

Funcion principal:

```python
step_mission_simulation(...)
```

A diferencia de `mission_engine.py`, este motor no tiene la logica especifica de lanzamiento, atmosfera y controlador translunar. Es mas generico: programas eventos en una `MissionTimeline` y el motor los ejecuta.

En la practica:

- `mission_simulation_engine.py`: motor generico con timeline
- `mission_engine.py`: motor especifico para lanzamiento Tierra-Luna

### `ephemeris_provider.py`

Interfaz preparada para futuras efemerides.

Define:

- `EphemerisState`
- `EphemerisProvider`
- `StaticEnvironmentEphemerisProvider`

La idea es poder cambiar el sistema aproximado actual por un proveedor real, por ejemplo SPICE/JPL, sin reescribir toda la simulacion.

Actualmente `StaticEnvironmentEphemerisProvider` simplemente lee el estado actual de los cuerpos ya cargados en memoria.

## Flujo de una mision Artemis/Apollo

Cuando ejecutas `mission_viewer_3d.py`:

1. `main()` parsea argumentos.
2. Crea `MissionViewer3DV2`.
3. El viewer llama a `create_translunar_launch_scenario`.
4. Se crea el sistema solar con `create_default_solar_system_nbody`.
5. Se crea la nave Apollo-like o Artemis-like.
6. Se crea un `TransLunarMissionController`.
7. El controlador coloca la nave en la superficie terrestre.
8. Arranca la animacion Matplotlib.
9. Cada frame llama a `step_launch_mission_simulation`.
10. El motor avanza fisica, propulsion, drag, staging y fase de mision.
11. El viewer redibuja Tierra, Luna, nave, trails y texto informativo.

## Que simula bien

El proyecto ya cubre:

- gravedad N-body basica
- masas variables de naves
- motores con Isp y consumo
- cohetes por etapas
- separacion de etapas
- drag atmosferico simplificado
- guiado prograde/radial/vector objetivo
- visualizacion 3D interactiva
- escenarios Apollo-like y Artemis-like
- transicion de lanzamiento a orbita y TLI

## Simplificaciones

Limitaciones actuales:

- No usa efemerides reales de alta precision.
- Los planetas arrancan desde elementos orbitales aproximados.
- La actitud de la nave cambia instantaneamente.
- No hay aerodinamica real, solo drag exponencial.
- No hay sustentacion, control de pitch/yaw/roll ni limitaciones estructurales.
- El launch guidance es heuristico, no una optimizacion de trayectoria.
- No hay colisiones ni aterrizajes.
- La Luna se predice linealmente para apuntar la TLI.
- Los tamanos visuales estan escalados artificialmente para verse en pantalla.
- La mision translunar es aproximada, no una reproduccion exacta de Apollo o Artemis.

## Estado del proyecto

`v3_spaceship_missions` parece una evolucion hacia un simulador mas serio: separa dominio, fisica, mision, escenarios y visualizacion.

Hay dos caminos de simulacion:

- Camino generico: `mission_timeline.py` + `mission_simulation_engine.py`
- Camino translunar especifico: `launch_mission_profiles.py` + `mission_engine.py` + `mission_viewer_3d.py`

El camino translunar especifico es el mas completo y probablemente el flujo principal de uso actual.

