# AstraTravel Web

Laboratorio de vuelo espacial en español. Funciona en el navegador con módulos nativos, Canvas 2D con proyección 3D y un Web Worker independiente para la física. No descarga bibliotecas, fuentes, texturas ni efemérides externas al arrancar.

## Arranque

Desde la raíz del repositorio, con Node.js 20 o superior:

```bash
node web/server.mjs
```

Abre http://localhost:8097. También puedes ejecutar `npm start` dentro de `web/`; no requiere `npm install`. El puerto se cambia con `PORT`. No abras `index.html` mediante `file://`: los módulos y el worker necesitan HTTP.

Con Docker, desde la raíz:

```bash
docker compose up -d --build
```

Abre http://localhost:8097. Puedes cambiar el puerto con `ASTRATRAVEL_PORT`. La imagen solo contiene los siete archivos públicos necesarios y Nginx; no ejecuta Python ni Node en producción. La misma carpeta web puede servirse con cualquier servidor estático que entregue JavaScript con el MIME correcto. Los recursos usan rutas relativas para permitir alojarla bajo un subdirectorio.

## Escenarios

- **De la Tierra a órbita:** cohete didáctico de dos etapas, atmósfera terrestre en corrotación, guiado de ascenso y corte por periapsis. El caso inicial alcanza una órbita aproximadamente de 167 × 394 km, con inserción alrededor de T+468 s.
- **Rumbo a la Luna:** comienza en órbita de aparcamiento a 200 km; la etapa enciende durante unos 215 s y después vuela libremente bajo gravedad terrestre, lunar y solar. El caso inicial pasa a unos 3.612 km sobre la superficie lunar a los 3,65 días. No promete alunizaje ni retorno.
- **Laboratorio orbital:** comienza en una órbita de 400 km, con control manual de potencia, orientación y referencia Tierra/Luna. Prueba empuje prógrado, retrógrado o normal para cambiar la órbita.
- **Sistema solar:** Sol y ocho planetas integrados con atracción mutua, a partir de estados circulares coplanares idealizados.

Seleccionar un escenario o reiniciar descarta el vuelo actual. La telemetría puede exportarse antes como CSV. Al ocultar la pestaña se pausa el vuelo; no se recupera un salto de tiempo al regresar. Todo el estado de misión es local a la pestaña.

## Controles

- Iniciar/pausar, avance de 60 s en pausa y reinicio.
- Escala de tiempo entre ×1 y ×10.000.000. Es un máximo solicitado: si el dispositivo no alcanza esa velocidad, el tiempo simulado avanza más despacio sin ampliar los pasos físicos.
- Desactiva el guiado automático para orientar la nave y regular el empuje. **Cortar motor** también desactiva el guiado, evitando que vuelva a encenderse solo.
- Separación manual de etapas en modo manual: descarta la etapa activa, incluyendo el propelente restante.
- Cámara centrada en Tierra, Luna, nave, Tierra–Luna o Sol; arrastre para rotar, rueda o pellizco para zoom y botones equivalentes. Botón de vista cenital.
- Espacio inicia/pausa cuando el foco no está en otro control; +/− ajustan el zoom.
- La línea continua muestra posiciones recorridas relativas al cuerpo de enfoque; la discontinua representa la órbita osculante de dos cuerpos, no una predicción N-body completa. Los marcadores pequeños se amplían visualmente; las coordenadas conservan escala lineal.

## Física y límites

`physics.js` usa kilómetros, segundos y kilogramos; recibe empuje en newtons y convierte su aceleración a km/s². Integra con Runge–Kutta de orden 4, con pasos máximos de 0,5 s durante encendidos y cerca de la atmósfera. En vuelo libre limita el paso según la escala dinámica local; en modo solar el máximo es 1.800 s. El worker acumula fracciones de tiempo real para mantener los pasos independientes de la frecuencia del dibujo.

- Gravedad: `a = Σ μᵢ (rᵢ − r) / |rᵢ − r|³`.
- Consumo: `ṁ = F / (Isp · g₀)`, con masa evaluada durante cada subetapa RK4. El paso se corta exactamente al agotar el tanque.
- Presupuesto ideal de velocidad: ecuación de Tsiolkovski por etapas; no incluye pérdidas gravitatorias ni atmosféricas.
- Drag: `½ ρ v_rel² Cd A`, con `ρ = 1,225 exp(−h/8.500 m)`, `Cd = 0,4`, `A = 20 m²` y atmósfera en corrotación. Por encima de 180 km se omite. No hay sustentación ni calentamiento.
- La nave es una partícula de prueba: recibe gravedad sin alterar la órbita de los cuerpos celestes. Estos sí se atraen mutuamente.
- Detección de impactos con un segmento barrido entre estados; detiene la simulación. No modela aterrizajes ni relieve. El instante de impacto tiene la resolución del subpaso.
- Orientación instantánea, Tierra esférica, sin J₂, relatividad, limitaciones estructurales ni dinámica rotacional.
- Los estados iniciales NO son efemérides de una fecha real. La fase lunar se ha elegido para proporcionar un escenario didáctico reproducible. Los vehículos no reproducen hardware certificado de Apollo/Artemis.
- La deriva de energía mostrada pertenece exclusivamente a los cuerpos celestes; no es un error estimado de la trayectoria de la nave. La energía de la nave cambia legítimamente por empuje y rozamiento.
- Historial y trazas tienen tamaño acotado mediante reducción de muestras. El CSV exporta las muestras retenidas, no cada paso del integrador. La referencia de velocidad queda identificada por fila. Límite de sesión: 20 años simulados.

Fuentes: [JPL, constantes astrodinámicas](https://ssd.jpl.nasa.gov/astro_par.html), [JPL, parámetros planetarios](https://ssd.jpl.nasa.gov/planets/phys_par.html), [NASA, impulso específico](https://www.grc.nasa.gov/www/k-12/airplane/specimp.html), [NASA, ecuación ideal del cohete](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/ideal-rocket-equation/).

## Verificación

Desde `web/`:

```bash
npm run check
npm test
```

También se pueden ejecutar directamente `node --test web/tests/*.test.mjs` desde la raíz. Las pruebas cubren órbita analítica, conservación durante coast, consumo e impulso, agotamiento y separación de etapas, inserción orbital, sobrevuelo lunar, convergencia al reducir el paso, impactos, orientación relativa, cambios de inclinación, conservación solar, ciclo de vida del worker y entrega HTTP de los módulos. CI ejecuta además la construcción y una prueba HTTP del contenedor.

## Estructura

- `physics.js`: dominio físico independiente del navegador.
- `worker.js`: reloj, trabajo en lotes, comandos y telemetría.
- `renderer.js`: proyección ortográfica 3D, cámara, órbitas y gráficos.
- `app.js`: controles y presentación de resultados.
- `index.html`, `styles.css`: interfaz responsive y ayuda física.
- `server.mjs`: servidor local sin dependencias.
- `tests/`: pruebas físicas e integración sin dependencias.

El código Python y sus misiones JSON originales siguen en la raíz como referencia. Esta versión implementa una nueva simulación web inspirada en ellos; no importa ni ejecuta directamente los planes JSON del motor Python.
