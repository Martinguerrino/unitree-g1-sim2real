# Unitree G1 sim-to-real

Orquestador para mantener una aplicación común frente a MuJoCo, Isaac Sim y,
en el futuro, un G1 físico. Los proyectos originales siguen independientes en
`external/mujoco` y `external/isaac`, como submódulos fijados a un commit.

**Estado: MVP de telemetría DDS de escenas pausadas.** Carga el robot de cada
repositorio, publica posiciones/velocidades de 29 joints en `rt/lowstate` usando
los tipos oficiales Unitree HG y las consume con la misma `RobotInterface`.
No envía comandos, no ejecuta pick/place y no constituye un benchmark de física.
Isaac necesita validación en su runtime: una prueba del core no demuestra que
el bridge funcione en Isaac. Ver [análisis y decisiones](docs/architecture.md).

Validación local: **7 tests pasaron y MuJoCo recibió 20 estados por DDS** con
Python 3.12, MuJoCo 3.13.0 y SDK2 instalado editable. Isaac permanece sin validar.

```text
run.py → aplicación común → RobotInterface ← SDK2 / DDS ← bridge ← simulador
                                             ↑
                                  contrato HG LowState
```

La arquitectura futura agrega comandos en sentido contrario. Los accesos a
MuJoCo/Isaac están encapsulados en el proceso bridge; el core no importa esos
runtimes. ROS 2 continúa disponible en el proyecto MuJoCo original.

Instalación del core y MuJoCo:

```bash
git clone --recurse-submodules <URL_DE_ESTE_REPOSITORIO>
cd unitree-g1-sim2real
# Para un clon existente:
git submodule update --init --recursive
/usr/bin/python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[mujoco]'
```

Instalar [Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python)
en ese entorno. Revisión inspeccionada: `65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5`:

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git .venv/unitree_sdk2
git -C .venv/unitree_sdk2 checkout 65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5
python -m pip install -e .venv/unitree_sdk2
```

SDK2 depende del binding Python CycloneDDS 0.10.2. Si pip no encuentra la biblioteca
nativa, esta instalación local evita depender de `/usr/local`:

```bash
git clone https://github.com/eclipse-cyclonedds/cyclonedds.git /tmp/g1-cyclonedds
git -C /tmp/g1-cyclonedds checkout 5041f3560c088c99e5088b2b8520b69169621196
cmake -S /tmp/g1-cyclonedds -B /tmp/g1-cyclonedds-build \
  -DCMAKE_INSTALL_PREFIX="$PWD/.venv/cyclonedds" \
  -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF
cmake --build /tmp/g1-cyclonedds-build --parallel 6
cmake --install /tmp/g1-cyclonedds-build
export CYCLONEDDS_HOME="$PWD/.venv/cyclonedds"
python -m pip install --no-cache-dir -e .venv/unitree_sdk2
```

Requiere compilador C y CMake. Esa revisión nativa es 0.10.5, de la rama 0.10.x
recomendada por SDK2; el binding Python sigue siendo 0.10.2. Consultar también
el README oficial de SDK2 si cambia el entorno de instalación.
Usar Python 3.12 para el core en Ubuntu 24.04 y el Python propio de Isaac para
su bridge. Python 3.13 produjo un error nativo `_Py_IsFinalizing` con este binding
durante la validación y queda excluido de `pyproject.toml`.
Conservar el checkout SDK2: la instalación editable usa sus bibliotecas CRC.
El wheel generado desde la revisión inspeccionada omitió esos `.so`.
No es necesario instalar ROS 2, MoveIt, cuRobo ni Qwen para este MVP.

Ejecutar MuJoCo:

```bash
python scripts/run.py --sim mujoco --check
python scripts/run.py --sim mujoco
```

Ejecutar Isaac Sim 5.1:

1. Tener instalado Isaac y accesibles los payloads remotos del USD original.
2. Instalar el core y SDK2 también en el Python de Isaac. Mantener separado
   ese entorno del virtualenv del core; comprobar la compatibilidad de sus
   dependencias antes de reutilizar entornos con cuRobo/Warp.
3. Configurar la ruta de `python.sh`, que no se supone instalada en un home fijo.

```bash
/ruta/isaac-sim/python.sh -m pip install -e .
/ruta/isaac-sim/python.sh -m pip install -e .venv/unitree_sdk2
python scripts/run.py --sim isaac --backend-python /ruta/isaac-sim/python.sh
```

Para conservar la ruta, crear `configs/local.yaml` (ignorado por Git):

```yaml
backend:
  python: /ruta/isaac-sim/python.sh
```

```bash
python scripts/run.py --sim isaac --config configs/local.yaml
```

También se puede completar `backend.python` en `configs/isaac.yaml` y usar
exactamente `python scripts/run.py --sim isaac`. La configuración se combina en
orden: common → simulador → override. Las rutas de assets son relativas al repo
orquestador, no al directorio desde el que se invoca Python.

`--check` sólo comprueba configuración y archivos; indica explícitamente
`runtime_validated: false`. No reemplaza una ejecución DDS.
`--sim real` falla explícitamente; no existe backend físico en esta versión.

Cada corrida produce `experiments/runs/<id>/` con configuración efectiva,
`backend.log`, `states.jsonl` y `result.json`. Los resultados distinguen éxito
de telemetría de éxito físico. Un fallo de preflight se imprime en JSON y sale
con código 1; no crea una corrida. Ctrl-C termina el proceso bridge.

```bash
python scripts/evaluate.py
python -m unittest discover -s tests -v
```

La prueba requiere 20 estados nuevos. Un timeout o un proceso que termina
produce error, nunca estados sintéticos de reemplazo. Sólo q/dq son válidos;
otros campos del mensaje HG permanecen con sus defaults y no se exponen como
mediciones. `tick` es un contador de muestras del bridge, no tiempo simulado.

Usar un simulador por vez: ambos comparten dominio 42, interfaz `lo` y topic
`rt/lowstate`. No ejecutar otro publisher con ese contrato en el mismo dominio.
Para pruebas simultáneas habría que aislar dominios y consumidores por corrida.

En la RTX 2060/16 GB, empezar headless, sin cámaras ni políticas cargadas.
El USD de Isaac todavía incluye mobiliario y assets remotos; este scaffold no
garantiza que entre en 6 GB de VRAM. La simplificación de escena requiere una
medición en el equipo. El MVP de MuJoCo no abre un viewer.

Roadmap, en orden:

1. Validar telemetría real de ambos bridges y fijar versiones de runtimes/assets.
2. Agregar `LowCmd` bidireccional con mapping por nombre, PD/esfuerzo, límites,
   CRC, modos, watchdog y un único dueño del control. Ensayo articular con soporte.
3. Igualar modelo, manos, entorno, reloj, reset y condiciones iniciales.
4. Integrar una política de locomoción y manipulación intercambiables.
5. Reutilizar planner/validación/A* de Isaac mediante adapters, y conectar
   `GoTo(Table_A) → Pick(Box) → GoTo(Table_B) → Place(Box)`.
6. Agarre físico bimanual, RGB-D en cabeza, métricas físicas y perturbaciones.
7. Backend real; después percepción avanzada, ACT o VLM según el experimento.

No hay archivos vacíos para futuras capas: sus protocolos están concentrados
en `contracts.py` y se separarán en paquetes cuando tengan implementaciones.
