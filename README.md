# caddy_ai2_ros2_mvsim_simulation

**ROS 2:** Jazzy | **Simulador:** [MVSim](https://github.com/MRPT/mvsim) **≥ 2.0** | **Proyecto:** Caddy AI2 (carrito de golf, plataforma de pruebas de vehículos)

Simulación alternativa del Caddy AI2 con MVSim, ligera y con la **misma interfaz que la simulación de Gazebo**
(`caddy_ai2_ros2_gazebo_simulation`) y que el vehículo real: lo que consume topics, frames y TF (EKF,
EasyNav, RViz...) funciona igual con cualquiera de los tres.

## Qué simula

- **Vehículo:** generado al lanzar desde `caddy_ai2_ros2_description/bringup/config/robot_params.yaml`, la
  misma fuente única que usa Gazebo (distancia entre ejes, vía, caja de colisión, dirección máxima, masas).
- **Sensores** (los de `robot_params.yaml` con `enabled: true`): Helios RS-5515 3D (la definición de 32 canales y
  70° que trae MVSim), SICK LMS291 2D, IMU y GPS SBG IG-500N, IMU VN-100 y las ZED 2i activas. Parámetros de
  simulación en `config/mvsim_sensors.yaml` (copia de los `sensor_params.yaml` de cada paquete de sensor).
- **Mundos** (`world:=`): `parking_garage` (solo la planta baja; el mapa de ocupación `maps/parking_garage_ground.*`
  sale de un PCD de la planta baja, ver más abajo) y `road_circuit` (el `demo_road_circuit1` de MVSim, con rampas,
  baches y un puente; ver más abajo). MVSim es **2.5D**: la altura del terreno bajo cada rueda sale de mapas de
  alturas, planos y bloques, pero un mapa de ocupación no tiene altura, así que el parking de varias plantas
  apiladas no se puede representar bien.

## Uso

```bash
ros2 launch caddy_ai2_ros2_mvsim_simulation sim_mvsim.launch.py
ros2 launch caddy_ai2_ros2_mvsim_simulation sim_mvsim.launch.py headless:=True use_rviz:=False   # servidor, sin pantalla
```

| Argumento | Defecto | Descripción |
|---|---|---|
| `gazebo_compat` | `True` | Interfaz idéntica a Gazebo. Con `False`, MVSim "crudo" (`/caddy_ai2/<sensor>` y sus propios TF) |
| `world` | `parking_garage` | `parking_garage` o `road_circuit` |
| `headless` | `False` | Sin GUI; renderizado EGL en la GPU |
| `realtime_factor` | `1.0` | `<1` más lento, `>1` más rápido, `0` lo más rápido posible |
| `do_fake_localization` | `False` | Publica `map → odom` desde la verdad de terreno (Gazebo no lo hace: lo da el localizador) |
| `x`, `y`, `yaw_deg` | `0`, `0`, `0` | Pose inicial |

Para ver el rendimiento (factor de tiempo real y frecuencia/tiempo de cada sensor): `ros2 topic echo /diagnostics`.

## Interfaz (igual que Gazebo)

Con `gazebo_compat:=True`. Todo bajo `/caddy_ai2/`:

| Topic | Tipo |
|---|---|
| `rs_helios_5515/points` | `PointCloud2` |
| `sick_lms_291/scan` | `LaserScan` |
| `sbg_ig500n/imu`, `vectornav/imu` | `Imu` |
| `sbg_ig500n/navsat` | `NavSatFix` |
| `<zed>/zed_node/left/color/rect/image`, `.../camera_info` | `Image`, `CameraInfo` |
| `<zed>/zed_node/depth/depth_registered` | `Image` (`32FC1`, metros) |
| `<zed>/zed_node/point_cloud/cloud_registered` | `PointCloud2` |
| `bicycle_steering_controller/odometry`, `ground_truth/odometry` | `Odometry` |
| `bicycle_steering_controller/reference` (entrada) | `TwistStamped` |

**Cómo se consigue** (sin copiar ningún mensaje, salvo la odometría y el mando, que son pequeños):

- `mvsim_node` se lanza **directamente** (no con el `launch_world.launch.py` de MVSim, que no admite remapeos)
  con **remapeos de ROS** a los nombres de Gazebo.
- El árbol TF `base_footprint → base_link → enlaces de sensores` lo publica `robot_state_publisher` con la
  **misma URDF** que Gazebo (sin plugins); MVSim lo hace con `publish_sensor_tf:=False`.
- MVSim usa como `frame_id` de cada mensaje la etiqueta interna del sensor (p. ej. `rs_helios_5515_points`,
  `zed_2i_left_depth`) y no se puede cambiar. Se resuelven con **TF estáticos de identidad** desde el enlace
  correspondiente de la URDF. **Los `frame_id` de los mensajes no son idénticos a los de Gazebo**; lo que se
  unifica es el árbol TF (cualquier consumidor que use el frame del propio mensaje funciona igual).
- `scripts/gazebo_compat_adapter.py`: esos alias de frame, `odom → bicycle_steering_controller/odometry` +
  TF `odom → base_footprint` (en MVSim `base_link` está en el suelo: es el `base_footprint` de Gazebo) y el mando
  `TwistStamped → Twist`.
- Profundidad en `32FC1` (metros), como Gazebo: `ros_depth_image_encoding`, MVSim ≥ 2.0.

**No se publica** (MVSim no lo simula): `vectornav/magnetic`, `vectornav/pressure`,
`sbg_ig500n/navsat/{base,front_axle,rear_axle}/fix`.

## Diferencias respecto a Gazebo

- `parking_garage`: solo la planta baja. Con `garage_visual:=True` (mallado 3D solo visual) el Helios pierde media
  circunferencia cuando el sensor está en y ≈ 0 (causa sin identificar), por eso está desactivado.
- Las ZED van a la mitad de resolución (640×314 en lugar de 1280×628) con el mismo campo de visión: a resolución
  completa cada ZED es lo que más cargó a Gazebo. Se cambia en `config/mvsim_sensors.yaml`.
- Los `frame_id` de los mensajes (ver arriba).
- Las ganancias del control de velocidad y dirección (`mvsim/caddy_ai2.vehicle.xml.j2`) son **provisionales**:
  hay que ajustarlas contra el vehículo real (MVSim 2.0 trae `mvsim-pid-tuner`).
- El modelo 3D de la carrocería (`meshes/caddy_ai2_chassis.stl`) y su desplazamiento son una estimación.

## Mundo `road_circuit` (demo de MVSim con rampas y puente)

```bash
ros2 launch caddy_ai2_ros2_mvsim_simulation sim_mvsim.launch.py world:=road_circuit
python3 tools/drive_route.py --route all          # piloto automático: vuelta + colina + puente
```

El escenario (calzadas de 4 m con baches, colina interior de 4 m, tablero del puente a 4 m con columnas, cielo) se
toma **tal cual** de `mvsim_tutorial/demo_road_circuit1.world.xml` de la instalación de MVSim (no se copia);
solo se sustituye su vehículo por el Caddy con sus sensores. `tools/drive_route.py` usa solo la interfaz
unificada (`ground_truth/odometry` y `bicycle_steering_controller/reference`). Las esquinas de las calzadas son
más cerradas que el radio de giro del Caddy (~3,9 m): toma arcos de 6 m y se sale un poco de la calzada. Las
barandillas del puente están desactivadas en el propio demo ("TODO: fix collisions at different heights").

### EasyNav 3D en el circuito

```bash
ros2 launch caddy_ai2_ros2_mvsim_simulation sim_mvsim.launch.py world:=road_circuit headless:=True use_rviz:=False
ros2 launch caddy_ai2_ros2_navigation navigation_3d.launch.py \
    params_file:=$(ros2 pkg prefix caddy_ai2_ros2_mvsim_simulation)/share/caddy_ai2_ros2_mvsim_simulation/config/easynav_3d_road_circuit.yaml
python3 tools/nav_goals.py            # secuencia de 8 objetivos (calzadas, colina, puente)
```

Con esto EasyNav (localización AMCL, A* sobre el NavMap, Regulated Pure Pursuit) completa los 8 objetivos, incluidos
el paso bajo el puente y el cruce del tablero a 4 m. Cómo está montado y qué hizo falta:

- **Mapa 3D analítico** (`maps/road_circuit_bonxai.pcd` y `road_circuit.navmap`): `tools/make_circuit_map.py` muestrea
  los propios mapas de alturas, el tablero y las columnas del mundo de MVSim (la verdad exacta, sin sensor ni pose 3D;
  la verdad de terreno de MVSim es 2D). Validado contra el Helios simulado: mediana 0,14 m, 92 % a menos de 0,3 m.
  Quita el terreno que queda a menos de 1,9 m bajo el tablero (`--clearance`): si no, el mallador lo mezcla con la cara
  superior del tablero y A* no encuentra camino a la colina-tablero. NavMap: `pointcloud_to_navmap --resolution 0.3
  --max-slope-deg 35`.
- **Helios con `ignore_parent_body`** (`mvsim/sensors.xml.j2`): sin él, el lidar ve la carrocería del propio vehículo
  (por defecto una caja que lo encierra y todos los rayos devuelven 0,6-2 m).
- **`config/easynav_3d_road_circuit.yaml`**: copia de la del parking con los mapas del circuito y dos cambios: el filtro
  de obstáculos del NavMap más holgado (`min_height` 0,5 y `min_height_per_meter` 0,15, por la diferencia entre el mapa
  analítico y el terreno en las pendientes) y **`z_min_filter: 0.45` en `CollisionSafetyReflex`,
  `ObstacleTooCloseEvaluator` y `SafeRetreatRecovery`**: su valor por defecto (0,0) cuenta como obstáculo cualquier
  punto sobre el suelo, y el terreno junto a la calzada sube ~0,3 m: el Caddy se quedaba atascado en las esquinas en un
  bucle de "obstáculo cerca, retroceder, colisión inminente". El parking tiene el mismo valor por defecto.
- `tools/nav_goals.py` inicializa AMCL con la pose real (verdad de terreno), no con la de arranque: si el Caddy ya no
  está en el origen, un `/initialpose` en (0, 0) lo pierde.
- Limitación conocida: A* no respeta el radio mínimo de giro del Caddy (~3,9 m) y las esquinas de las calzadas son de
  4 m; Pure Pursuit las toma con arcos que se salen un poco de la calzada.

**Vídeo** (MVSim ≥ 2.0 graba la vista 3D en tiempo de simulación, también en headless; si el nombre lleva un patrón
`%06d` guarda imágenes sueltas y no hace falta `ffmpeg` en la máquina del simulador):

```bash
ros2 launch caddy_ai2_ros2_mvsim_simulation sim_mvsim.launch.py world:=road_circuit headless:=True use_rviz:=False \
    video_output:=$HOME/frames/f_%06d.jpg video_fps:=15 video_duration:=125 video_close_when_done:=True
```

## Verificado

En la máquina del laboratorio (RTX 4080 SUPER), headless, con MVSim 2.0.0 y todos los sensores activos:
factor de tiempo real 1,00; Helios 9,6 Hz (nominal 10), ZED 14–14,7 Hz (15), SICK 57–65 Hz (75), IMU 75–87 Hz (100);
carga ≈ 0,75 núcleos de CPU y 6 % de GPU. Los 13 frames de los mensajes se resuelven desde `base_footprint`; el suelo
cae en z ≈ 0,01 m tanto para la nube de las ZED como para su imagen de profundidad y el Helios. El vehículo avanza,
gira, la odometría de las ruedas coincide con la verdad de terreno (≈ 3 %) y choca con las paredes.

## Mapa de ocupación

```bash
python3 tools/make_garage_grid.py planta_baja.pcd --out maps/parking_garage_ground --zmin 0.1 --zmax 1.5
```

Genera `<out>.yaml` + `<out>.pgm` (formato `map_server` de ROS) a partir de un PCD con z = 0 ≈ suelo (los
generados con `navmap_map_builder`). Una celda es ocupada si algún punto cae en la banda de alturas (excluye
suelo y techo). Descarta grupos pequeños de celdas (`--min-component`, 12 por defecto): con `navmap_map_builder`
el Helios ve el costado del propio robot en cada posición visitada y deja celdas sueltas que, sin este filtro,
atascan el chasis al arrancar.

## Instalación

MVSim **2.0** aún no está en `apt` (ahí hay la 1.5.0): se incluye como submódulo en `caddy_ai2_ros2_sim`
(`src/mvsim`, etiqueta `2.0.0`) y se compila con colcon. Necesita `ros-jazzy-mrpt-imgui`,
`ros-jazzy-mrpt-imgui-vendor` y el resto de dependencias de `ros-jazzy-mvsim` (basta instalar este último:
`sudo apt install ros-jazzy-mvsim ros-jazzy-mrpt-imgui ros-jazzy-mrpt-imgui-vendor`).
