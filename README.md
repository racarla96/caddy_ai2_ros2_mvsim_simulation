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
- **Mundo:** solo la **planta baja** del parking. MVSim tiene física 2D: no hay rampas ni otras plantas.
  El mapa de ocupación (`maps/parking_garage_ground.*`) sale de un PCD de la planta baja, ver más abajo.

## Uso

```bash
ros2 launch caddy_ai2_ros2_mvsim_simulation sim_mvsim.launch.py
ros2 launch caddy_ai2_ros2_mvsim_simulation sim_mvsim.launch.py headless:=True use_rviz:=False   # servidor, sin pantalla
```

| Argumento | Defecto | Descripción |
|---|---|---|
| `gazebo_compat` | `True` | Interfaz idéntica a Gazebo. Con `False`, MVSim "crudo" (`/caddy_ai2/<sensor>` y sus propios TF) |
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

- Solo planta baja (física 2D).
- Las ZED van a la mitad de resolución (640×314 en lugar de 1280×628) con el mismo campo de visión: a resolución
  completa cada ZED es lo que más cargó a Gazebo. Se cambia en `config/mvsim_sensors.yaml`.
- Los `frame_id` de los mensajes (ver arriba).
- Las ganancias del control de velocidad y dirección (`mvsim/caddy_ai2.vehicle.xml.j2`) son **provisionales**:
  hay que ajustarlas contra el vehículo real (MVSim 2.0 trae `mvsim-pid-tuner`).
- El modelo 3D de la carrocería (`meshes/caddy_ai2_chassis.stl`) y su desplazamiento son una estimación.

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
