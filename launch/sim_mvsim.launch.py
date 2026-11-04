#!/usr/bin/env python3
"""
Simulacion MVSim del Caddy AI2 en la planta baja del parking. Requiere MVSim >= 2.0.

Genera el vehiculo y el mundo (jinja2) desde la misma fuente unica que usa Gazebo,
caddy_ai2_ros2_description/bringup/config/robot_params.yaml, y lanza mvsim_node DIRECTAMENTE
(no el launch_world.launch.py de MVSim, que no admite remapeos de ROS).

    ros2 launch caddy_ai2_ros2_mvsim_simulation sim_mvsim.launch.py
    ros2 launch caddy_ai2_ros2_mvsim_simulation sim_mvsim.launch.py headless:=True use_rviz:=False

Sensores (los de robot_params.yaml con enabled: true): Helios RS-5515 3D, SICK LMS291 2D,
IMU SBG IG-500N, GPS SBG, IMU VN-100 y las ZED 2i activas. Parametros de simulacion en
config/mvsim_sensors.yaml.

OBJETIVO: misma interfaz que la simulacion de Gazebo (y que el vehiculo real). Con gazebo_compat:=True
(por defecto):
  - Nombres de topics de Gazebo: remapeos de ROS sobre mvsim_node (no copian ningun mensaje).
  - Arbol TF de Gazebo: robot_state_publisher con la misma URDF (sin plugins) +
    publish_sensor_tf:=False en MVSim.
  - Profundidad en 32FC1 (metros), como Gazebo.
  - Los frame_id de los mensajes son los internos de MVSim (no se pueden cambiar): se resuelven con
    TF estaticos de identidad desde los enlaces de la URDF (scripts/gazebo_compat_adapter.py).
  - Mando (TwistStamped) y odometria (odom -> base_footprint): el mismo adaptador.
Con gazebo_compat:=False, MVSim "crudo": /caddy_ai2/<mvsim_name> y sus propios TF.

Limitaciones: MVSim tiene fisica 2D, asi que solo esta la planta baja (sin rampas ni otras plantas).
"""
import math
import os
import tempfile

import yaml
from jinja2 import Environment, FileSystemLoader, StrictUndefined

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NS = 'caddy_ai2'


def _truthy(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() == 'true'


def _read_pgm_size(path):
    """Ancho y alto de un PGM binario (P5)."""
    with open(path, 'rb') as f:
        tokens = []
        while len(tokens) < 3:
            line = f.readline().decode('ascii', 'ignore').split('#')[0].split()
            tokens += line
        return int(tokens[1]), int(tokens[2])


def _sensors(robot_params, cfg):
    """Lista de sensores activos con todo lo que necesita la plantilla del mundo."""
    wheel_radius = robot_params['wheel_radius']
    out = []
    for key, s in robot_params['sensors'].items():
        if not s.get('enabled', True):
            continue
        kind = s['type']
        c = dict(cfg['zed_2i'] if kind == 'camera_rgbd' else cfg[key])
        pose = s['pose']
        c.update(
            kind=kind,
            key=key,
            frame_id=s['frame_id'],
            mvsim_name=key if kind == 'camera_rgbd' else c['mvsim_name'],
            x=pose['x'], y=pose['y'],
            # robot_params da z respecto a base_link (a wheel_radius del suelo); MVSim, respecto al suelo
            z=pose['z'] + wheel_radius,
            yaw_deg=math.degrees(pose.get('yaw', 0.0)),
            pitch_deg=math.degrees(pose.get('pitch', 0.0)),
            roll_deg=math.degrees(pose.get('roll', 0.0)),
        )
        if kind == 'camera_rgbd':
            fx = (c['image_width'] / 2.0) / math.tan(math.radians(c['horizontal_fov_deg']) / 2.0)
            c.update(fx=round(fx, 3), cx=c['image_width'] / 2.0, cy=c['image_height'] / 2.0)
        out.append(c)
    return out


def _interface(sensors, ns=NS):
    """Remapeos de topics (MVSim -> Gazebo) y alias de frames (enlace URDF -> frame de MVSim).

    Los nombres de destino son los de caddy_ai2_ros2_gazebo_simulation/bringup/config/gz_msg_bridge.yaml.j2.
    Los de origen y los frames, los que publica MVSim (etiqueta del sensor; ver docs/sensors.rst de MVSim).
    """
    remaps, aliases = [], []

    def remap(src, dst):
        remaps.append(('/%s/%s' % (ns, src), '/%s/%s' % (ns, dst)))

    for s in sensors:
        n, k, link = s['mvsim_name'], s['kind'], s['frame_id']
        if k == 'imu':
            dst = 'vectornav/imu' if s['key'] == 'imu_vn100' else 'sbg_ig500n/imu'
            remap(n, dst)
            aliases.append((link, n))
        elif k == 'gps':
            remap(n, 'sbg_ig500n/navsat')
            aliases.append((link, n))
        elif k == 'lidar_2d':
            remap(n, 'sick_lms_291/scan')
            aliases.append((link, n))
        elif k == 'lidar_3d':
            remap(n + '_points', 'rs_helios_5515/points')
            aliases.append((link, n + '_points'))
        elif k == 'camera_rgbd':
            z = '%s/zed_node/' % n
            remap(n + '_rgb/image_raw', z + 'left/color/rect/image')
            remap(n + '_rgb/camera_info', z + 'left/color/rect/camera_info')
            remap(n + '_depth/image_raw', z + 'depth/depth_registered')
            remap(n + '_points', z + 'point_cloud/cloud_registered')
            # La profundidad y el RGB estan en un frame optico (+Z adelante); la nube, en el del sensor.
            aliases += [(link + '_optical', n + '_depth'), (link + '_optical', n + '_rgb/image_raw'),
                        (link, n + '_points')]
    remaps.append(('/%s/base_pose_ground_truth' % ns, '/%s/ground_truth/odometry' % ns))
    return remaps, ['%s->%s' % a for a in aliases]


def _render_sensor_fragments(robot_params, prefix):
    """Fragmentos URDF de los sensores activos (los mismos que usa Gazebo, sin plugins de Gazebo).

    Igual que _render_sensor_fragments(gazebo=False) de caddy_ai2_ros2_gazebo_simulation.
    """
    fragments = []
    for sensor_name, cfg in robot_params.get('sensors', {}).items():
        if not cfg.get('enabled', True):
            continue
        s_share = get_package_share_directory(cfg['package'])
        if not os.path.isfile(os.path.join(s_share, 'description', 'sensor.urdf.j2')):
            continue
        pose = cfg['pose']
        env = Environment(loader=FileSystemLoader(os.path.join(s_share, 'description')),
                          keep_trailing_newline=True)
        fragments.append(env.get_template('sensor.urdf.j2').render(
            prefix=prefix, sensor_name=sensor_name, sensor_type=cfg['type'], frame_id=cfg['frame_id'],
            x=pose['x'], y=pose['y'], z=pose['z'],
            roll=pose['roll'], pitch=pose['pitch'], yaw=pose['yaw'],
            sensor_share=s_share, gazebo=False))
    return '\n'.join(fragments)


def _launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('caddy_ai2_ros2_mvsim_simulation')
    desc = get_package_share_directory('caddy_ai2_ros2_description')
    mvsim = get_package_share_directory('mvsim')
    compat = _truthy(context, 'gazebo_compat')

    robot_params_path = os.path.join(desc, 'bringup', 'config', 'robot_params.yaml')
    with open(robot_params_path) as f:
        rp = yaml.safe_load(f)
    with open(os.path.join(pkg, 'config', 'mvsim_sensors.yaml')) as f:
        cfg = yaml.safe_load(f)

    grid_yaml = os.path.join(pkg, 'maps', 'parking_garage_ground.yaml')
    with open(grid_yaml) as f:
        grid = yaml.safe_load(f)
    w_px, h_px = _read_pgm_size(os.path.join(os.path.dirname(grid_yaml), grid['image']))
    ox, oy = grid['origin'][0], grid['origin'][1]
    res = grid['resolution']

    params = dict(rp)
    params.update(
        max_steer_deg=round(math.degrees(rp['max_steer_angle']), 3),
        chassis_mass=rp['mass'] - 4 * rp['wheel_mass'],
        chassis_mesh=os.path.join(pkg, 'meshes', 'caddy_ai2_chassis.stl'),
    )
    params.pop('sensors', None)
    sensors = _sensors(rp, cfg)
    world = dict(
        grid_yaml=grid_yaml,
        floor=dict(x_min=ox, y_min=oy, x_max=ox + w_px * res, y_max=oy + h_px * res),
        defs=os.path.join(mvsim, 'definitions'),
        georef=cfg['georeference'],
        sensors=sensors,
        init_x=float(LaunchConfiguration('x').perform(context)),
        init_y=float(LaunchConfiguration('y').perform(context)),
        init_yaw_deg=float(LaunchConfiguration('yaw_deg').perform(context)),
    )

    env = Environment(loader=FileSystemLoader(os.path.join(pkg, 'mvsim')),
                      undefined=StrictUndefined, keep_trailing_newline=True)
    out_dir = tempfile.mkdtemp(prefix='caddy_ai2_mvsim_')
    for tpl, ctx in (('caddy_ai2.vehicle.xml.j2', params), ('parking_garage.world.xml.j2', world)):
        with open(os.path.join(out_dir, tpl[:-3]), 'w') as f:
            f.write(env.get_template(tpl).render(**ctx))
    world_file = os.path.join(out_dir, 'parking_garage.world.xml')
    print('[sim_mvsim] mundo generado en', world_file)

    remaps, aliases = _interface(sensors) if compat else ([], [])
    actions = []
    if compat:
        # Misma URDF que la simulacion de Gazebo (sin plugins): robot_state_publisher da el arbol TF
        # base_footprint -> base_link -> enlaces de sensores, igual que en Gazebo y en el vehiculo real.
        urdf_dir = os.path.join(desc, 'description', 'model', 'urdf')
        urdf = Environment(loader=FileSystemLoader(urdf_dir), keep_trailing_newline=True).get_template(
            'caddy_ai2_model.urdf.j2').render(
            namespace=NS, prefix='', sensors_urdf_fragment=_render_sensor_fragments(rp, ''),
            desc_share=desc, **rp)
        actions += [
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 namespace=NS, output='screen', parameters=[{'robot_description': urdf}]),
            Node(package='caddy_ai2_ros2_mvsim_simulation', executable='gazebo_compat_adapter.py',
                 name='gazebo_compat_adapter', output='screen',
                 parameters=[{'namespace': NS, 'prefix': '', 'frame_aliases': aliases}]),
        ]

    actions.append(Node(
        package='mvsim', executable='mvsim_node', name='mvsim', output='screen',
        remappings=remaps,
        parameters=[{
            'world_file': world_file,
            'headless': _truthy(context, 'headless'),
            # En compat, estos TF los dan robot_state_publisher y el adaptador (odom -> base_footprint);
            # los de MVSim chocarian con el arbol de la URDF (MVSim pone base_link en el suelo).
            'do_fake_localization': _truthy(context, 'do_fake_localization'),
            'publish_tf_odom2baselink': (not compat) and _truthy(context, 'publish_tf_odom2baselink'),
            'publish_sensor_tf': not compat,
            'force_publish_vehicle_namespace': _truthy(context, 'force_publish_vehicle_namespace'),
            'publish_log_topics': _truthy(context, 'publish_log_topics'),
            'realtime_factor': float(LaunchConfiguration('realtime_factor').perform(context)),
        }]))

    if _truthy(context, 'use_rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                            arguments=['-d', os.path.join(pkg, 'rviz', 'sim_mvsim.rviz')]))
    return actions


def generate_launch_description():
    def arg(name, default, desc):
        return DeclareLaunchArgument(name, default_value=default, description=desc)

    return LaunchDescription([
        arg('gazebo_compat', 'True', 'Misma interfaz que Gazebo: nombres de topics (remapeos), TF (URDF + '
                                      'alias), profundidad 32FC1, mando y odometria. Con False, MVSim crudo'),
        arg('headless', 'False', 'Sin GUI (renderizado EGL en la GPU)'),
        arg('do_fake_localization', 'False', 'Publica map -> odom desde la verdad de terreno (Gazebo no lo '
                                              'hace: lo da el localizador)'),
        arg('publish_tf_odom2baselink', 'True', 'Solo con gazebo_compat:=False: TF odom -> base_link de MVSim'),
        arg('force_publish_vehicle_namespace', 'True', 'Topics bajo /caddy_ai2/ aunque haya un solo vehiculo'),
        arg('publish_log_topics', 'False', 'Publica cada columna del CSV logger (alta frecuencia)'),
        arg('realtime_factor', '1.0', 'Velocidad: <1 mas lento, >1 mas rapido, 0 lo mas rapido posible'),
        arg('use_rviz', 'True', 'Lanza RViz2'),
        arg('x', '0.0', 'Posicion inicial x (m, mundo)'),
        arg('y', '0.0', 'Posicion inicial y (m, mundo)'),
        arg('yaw_deg', '0.0', 'Orientacion inicial (grados)'),
        OpaqueFunction(function=_launch_setup),
    ])
