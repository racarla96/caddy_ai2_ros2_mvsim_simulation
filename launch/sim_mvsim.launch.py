#!/usr/bin/env python3
"""
Simulacion MVSim del Caddy AI2 en la planta baja del parking.

Genera el vehiculo y el mundo (jinja2) desde la misma fuente unica que usa Gazebo,
caddy_ai2_ros2_description/bringup/config/robot_params.yaml, y arranca MVSim con ellos.

    ros2 launch caddy_ai2_ros2_mvsim_simulation sim_mvsim.launch.py
    ros2 launch caddy_ai2_ros2_mvsim_simulation sim_mvsim.launch.py headless:=True use_rviz:=False

Sensores (los de robot_params.yaml con enabled: true): Helios RS-5515 3D, SICK LMS291 2D,
IMU SBG IG-500N, GPS SBG, IMU VN-100 y las ZED 2i activas. Parametros de simulacion en
config/mvsim_sensors.yaml. Topics: /caddy_ai2/<mvsim_name> (force_publish_vehicle_namespace).

Limitaciones: MVSim tiene fisica 2D, asi que solo esta la planta baja (sin rampas ni otras plantas).
"""
import math
import os
import tempfile

import yaml
from jinja2 import Environment, FileSystemLoader, StrictUndefined

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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


def _launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('caddy_ai2_ros2_mvsim_simulation')
    desc = get_package_share_directory('caddy_ai2_ros2_description')
    mvsim = get_package_share_directory('mvsim')

    with open(os.path.join(desc, 'bringup', 'config', 'robot_params.yaml')) as f:
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
    world = dict(
        grid_yaml=grid_yaml,
        floor=dict(x_min=ox, y_min=oy, x_max=ox + w_px * res, y_max=oy + h_px * res),
        defs=os.path.join(mvsim, 'definitions'),
        georef=cfg['georeference'],
        sensors=_sensors(rp, cfg),
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

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mvsim, 'launch', 'launch_world.launch.py')),
        launch_arguments={
            'world_file': world_file,
            'headless': LaunchConfiguration('headless'),
            'do_fake_localization': LaunchConfiguration('do_fake_localization'),
            'publish_tf_odom2baselink': LaunchConfiguration('publish_tf_odom2baselink'),
            'force_publish_vehicle_namespace': LaunchConfiguration('force_publish_vehicle_namespace'),
            'publish_log_topics': LaunchConfiguration('publish_log_topics'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config_file': os.path.join(pkg, 'rviz', 'sim_mvsim.rviz'),
        }.items())]


def generate_launch_description():
    def arg(name, default, desc):
        return DeclareLaunchArgument(name, default_value=default, description=desc)

    return LaunchDescription([
        arg('headless', 'False', 'Sin GUI (renderizado EGL en la GPU)'),
        arg('do_fake_localization', 'True', 'Publica map -> odom desde la verdad de terreno'),
        arg('publish_tf_odom2baselink', 'True', 'Publica odom -> base_link'),
        arg('force_publish_vehicle_namespace', 'True', 'Topics bajo /caddy_ai2/ aunque haya un solo vehiculo'),
        arg('publish_log_topics', 'False', 'Publica cada columna del CSV logger (alta frecuencia)'),
        arg('use_rviz', 'True', 'Lanza RViz2'),
        arg('x', '0.0', 'Posicion inicial x (m, mundo)'),
        arg('y', '0.0', 'Posicion inicial y (m, mundo)'),
        arg('yaw_deg', '0.0', 'Orientacion inicial (grados)'),
        OpaqueFunction(function=_launch_setup),
    ])
