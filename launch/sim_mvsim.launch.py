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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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
            # robot_params da la pose respecto a base_link, que esta wheelbase/2 por delante del origen de MVSim (eje trasero)
            x=pose['x'] + robot_params['wheelbase'] / 2.0, y=pose['y'],
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


def _circuit_scenario(mvsim_share):
    """Escenario de demo_road_circuit1 (todo lo anterior a las definiciones de vehiculos), tal cual."""
    path = os.path.join(mvsim_share, 'mvsim_tutorial', 'demo_road_circuit1.world.xml')
    with open(path) as f:
        text = f.read()
    start = text.index('<element')
    end = text.rfind('<!--', 0, text.index('Vehicle classes definition'))
    return text[start:end]


def _video(context, pkg_out_dir):
    """Opciones de grabacion de video (MVSim >= 2.0), o None. Si el nombre lleva %06d guarda PNG/JPG sueltos
    (no hace falta ffmpeg en la maquina del simulador)."""
    out = LaunchConfiguration('video_output').perform(context).strip()
    if not out:
        return None
    return dict(
        output_file=out,
        fps=float(LaunchConfiguration('video_fps').perform(context)),
        width=int(LaunchConfiguration('video_width').perform(context)),
        height=int(LaunchConfiguration('video_height').perform(context)),
        start_time=float(LaunchConfiguration('video_start').perform(context)),
        duration=float(LaunchConfiguration('video_duration').perform(context)),
        close_when_done=_truthy(context, 'video_close_when_done'),
        cam_distance=float(LaunchConfiguration('video_cam_distance').perform(context)),
        cam_azimuth=float(LaunchConfiguration('video_cam_azimuth').perform(context)),
        cam_elevation=float(LaunchConfiguration('video_cam_elevation').perform(context)),
        fov_deg=float(LaunchConfiguration('video_fov').perform(context)),
    )


def _launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('caddy_ai2_ros2_mvsim_simulation')
    desc = get_package_share_directory('caddy_ai2_ros2_description')
    mvsim = get_package_share_directory('mvsim')
    compat = _truthy(context, 'gazebo_compat')
    control = LaunchConfiguration('control').perform(context).strip()
    if control not in ('builtin', 'ros2_control'):
        raise RuntimeError("control debe ser 'builtin' o 'ros2_control', no '%s'" % control)
    steering_mode = LaunchConfiguration('steering_mode').perform(context).strip()
    if steering_mode not in ('ackermann', 'bicycle'):
        raise RuntimeError("steering_mode debe ser 'ackermann' o 'bicycle', no '%s'" % steering_mode)
    out_dir = tempfile.mkdtemp(prefix='caddy_ai2_mvsim_')
    ctrl_yaml = os.path.join(out_dir, 'ros2_control_controllers.yaml')

    robot_params_path = os.path.join(desc, 'bringup', 'config', 'robot_params.yaml')
    with open(robot_params_path) as f:
        rp = yaml.safe_load(f)
    with open(os.path.join(pkg, 'config', 'mvsim_sensors.yaml')) as f:
        cfg = yaml.safe_load(f)

    which = LaunchConfiguration('world').perform(context).strip()
    params = dict(rp)
    params.update(
        max_steer_deg=round(math.degrees(rp['max_steer_angle']), 3),
        chassis_mass=rp['mass'] - 4 * rp['wheel_mass'],
        chassis_mesh=(os.path.join(pkg, 'meshes', 'caddy_ai2_chassis.stl')
                      if _truthy(context, 'chassis_visual') else None),
    )
    params.pop('sensors', None)
    with open(os.path.join(pkg, 'config', 'mvsim_vehicle.yaml')) as f:
        veh = yaml.safe_load(f)['ros2_control']
    params.update(control=control, steering_mode=steering_mode, controllers_yaml=ctrl_yaml, wheel_kp=veh['wheel_pid']['kp'],
                  wheel_ki=veh['wheel_pid']['ki'], wheel_max_torque=veh['wheel_pid']['max_torque'],
                  max_steer_rate_deg=veh['max_steer_rate_deg'])
    if control == 'ros2_control':
        # YAML de la misma cadena de controladores que Gazebo y el vehiculo real, con la geometria de robot_params.yaml
        cenv = Environment(loader=FileSystemLoader(os.path.join(pkg, 'config')), undefined=StrictUndefined,
                           keep_trailing_newline=True)
        with open(ctrl_yaml, 'w') as f:
            f.write(cenv.get_template('ros2_control_controllers.yaml.j2').render(steering_mode=steering_mode, **rp))
    sensors = _sensors(rp, cfg)

    world = dict(
        defs=os.path.join(mvsim, 'definitions'),
        georef=cfg['georeference'],
        sensors=sensors,
        init_x=float(LaunchConfiguration('x').perform(context)),
        init_y=float(LaunchConfiguration('y').perform(context)),
        init_yaw_deg=float(LaunchConfiguration('yaw_deg').perform(context)),
    )

    if which == 'parking_garage':
        grid_yaml = os.path.join(pkg, 'maps', 'parking_garage_ground.yaml')
        with open(grid_yaml) as f:
            grid = yaml.safe_load(f)
        w_px, h_px = _read_pgm_size(os.path.join(os.path.dirname(grid_yaml), grid['image']))
        ox, oy = grid['origin'][0], grid['origin'][1]
        res = grid['resolution']

        # Mallado del parking de Gazebo como modelo visual (no se copia: es GPL, se usa el instalado).
        garage_mesh = None
        if _truthy(context, 'garage_visual'):
            mesh = os.path.join(get_package_share_directory('caddy_ai2_ros2_gazebo_simulation'), 'description',
                                'models', 'parking_garage', 'meshes', 'parking_garage.dae')
            if os.path.isfile(mesh):
                garage_mesh = mesh
            else:
                print('[sim_mvsim] AVISO: no se encuentra el mallado del parking, sin modelo visual:', mesh)
        world.update(
            garage_mesh=garage_mesh, grid_yaml=grid_yaml,
            floor=dict(x_min=ox, y_min=oy, x_max=ox + w_px * res, y_max=oy + h_px * res))
        world_tpl = 'parking_garage.world.xml.j2'
    elif which == 'road_circuit':
        world.update(scenario=_circuit_scenario(mvsim), video=_video(context, None), cam_distance=30)
        world_tpl = 'road_circuit.world.xml.j2'
    else:
        raise RuntimeError("world debe ser 'parking_garage' o 'road_circuit', no '%s'" % which)

    env = Environment(loader=FileSystemLoader(os.path.join(pkg, 'mvsim')),
                      undefined=StrictUndefined, keep_trailing_newline=True)
    for tpl, ctx in (('caddy_ai2.vehicle.xml.j2', params), (world_tpl, world)):
        with open(os.path.join(out_dir, tpl[:-3]), 'w') as f:
            f.write(env.get_template(tpl).render(**ctx))
    world_file = os.path.join(out_dir, world_tpl[:-3])
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
            desc_share=desc, base_footprint_offset_x=rp['wheelbase'] / 2.0, **rp)
        actions += [
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 namespace=NS, output='screen', parameters=[{'robot_description': urdf}]),
            Node(package='caddy_ai2_ros2_mvsim_simulation', executable='gazebo_compat_adapter.py',
                 name='gazebo_compat_adapter', output='screen',
                 parameters=[{'namespace': NS, 'prefix': '', 'frame_aliases': aliases,
                              # con ros2_control, la odometria, su TF y el mando son los del propio controlador
                              'republish_odom': control != 'ros2_control',
                              'convert_cmd': control != 'ros2_control'}]),
        ]

    if control == 'ros2_control':
        # Mismo orden que en Gazebo: cada spawner arranca cuando termina el anterior (los controladores encadenables
        # exportan sus interfaces de referencia al configurarse, y los de abajo deben existir antes).
        def spawner(name, ros_args=None):
            args = [name, '--param-file', ctrl_yaml, '--controller-manager-timeout', '120']
            if ros_args:
                args += ['--controller-ros-args', ros_args]
            return Node(package='controller_manager', executable='spawner', namespace=NS, output='screen',
                        arguments=args)
        s_jsb = spawner('joint_state_broadcaster')
        s_st = spawner('bicycle_to_ackermann_steering_adapter')
        s_tr = spawner('bicycle_to_ackermann_traction_adapter')
        # El TF de odometria (odom -> base_footprint) del bicycle_steering_controller va a /tf, como en Gazebo (alli lo hace
        # el plugin de ros2_control). Los controladores lo publican en ~/tf_odometry: el remapeo se le da al controlador
        # (en Jazzy, --controller-ros-args del spawner; remapearlo en mvsim_node no le llega).
        s_bi = spawner('bicycle_steering_controller',
                       ros_args='-r /%s/bicycle_steering_controller/tf_odometry:=/tf' % NS)
        if steering_mode == 'ackermann':
            actions += [s_jsb,
                        RegisterEventHandler(OnProcessExit(target_action=s_jsb, on_exit=[s_st])),
                        RegisterEventHandler(OnProcessExit(target_action=s_st, on_exit=[s_tr])),
                        RegisterEventHandler(OnProcessExit(target_action=s_tr, on_exit=[s_bi]))]
        else:   # bicycle: MVSim reparte el angulo a las dos ruedas, no hace falta el adaptador de direccion
            actions += [s_jsb,
                        RegisterEventHandler(OnProcessExit(target_action=s_jsb, on_exit=[s_tr])),
                        RegisterEventHandler(OnProcessExit(target_action=s_tr, on_exit=[s_bi]))]

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
        arg('control', 'builtin', "Control del vehiculo: 'builtin' (cmd_vel -> controlador twist de MVSim) o 'ros2_control' "
                                  "(la pila real: bicycle_steering_controller -> adaptadores -> juntas; necesita el PR MRPT/mvsim#169 "
                                  "(ros2_control para Ackermann))"),
        arg('steering_mode', 'ackermann', "Solo con control:=ros2_control. 'ackermann': dos juntas de direccion (FL, FR) con el adaptador "
                                         "bicycle->ackermann; 'bicycle': una junta virtual con el angulo equivalente (sin adaptador de direccion)"),
        arg('world', 'parking_garage', "Mundo: 'parking_garage' (planta baja) o 'road_circuit' (demo_road_circuit1 de MVSim)"),
        arg('video_output', '', 'Si no esta vacio, graba la vista 3D: archivo .mp4 (necesita ffmpeg) o patron de '
                                'fotogramas, p. ej. /ruta/f_%06d.jpg (no necesita ffmpeg)'),
        arg('video_fps', '15', 'Fotogramas por segundo del video'),
        arg('video_width', '1280', 'Ancho del video'),
        arg('video_height', '720', 'Alto del video'),
        arg('video_start', '1.0', 'Segundo de simulacion en que empieza a grabar'),
        arg('video_duration', '0', 'Segundos de simulacion a grabar (0 = hasta parar)'),
        arg('video_close_when_done', 'False', 'Cierra el simulador al terminar la grabacion'),
        arg('video_cam_distance', '12', 'Distancia de la camara al Caddy (m)'),
        arg('video_cam_azimuth', '-150', 'Acimut de la camara respecto al rumbo del Caddy (grados)'),
        arg('video_cam_elevation', '22', 'Elevacion de la camara (grados)'),
        arg('video_fov', '60', 'Campo de vision de la camara del video (grados)'),
        arg('gazebo_compat', 'True', 'Misma interfaz que Gazebo: nombres de topics (remapeos), TF (URDF + '
                                      'alias), profundidad 32FC1, mando y odometria. Con False, MVSim crudo'),
        arg('chassis_visual', 'True', 'Dibuja la carroceria 3D del Caddy (puede tapar la vista del Helios)'),
        arg('garage_visual', 'False', 'Solo parking_garage. Dibuja el mallado 3D del parking (solo visual). Con True el Helios pierde media \n'
                                            'circunferencia cuando esta en y ~ 0 (causa sin identificar): por eso False'),
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
