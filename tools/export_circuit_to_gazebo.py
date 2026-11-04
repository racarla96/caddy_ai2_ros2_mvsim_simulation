#!/usr/bin/env python3
"""
Exporta el circuito de MVSim (demo_road_circuit1) a un mundo y un modelo de Gazebo, para tener el MISMO escenario 3D
en los dos simuladores (mismo mapa para EasyNav, mismos sensores, mismo vehiculo).

Lee el mundo de MVSim con make_circuit_map.py (mismos mapas de alturas, tablero y columnas) y escribe, en el
directorio de `caddy_ai2_ros2_gazebo_simulation/description`:

  models/road_circuit/model.config, model.sdf
  models/road_circuit/meshes/terrain.stl   malla del terreno (altura exacta de MVSim, rejilla --step)
  models/road_circuit/meshes/roads.stl     las calzadas, 4 mm por encima del terreno (solo visual: color de asfalto)
  world/road_circuit.sdf                   mismo mundo que parking_garage.sdf pero con este modelo

El terreno es una malla y no un `heightmap` de Gazebo: este exige una imagen cuadrada de 2^n+1 pixeles, normalmente de
8 bits (los baches de las calzadas, de 1-5 cm, sobre una montana de 29 m se perderian).
El tablero del puente es una caja (cara superior a 4.02 m) y las columnas, cilindros.

    export_circuit_to_gazebo.py --out <.../caddy_ai2_ros2_gazebo_simulation/description> [--step 1.0]
"""
import argparse
import os
import struct

import numpy as np

import make_circuit_map as mc


def write_stl(path, tris, name='mesh'):
    """STL binario. tris: (n, 3, 3)."""
    tris = np.asarray(tris, '<f4')
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    nrm = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    rec['n'], rec['v'] = nrm, tris.reshape(-1, 9)
    with open(path, 'wb') as f:
        f.write(name.encode().ljust(80, b'\0'))
        f.write(struct.pack('<I', len(tris)))
        f.write(rec.tobytes())


def grid_tris(X, Y, Z, mask=None):
    """Triangulos de una rejilla regular (2 por celda, diagonal fija). mask: celdas (ny-1, nx-1) a incluir."""
    ny, nx = Z.shape
    p = np.stack([X, Y, Z], -1)
    a, b, c, d = p[:-1, :-1], p[:-1, 1:], p[1:, 1:], p[1:, :-1]
    t1, t2 = np.stack([a, b, c], -2), np.stack([a, c, d], -2)           # normal +z (x hacia la derecha, y hacia arriba)
    if mask is None:
        mask = np.ones((ny - 1, nx - 1), bool)
    return np.concatenate([t1[mask], t2[mask]])


MODEL_SDF = """<?xml version="1.0" ?>
<sdf version="1.6">
  <!-- Generado por caddy_ai2_ros2_mvsim_simulation/tools/export_circuit_to_gazebo.py a partir de demo_road_circuit1 de
       MVSim (MRPT/mvsim, BSD-3). No editar a mano. -->
  <model name="road_circuit">
    <static>true</static>
    <link name="link">
      <visual name="terrain">
        <geometry><mesh><uri>model://road_circuit/meshes/terrain.stl</uri></mesh></geometry>
        <material><ambient>0.42 0.36 0.29 1</ambient><diffuse>0.50 0.43 0.34 1</diffuse><specular>0.02 0.02 0.02 1</specular></material>
      </visual>
      <collision name="terrain">
        <geometry><mesh><uri>model://road_circuit/meshes/terrain.stl</uri></mesh></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <visual name="roads">
        <geometry><mesh><uri>model://road_circuit/meshes/roads.stl</uri></mesh></geometry>
        <material><ambient>0.13 0.13 0.14 1</ambient><diffuse>0.18 0.18 0.19 1</diffuse><specular>0.02 0.02 0.02 1</specular></material>
      </visual>
{deck}
{columns}
    </link>
  </model>
</sdf>
"""


def box(name, cx, cy, cz, sx, sy, sz, rgb):
    return """      <visual name="{n}"><pose>{cx} {cy} {cz} 0 0 0</pose><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse></material></visual>
      <collision name="{n}"><pose>{cx} {cy} {cz} 0 0 0</pose><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>""".format(
        n=name, cx=cx, cy=cy, cz=cz, sx=sx, sy=sy, sz=sz, c=rgb)


def cyl(name, cx, cy, cz, r, length, rgb):
    return """      <visual name="{n}"><pose>{cx} {cy} {cz} 0 0 0</pose><geometry><cylinder><radius>{r}</radius><length>{l}</length></cylinder></geometry>
        <material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse></material></visual>
      <collision name="{n}"><pose>{cx} {cy} {cz} 0 0 0</pose><geometry><cylinder><radius>{r}</radius><length>{l}</length></cylinder></geometry></collision>""".format(
        n=name, cx=cx, cy=cy, cz=cz, r=r, l=length, c=rgb)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default=None, help='demo_road_circuit1.world.xml (por defecto, el de MVSim instalado)')
    ap.add_argument('--out', required=True, help='.../caddy_ai2_ros2_gazebo_simulation/description')
    ap.add_argument('--step', type=float, default=1.0, help='paso de la malla del terreno (m)')
    ap.add_argument('--deck-thickness', type=float, default=0.30)
    ap.add_argument('--template-world', default=None,
                    help='parking_garage.sdf del que copiar plugins, fisica y escena (por defecto, el del --out)')
    args = ap.parse_args()

    maps, planes, cols = mc.parse(args.world or mc.find_world())
    x0 = min(m['x0'] for m in maps); x1 = max(m['x0'] + (m['z'].shape[0] - 1) * m['res'] for m in maps)
    y0 = min(m['y0'] for m in maps); y1 = max(m['y0'] + (m['z'].shape[1] - 1) * m['res'] for m in maps)
    xs = np.arange(x0, x1 + 1e-6, args.step); ys = np.arange(y0, y1 + 1e-6, args.step)
    X, Y = np.meshgrid(xs, ys)
    Z = mc.height(maps, X.ravel(), Y.ravel()).reshape(X.shape)
    if np.isnan(Z).any():
        raise SystemExit('hay %d nodos sin altura: los mapas no cubren todo el recuadro' % np.isnan(Z).sum())

    mdl = os.path.join(args.out, 'models', 'road_circuit')
    os.makedirs(os.path.join(mdl, 'meshes'), exist_ok=True)
    terrain = grid_tris(X, Y, Z)
    write_stl(os.path.join(mdl, 'meshes', 'terrain.stl'), terrain, 'terrain')

    # calzadas: los mapas pequenos de la carretera (altura maxima < 0.1 m); celdas dentro de sus rectangulos
    road_mask = np.zeros((len(ys) - 1, len(xs) - 1), bool)
    for m in maps:
        if m['z'].max() < 0.1:
            rx0, rx1 = m['x0'], m['x0'] + (m['z'].shape[0] - 1) * m['res']
            ry0, ry1 = m['y0'], m['y0'] + (m['z'].shape[1] - 1) * m['res']
            cx = (xs[:-1] + xs[1:]) / 2; cy = (ys[:-1] + ys[1:]) / 2
            road_mask |= ((cx[None, :] >= rx0) & (cx[None, :] <= rx1) & (cy[:, None] >= ry0) & (cy[:, None] <= ry1))
    roads = grid_tris(X, Y, Z + 0.004, road_mask)
    write_stl(os.path.join(mdl, 'meshes', 'roads.stl'), roads, 'roads')

    deck = columns = ''
    for i, p in enumerate(planes):
        th = args.deck_thickness
        deck += box('deck%d' % i, (p['x_min'] + p['x_max']) / 2, (p['y_min'] + p['y_max']) / 2, p['z'] - th / 2,
                    p['x_max'] - p['x_min'], p['y_max'] - p['y_min'], th, '0.62 0.60 0.56') + '\n'
    for i, (cx, cy, r, zmax) in enumerate(cols):
        columns += cyl('column%d' % i, cx, cy, zmax / 2, r, zmax, '0.38 0.38 0.38') + '\n'
    with open(os.path.join(mdl, 'model.sdf'), 'w') as f:
        f.write(MODEL_SDF.replace('{deck}', deck.rstrip()).replace('{columns}', columns.rstrip()))
    with open(os.path.join(mdl, 'model.config'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<model>\n  <name>road_circuit</name>\n  <version>1.0</version>\n'
                '  <sdf version="1.6">model.sdf</sdf>\n  <description>Circuito de carretera de MVSim '
                '(demo_road_circuit1) exportado a Gazebo.</description>\n</model>\n')

    tpl = args.template_world or os.path.join(args.out, 'world', 'parking_garage.sdf')
    w = open(tpl).read()
    w = w.replace("world name='parking_garage_world'", "world name='road_circuit_world'")
    w = w.replace('<uri>model://parking_garage</uri>', '<uri>model://road_circuit</uri>')
    w = w.replace('<!-- Include the parking_garage model -->', '<!-- Circuito de MVSim (demo_road_circuit1), generado por export_circuit_to_gazebo.py -->')
    assert 'road_circuit' in w, 'la plantilla de mundo no tiene el include esperado'
    with open(os.path.join(args.out, 'world', 'road_circuit.sdf'), 'w') as f:
        f.write(w)

    print('malla del terreno: %d triangulos (%.1fx%.1f m, paso %.2f) | calzadas: %d | z %.2f..%.2f' %
          (len(terrain), x1 - x0, y1 - y0, args.step, len(roads), Z.min(), Z.max()))
    print('tablero: %d, columnas: %d' % (len(planes), len(cols)))
    for rel in ('models/road_circuit/meshes/terrain.stl', 'models/road_circuit/meshes/roads.stl',
                'models/road_circuit/model.sdf', 'world/road_circuit.sdf'):
        print('  %-42s %8.1f KB' % (rel, os.path.getsize(os.path.join(args.out, rel)) / 1024))


if __name__ == '__main__':
    main()
