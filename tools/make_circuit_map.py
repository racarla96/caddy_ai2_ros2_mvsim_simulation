#!/usr/bin/env python3
"""
Genera el mapa 3D (nube de puntos .pcd de superficies) del mundo road_circuit a partir del propio mundo de MVSim.

MVSim da la altura del terreno bajo cada rueda a partir de sus elementos (mapas de alturas, planos, bloques).
Aqui se muestrean esos mismos elementos para obtener la geometria EXACTA del circuito, sin ruido de sensor ni
necesidad de la pose 3D del robot (la verdad de terreno de MVSim es 2D):

  - elevation_map: rejilla de alturas, nodos cada <resolution> m desde (corner_min_x, corner_min_y); filas = x,
    columnas = y; interpolacion bilineal. Varios mapas solapados: manda el mas alto.
  - horizontal_plane: la cara SUPERIOR del tablero del puente.
  - block support_column (cilindro): su superficie lateral.

Solo se muestrean las caras que miran hacia arriba o los laterales (nada de la cara inferior del tablero): una
nube sin posicion de sensor no distingue suelo de techo visto desde abajo y el NavMap saldria con un "suelo"
colgando del tablero.

Uso:
    make_circuit_map.py [--world demo_road_circuit1.world.xml] --out maps/road_circuit_bonxai.pcd [--step 0.3]
Despues: ros2 run navmap_tools pointcloud_to_navmap --input <pcd> --output road_circuit.navmap --resolution 0.3
"""
import argparse
import os
import re
import subprocess

import numpy as np


def find_world():
    prefix = subprocess.run(['ros2', 'pkg', 'prefix', 'mvsim'], capture_output=True, text=True).stdout.strip()
    return os.path.join(prefix, 'share', 'mvsim', 'mvsim_tutorial', 'demo_road_circuit1.world.xml')


def tag(body, name, default=None):
    m = re.search(r'<%s>\s*([^<]*?)\s*</%s>' % (name, name), body)
    return m.group(1) if m else default


def parse(path):
    text = re.sub(r'<!--.*?-->', '', open(path).read(), flags=re.S)
    maps, planes = [], []
    for cls, body in re.findall(r"<element class=['\"](\w+)['\"]>(.*?)</element>", text, re.S):
        if cls == 'elevation_map':
            mat = tag(body, 'elevation_data_matrix')
            if mat is None:
                raise SystemExit('elevation_map sin elevation_data_matrix (con imagen no soportado aqui)')
            z = np.array([[float(v) for v in r.split()] for r in re.sub(r'[\[\]]', '', mat).strip().split(';')
                          if r.strip()])
            maps.append(dict(x0=float(tag(body, 'corner_min_x', 0)), y0=float(tag(body, 'corner_min_y', 0)),
                             res=float(tag(body, 'resolution', 1)), z=z))
        elif cls == 'horizontal_plane':
            planes.append({k: float(tag(body, k)) for k in ('x_min', 'y_min', 'x_max', 'y_max', 'z')})
    cols = []
    cls_m = re.search(r'<block:class name="support_column">(.*?)</block:class>', text, re.S)
    if cls_m:
        radius = float(re.search(r'radius="([\d.]+)"', cls_m.group(1)).group(1))
        zmax = float(tag(cls_m.group(1), 'zmax', 4.0))
        for pose in re.findall(r'<block class="support_column">\s*<init_pose>\s*([^<]+?)\s*</init_pose>', text):
            x, y = (float(v) for v in pose.split()[:2])
            cols.append((x, y, radius, zmax))
    return maps, planes, cols


def height(maps, X, Y):
    """Altura del terreno (max de los mapas que cubren el punto); NaN fuera de todos."""
    H = np.full(X.shape, np.nan)
    for m in maps:
        nr, nc = m['z'].shape
        u = (X - m['x0']) / m['res']; v = (Y - m['y0']) / m['res']
        inside = (u >= 0) & (u <= nr - 1) & (v >= 0) & (v <= nc - 1)
        i0 = np.clip(np.floor(u).astype(int), 0, nr - 2); j0 = np.clip(np.floor(v).astype(int), 0, nc - 2)
        fu, fv = u - i0, v - j0
        z = m['z']
        h = (z[i0, j0] * (1 - fu) * (1 - fv) + z[i0 + 1, j0] * fu * (1 - fv)
             + z[i0, j0 + 1] * (1 - fu) * fv + z[i0 + 1, j0 + 1] * fu * fv)
        H = np.where(inside & (np.isnan(H) | (h > H)), h, H)
    return H


def write_pcd(path, pts):
    hdr = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\n'
           'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n' % (len(pts), len(pts)))
    with open(path, 'wb') as f:
        f.write(hdr.encode())
        f.write(pts.astype('<f4').tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default=None, help='demo_road_circuit1.world.xml (por defecto, el de MVSim instalado)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--step', type=float, default=0.3, help='separacion de muestreo (m)')
    ap.add_argument('--clearance', type=float, default=1.9,
                    help='altura libre minima (m) para que el robot quepa: se quita el terreno que queda a menos de '
                         'esta distancia por debajo del tablero (0 = no quitar nada)')
    args = ap.parse_args()

    maps, planes, cols = parse(args.world or find_world())
    x0 = min(m['x0'] for m in maps); x1 = max(m['x0'] + (m['z'].shape[0] - 1) * m['res'] for m in maps)
    y0 = min(m['y0'] for m in maps); y1 = max(m['y0'] + (m['z'].shape[1] - 1) * m['res'] for m in maps)
    print('%d mapas de alturas, %d planos, %d columnas | x %.0f..%.0f, y %.0f..%.0f' %
          (len(maps), len(planes), len(cols), x0, x1, y0, y1))

    gx, gy = np.meshgrid(np.arange(x0, x1 + 1e-6, args.step), np.arange(y0, y1 + 1e-6, args.step))
    X, Y = gx.ravel(), gy.ravel()
    Z = height(maps, X, Y)
    ok = ~np.isnan(Z)
    # El terreno que queda justo debajo del tablero (menos de `clearance` m) no es transitable y, al estar a 0.3-1 m
    # de la cara superior del tablero, el mallador lo mezcla con ella y rompe la conectividad colina -> tablero.
    keep = ok.copy()
    removed = 0
    if args.clearance > 0:
        for p in planes:
            under = ((X >= p['x_min']) & (X <= p['x_max']) & (Y >= p['y_min']) & (Y <= p['y_max'])
                     & (Z > p['z'] - args.clearance) & (Z <= p['z'] + 0.05))
            removed += int((under & keep).sum())
            keep &= ~under
    parts = [np.stack([X[keep], Y[keep], Z[keep]], 1)]
    print('terreno: %d puntos, z %.2f..%.2f (quitados %d bajo el tablero)' % (keep.sum(), np.nanmin(Z), np.nanmax(Z), removed))

    for p in planes:
        px, py = np.meshgrid(np.arange(p['x_min'], p['x_max'] + 1e-6, args.step),
                             np.arange(p['y_min'], p['y_max'] + 1e-6, args.step))
        parts.append(np.stack([px.ravel(), py.ravel(), np.full(px.size, p['z'])], 1))
        print('tablero: %d puntos a z=%.2f (x %.0f..%.0f, y %.0f..%.0f)' % (px.size, p['z'], p['x_min'], p['x_max'],
                                                                           p['y_min'], p['y_max']))
    for (cx, cy, r, zmax) in cols:
        ang = np.linspace(0, 2 * np.pi, 12, endpoint=False)
        zz = np.arange(0.0, zmax + 1e-6, args.step)
        A, ZZ = np.meshgrid(ang, zz)
        parts.append(np.stack([cx + r * np.cos(A).ravel(), cy + r * np.sin(A).ravel(), ZZ.ravel()], 1))
    pts = np.vstack(parts)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    write_pcd(args.out, pts)
    print('escrito %s: %d puntos' % (args.out, len(pts)))


if __name__ == '__main__':
    main()
