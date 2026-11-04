#!/usr/bin/env python3
"""Genera el mapa de ocupacion 2D (ROS map .yaml + .pgm) de una planta del parking a partir de un PCD.

MVSim es un simulador con fisica 2D: las colisiones del Caddy con el parking salen de este mapa.
Una celda es ocupada si algun punto del PCD cae dentro de ella en la banda de alturas dada
(la banda excluye el suelo y el techo, asi quedan paredes, pilares y coches).

Uso:
    make_garage_grid.py nube.pcd --out maps/parking_garage_ground --zmin 0.1 --zmax 1.5

Las alturas son las del PCD. En los PCD generados con navmap_map_builder, z=0 es el suelo
de la planta donde arranco el robot (el suelo queda en torno a z=-0.2 y el techo en z=1.8-2.0).
"""
import argparse
import io
import os

import numpy as np
from PIL import Image


def load_pcd(path):
    with open(path, "rb") as f:
        while True:
            line = f.readline().decode()
            if line.startswith("DATA"):
                kind = line.split()[1]
                break
        if kind == "ascii":
            return np.loadtxt(io.BytesIO(f.read()))[:, :3]
        dt = np.dtype([(n, "<f4") for n in "xyz"])
        d = np.frombuffer(f.read(), dtype=dt)
        return np.stack([d["x"], d["y"], d["z"]], 1).astype(np.float64)


def remove_small_components(occ, min_cells):
    """Quita los grupos conexos (8 vecinos) de menos de min_cells celdas ocupadas."""
    h, w = occ.shape
    seen = np.zeros_like(occ, bool)
    out = occ.copy()
    removed = 0
    for sy, sx in zip(*np.nonzero(occ)):
        if seen[sy, sx]:
            continue
        stack, comp = [(sy, sx)], []
        seen[sy, sx] = True
        while stack:
            y, x = stack.pop()
            comp.append((y, x))
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < h and 0 <= nx < w and occ[ny, nx] and not seen[ny, nx]:
                        seen[ny, nx] = True
                        stack.append((ny, nx))
        if len(comp) < min_cells:
            for y, x in comp:
                out[y, x] = False
            removed += len(comp)
    return out, removed


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pcd")
    ap.add_argument("--out", required=True, help="prefijo de salida (escribe <out>.pgm y <out>.yaml)")
    ap.add_argument("--resolution", type=float, default=0.1)
    ap.add_argument("--zmin", type=float, default=0.1)
    ap.add_argument("--zmax", type=float, default=1.5)
    ap.add_argument("--margin", type=float, default=1.0, help="margen libre alrededor (m)")
    ap.add_argument("--min-component", type=int, default=12,
                    help="descartar grupos de menos celdas ocupadas (ruido); 1 = no filtrar")
    args = ap.parse_args()

    pts = load_pcd(args.pcd)
    pts = pts[np.isfinite(pts).all(1)]
    sel = (pts[:, 2] >= args.zmin) & (pts[:, 2] <= args.zmax)
    obst = pts[sel]
    if len(obst) == 0:
        raise SystemExit("ningun punto en la banda z=[%g, %g]" % (args.zmin, args.zmax))

    r = args.resolution
    # el recuadro lo marca TODA la nube de la planta (no solo los obstaculos)
    xmin, ymin = pts[:, 0].min() - args.margin, pts[:, 1].min() - args.margin
    xmax, ymax = pts[:, 0].max() + args.margin, pts[:, 1].max() + args.margin
    w = int(np.ceil((xmax - xmin) / r))
    h = int(np.ceil((ymax - ymin) / r))

    ix = np.clip(((obst[:, 0] - xmin) / r).astype(int), 0, w - 1)
    iy = np.clip(((obst[:, 1] - ymin) / r).astype(int), 0, h - 1)
    occ = np.zeros((h, w), bool)
    occ[iy, ix] = True

    # Quitar ruido: grupos pequenos de celdas ocupadas. Con navmap_map_builder el Helios ve el costado
    # del propio robot en cada posicion visitada (celdas sueltas cada ~1 m); las paredes y pilares
    # reales forman grupos grandes y se conservan.
    removed = 0
    if args.min_component > 1:
        occ, removed = remove_small_components(occ, args.min_component)

    # fila 0 de la imagen = y maximo (convencion de ROS map_server)
    img = np.where(occ[::-1], 0, 254).astype(np.uint8)
    out = args.out
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    Image.fromarray(img, mode="L").save(out + ".pgm")
    with open(out + ".yaml", "w") as f:
        f.write("image: %s.pgm\n" % os.path.basename(out))
        f.write("mode: trinary\n")
        f.write("resolution: %g\n" % r)
        f.write("origin: [%.3f, %.3f, 0.0]\n" % (xmin, ymin))
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    print("mapa %dx%d celdas de %g m | %d ocupadas (%.1f%%), %d descartadas como ruido | origen (%.2f, %.2f) | puntos en banda: %d"
          % (w, h, r, occ.sum(), 100 * occ.mean(), removed, xmin, ymin, len(obst)))


if __name__ == "__main__":
    main()
