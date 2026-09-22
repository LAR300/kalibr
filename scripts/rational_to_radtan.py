#!/usr/bin/env python3
"""Ajusta um `radtan` de 4 parametros que REPRODUZ um modelo OpenCV `rational_polynomial`.

## Por que isto existe

O SDK da ZED publica a distorcao como `rational_polynomial` (8 coeficientes: k1,k2,p1,p2,k3,k4,k5,k6):

    x' = x * (1 + k1 r^2 + k2 r^4 + k3 r^6) / (1 + k4 r^2 + k5 r^4 + k6 r^6)  + tangencial

O Kalibr so' tem `radtan` (4 coeficientes), que e' o numerador truncado:

    x' = x * (1 + k1 r^2 + k2 r^4)  + tangencial

**Truncar e' catastrofico** neste caso. Medido na ZED 2i deste projeto (camera esquerda):

    r      racional   truncado    erro
    0.20     0.9962     1.1078      11%
    0.50     0.9797     3.0627     213%
    0.75     0.9448    10.2923     989%

O numerador e o denominador quase se cancelam, entao a distorcao REAL e' suave (±6%); jogar o
denominador fora faz o polinomio explodir. Reusar os 4 primeiros coeficientes como se fossem radtan
produz lixo silencioso.

Este script resolve: amostra o mapeamento racional sobre o FOV e ajusta (k1,k2,p1,p2) por minimos
quadrados para reproduzi-lo. O residuo e' reportado em pixels, para que se saiba o custo da
aproximacao.

Uso:
  python3 rational_to_radtan.py --fx 957.790 --fy 958.185 --cx 640.875 --cy 354.266 \
      --D 1.63581 26.4600 4.88327e-04 1.85865e-06 -11.7148 1.74078 26.3567 -9.16919 \
      --width 1280 --height 720
"""
from __future__ import print_function
import argparse

import numpy as np


def distort_rational(x, y, D):
    k1, k2, p1, p2, k3, k4, k5, k6 = D
    r2 = x * x + y * y
    r4, r6 = r2 * r2, r2 * r2 * r2
    radial = (1 + k1 * r2 + k2 * r4 + k3 * r6) / (1 + k4 * r2 + k5 * r4 + k6 * r6)
    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return xd, yd


def distort_radtan(x, y, k1, k2, p1, p2):
    r2 = x * x + y * y
    radial = 1 + k1 * r2 + k2 * r2 * r2
    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return xd, yd


def fit(fx, fy, cx, cy, D, w, h, n=120):
    """Ajusta radtan-4 ao mapeamento racional, amostrando o FOV real da imagem."""
    xs = (np.linspace(0, w - 1, n) - cx) / fx
    ys = (np.linspace(0, h - 1, n) - cy) / fy
    X, Y = np.meshgrid(xs, ys)
    x, y = X.ravel(), Y.ravel()
    xd, yd = distort_rational(x, y, D)

    # o modelo e' LINEAR nos coeficientes -> minimos quadrados direto, sem iteracao
    r2 = x * x + y * y
    r4 = r2 * r2
    # dx = xd - x = x*(k1 r2 + k2 r4) + 2 p1 x y + p2 (r2 + 2x^2)
    Ax = np.stack([x * r2, x * r4, 2 * x * y, r2 + 2 * x * x], axis=1)
    Ay = np.stack([y * r2, y * r4, r2 + 2 * y * y, 2 * x * y], axis=1)
    A = np.vstack([Ax, Ay])
    b = np.concatenate([xd - x, yd - y])
    coef, *_ = np.linalg.lstsq(A, b, rcond=None)
    k1, k2, p1, p2 = coef

    xf, yf = distort_radtan(x, y, k1, k2, p1, p2)
    err_px = np.hypot((xf - xd) * fx, (yf - yd) * fy)
    return coef, err_px


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fx", type=float, required=True)
    ap.add_argument("--fy", type=float, required=True)
    ap.add_argument("--cx", type=float, required=True)
    ap.add_argument("--cy", type=float, required=True)
    ap.add_argument("--D", type=float, nargs=8, required=True,
                    help="k1 k2 p1 p2 k3 k4 k5 k6 (ordem OpenCV rational)")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    a = ap.parse_args()

    coef, err = fit(a.fx, a.fy, a.cx, a.cy, a.D, a.width, a.height)
    trunc = np.array(a.D[:4])
    print("racional (8 coef)      : %s" % np.array2string(np.array(a.D), precision=6))
    print("truncado ingenuo (4)   : %s   <- NAO USAR" % np.array2string(trunc, precision=6))
    print("radtan AJUSTADO (4)    : [%.8f, %.8f, %.8f, %.8f]" % tuple(coef))
    print("residuo do ajuste [px] : media %.3f · mediana %.3f · p95 %.3f · max %.3f"
          % (err.mean(), np.median(err), np.percentile(err, 95), err.max()))
    print()
    print("distortion_coeffs: [%.8f, %.8f, %.8f, %.8f]" % tuple(coef))


if __name__ == "__main__":
    main()
