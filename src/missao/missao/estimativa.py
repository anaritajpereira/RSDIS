"""Ajuste da deriva do alvo por regressão linear (sem ROS)."""
import numpy as np


def ajustar_reta(tempos, valores):
    """Devolve (valor em t=0, declive) da reta que melhor se ajusta aos pontos."""
    declive, origem = np.polyfit(tempos, valores, 1)
    return float(origem), float(declive)


def ajustar_deriva(amostras, limite_residuo, pontos_min):
    """
    Ajusta x(t) e y(t) a retas e devolve (x0, y0, vx, vy, n_usados).

    amostras é uma lista de (t, x, y). Faz duas passagens: ajusta com tudo,
    tira os pontos que ficam a mais de limite_residuo da reta e volta a ajustar.
    Devolve None se não houver pontos suficientes.
    """
    if len(amostras) < pontos_min:
        return None
    dados = np.asarray(amostras, dtype=float)
    # tempos relativos ao primeiro ponto para as contas serem mais estáveis
    t0 = dados[0, 0]
    tempos = dados[:, 0] - t0

    x0, vx = ajustar_reta(tempos, dados[:, 1])
    y0, vy = ajustar_reta(tempos, dados[:, 2])
    residuos = np.hypot(dados[:, 1] - (x0 + vx * tempos),
                        dados[:, 2] - (y0 + vy * tempos))
    bons = residuos <= limite_residuo
    usados = len(amostras)
    if bons.sum() >= pontos_min and not bons.all():
        x0, vx = ajustar_reta(tempos[bons], dados[bons, 1])
        y0, vy = ajustar_reta(tempos[bons], dados[bons, 2])
        usados = int(bons.sum())
    # devolve a origem em tempo absoluto: x(t) = x0 + vx * t
    return x0 - vx * t0, y0 - vy * t0, vx, vy, usados
