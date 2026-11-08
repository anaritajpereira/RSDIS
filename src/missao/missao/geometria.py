"""Funções geométricas puras (sem ROS), partilhadas pelos nós."""
import math

import numpy as np


def normalizar_angulo(a):
    """Devolve o ângulo equivalente em [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_de_quaternion(x, y, z, w):
    """Devolve o rumo (rotação em z) de um quaternião."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def mundo_para_corpo(x, y, rumo):
    """Roda um vetor horizontal do referencial do mundo para o do corpo."""
    cos_rumo, sin_rumo = math.cos(rumo), math.sin(rumo)
    return cos_rumo * x + sin_rumo * y, -sin_rumo * x + cos_rumo * y


def saturar(v, limite):
    """Limita v a [-limite, limite]."""
    return max(-limite, min(limite, v))


def saturar_vetor(x, y, limite):
    """Limita a norma de (x, y) sem mudar a direção."""
    norma = math.hypot(x, y)
    if norma <= limite or norma == 0.0:
        return x, y
    escala = limite / norma
    return x * escala, y * escala


def velocidade_desejada(x, y, alvo_x, alvo_y, ganho, vel_max):
    """Velocidade em direção ao alvo, proporcional à distância e limitada."""
    return saturar_vetor(ganho * (alvo_x - x), ganho * (alvo_y - y), vel_max)


def comando_vetorial(rumo, vetor_x, vetor_y, ganho_rumo, vel_max, rot_max,
                     erro_max, minimo):
    """
    Aponta a proa de um veículo diferencial para o vetor dado e avança.

    Devolve (avanço, rotação). Acima de erro_max só roda; com a norma do vetor
    abaixo de minimo não comanda nada, para não reagir ao ruído da velocidade medida.
    """
    norma = math.hypot(vetor_x, vetor_y)
    if norma < minimo:
        return 0.0, 0.0
    erro_rumo = normalizar_angulo(math.atan2(vetor_y, vetor_x) - rumo)
    # se estiver muito desalinhado primeiro roda no sítio e só depois anda
    rotacao = saturar(ganho_rumo * erro_rumo, rot_max)
    avanco = 0.0
    if abs(erro_rumo) < erro_max:
        avanco = min(vel_max, norma * math.cos(erro_rumo))
    return avanco, rotacao


def matriz_de_quaternion(x, y, z, w):
    """Matriz de rotação 3x3 de um quaternião unitário."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matriz_de_rpy(roll, pitch, yaw):
    """Matriz de rotação 3x3 com a convenção do SDF (Rz * Ry * Rx)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def raio_do_pixel(u, v, fx, fy, cx, cy):
    """
    Direção do raio de um píxel no referencial da câmara do Gazebo.

    Nesse referencial x aponta para a frente, y para a esquerda e z para cima;
    na imagem, u cresce para a direita e v para baixo.
    """
    # sinais trocados porque u cresce para a direita (y é para a esquerda)
    # e v cresce para baixo (z é para cima)
    return np.array([1.0, -(u - cx) / fx, -(v - cy) / fy])


def intersetar_plano(origem, direcao, altura):
    """Ponto onde o raio cruza o plano z = altura, ou None se não o cruzar à frente."""
    # raio na horizontal ou a subir nunca chega à água
    if direcao[2] >= -1e-9:
        return None
    t = (altura - origem[2]) / direcao[2]
    if t <= 0.0:
        return None
    return origem + t * direcao


def pixel_para_mundo(u, v, intrinsecos, posicao, rotacao, camera_xyz,
                     camera_rot, altura):
    """
    Projeta um píxel no plano z = altura do mundo.

    intrinsecos é (fx, fy, cx, cy); posicao e rotacao são a pose do veículo no
    mundo; camera_xyz e camera_rot são a pose da câmara no veículo.
    Devolve o ponto (x, y, z) ou None se o raio não chegar ao plano.
    """
    # a câmara não está no centro do drone, por isso a origem do raio leva o offset
    origem = np.asarray(posicao) + rotacao @ np.asarray(camera_xyz)
    direcao = rotacao @ camera_rot @ raio_do_pixel(u, v, *intrinsecos)
    return intersetar_plano(origem, direcao, altura)


def espacamento_faixas(altitude, fov_horizontal, depressao_min, sobreposicao):
    """
    Distância entre faixas de busca para uma câmara virada para a frente.

    A câmara usa a água até onde o raio desce depressao_min abaixo do horizonte,
    ou seja até altitude/tan(depressao_min); a essa distância vê uma largura de
    2*d*tan(fov/2). A sobreposição (0 a 1) reduz o espaçamento.
    """
    # com 20 m de altitude e 0.2 rad dá ~100 m de alcance e ~91 m entre faixas
    alcance = altitude / math.tan(depressao_min)
    return 2.0 * alcance * math.tan(fov_horizontal / 2.0) * (1.0 - sobreposicao)


def pontos_busca(x_min, x_max, y_min, y_max, espacamento):
    """Pontos de um varrimento em faixas paralelas a x, alternando o sentido."""
    n_faixas = max(1, math.ceil((y_max - y_min) / espacamento))
    largura_faixa = (y_max - y_min) / n_faixas
    pontos = []
    for i in range(n_faixas):
        y = y_min + (i + 0.5) * largura_faixa
        faixa = [(x_min, y), (x_max, y)]
        # vai e volta (tipo cortar a relva) para não perder tempo a voltar ao início
        pontos.extend(faixa if i % 2 == 0 else faixa[::-1])
    return pontos


def ponto_a_distancia(alvo_x, alvo_y, referencia_x, referencia_y, distancia):
    """Ponto a uma dada distância do alvo, do lado da referência."""
    falta_x, falta_y = referencia_x - alvo_x, referencia_y - alvo_y
    norma = math.hypot(falta_x, falta_y)
    # se a referência estiver em cima do alvo não há direção, escolhe-se +x
    if norma < 1e-6:
        return alvo_x + distancia, alvo_y
    return (alvo_x + distancia * falta_x / norma,
            alvo_y + distancia * falta_y / norma)


def bateria_necessaria(distancia_base, altura, vel_cruzeiro, vel_descida,
                       consumo_por_segundo):
    """Fração de bateria para voltar à base e aterrar, a consumo constante."""
    # conta só a linha reta até à base mais a descida; a margem cobre o resto
    tempo = distancia_base / vel_cruzeiro + max(0.0, altura) / vel_descida
    return tempo * consumo_por_segundo


def velocidade_com_avanco(x, y, alvo_x, alvo_y, vel_alvo_x, vel_alvo_y, ganho,
                          vel_max):
    """Como velocidade_desejada, mas somando a velocidade do alvo (para o acompanhar)."""
    # sem a parcela do alvo ficávamos sempre um bocado atrás de um alvo que anda
    return saturar_vetor(vel_alvo_x + ganho * (alvo_x - x),
                         vel_alvo_y + ganho * (alvo_y - y), vel_max)


def tempo_intercecao(alvo_x, alvo_y, vel_alvo_x, vel_alvo_y, x, y, velocidade):
    """
    Menor t > 0 com |p + v*t - b| = s*t, ou None se não houver.

    p e v são a posição e a velocidade do alvo, b a posição do robô e s a
    velocidade do robô.
    """
    # fica uma equação do 2º grau em t: a*t^2 + b*t + c = 0
    dx, dy = alvo_x - x, alvo_y - y
    a = vel_alvo_x ** 2 + vel_alvo_y ** 2 - velocidade ** 2
    b = 2.0 * (dx * vel_alvo_x + dy * vel_alvo_y)
    c = dx ** 2 + dy ** 2
    if abs(a) < 1e-9:
        # o alvo anda tão depressa como nós: a equação fica de 1º grau
        return -c / b if b < 0.0 else None
    discriminante = b * b - 4.0 * a * c
    if discriminante < 0.0:
        return None
    raiz = math.sqrt(discriminante)
    positivos = [t for t in ((-b - raiz) / (2 * a), (-b + raiz) / (2 * a)) if t > 0.0]
    return min(positivos) if positivos else None


def ponto_espiral(centro_x, centro_y, angulo, raio):
    """Ponto da espiral à volta do centro, no ângulo e raio dados."""
    return centro_x + raio * math.cos(angulo), centro_y + raio * math.sin(angulo)


def corpo_para_mundo(x, y, rumo):
    """Roda um vetor horizontal do referencial do corpo para o do mundo."""
    return mundo_para_corpo(x, y, -rumo)
