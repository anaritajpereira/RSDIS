"""Analisa os CSV do registo_metricas e escreve um resumo da missão."""
import argparse
import csv
import glob
import math
import os
import re
import statistics

# os marcos da missão, pela ordem em que acontecem: (origem, estado, descrição)
MARCOS = [
    ('estado_missao', 'BUSCA', 'missão arrancou'),
    ('estado_drone', 'DESCOLAR', 'drone a descolar'),
    ('estado_drone', 'BUSCAR', 'drone a procurar'),
    ('estado_drone', 'ACOMPANHAR', 'alvo detetado'),
    ('estado_missao', 'RESGATE', 'pedido de resgate'),
    ('estado_robo', 'ACOMPANHAR_ALVO', 'robô confirmou o alvo'),
    ('estado_missao', 'HAND_OFF', 'hand-off'),
    ('estado_robo', 'REBOCAR', 'alvo preso (início do reboque)'),
    ('estado_robo', 'ENTREGUE', 'alvo entregue'),
    ('estado_drone', 'POUSADO', 'drone em terra'),
    ('estado_missao', 'CONCLUIDA', 'missão concluída'),
    ('estado_missao', 'FALHOU', 'missão falhou'),
]


def numero(texto):
    """Número a partir do texto do CSV (campo vazio vira nan)."""
    try:
        return float(texto)
    except (TypeError, ValueError):
        return math.nan


def validos(valores):
    return [v for v in valores if not math.isnan(v)]


def resumo(valores, unidade='m'):
    """Média, mediana e máximo de uma lista, em texto."""
    valores = validos(valores)
    if not valores:
        return 'sem dados'
    return (f'média {statistics.mean(valores):.2f} {unidade}, '
            f'mediana {statistics.median(valores):.2f} {unidade}, '
            f'máx {max(valores):.2f} {unidade} ({len(valores)} amostras)')


def coluna(linha, nome, antigo=None):
    """Lê uma coluna, aceitando o nome antigo (CSV de antes da mudança de nomes)."""
    if nome in linha:
        return numero(linha[nome])
    return numero(linha.get(antigo)) if antigo else math.nan


def declive(tempos, valores):
    """Declive da reta de mínimos quadrados (velocidade média)."""
    pares = [(t, v) for t, v in zip(tempos, valores) if not math.isnan(v)]
    if len(pares) < 2:
        return math.nan
    media_t = statistics.mean(t for t, _ in pares)
    media_v = statistics.mean(v for _, v in pares)
    variancia = sum((t - media_t) ** 2 for t, _ in pares)
    if variancia == 0.0:
        return math.nan
    return sum((t - media_t) * (v - media_v) for t, v in pares) / variancia


def zona_entrega(prefixo):
    """Zona de entrega lida do yaml guardado junto aos CSV (ou a de omissão)."""
    zona = {'zona_entrega_x': 15.0, 'zona_entrega_y': 10.0}
    try:
        texto = open(prefixo + '_parametros.yaml').read()
    except OSError:
        return zona['zona_entrega_x'], zona['zona_entrega_y']
    for chave in zona:
        encontrado = re.search(rf'{chave}:\s*(-?[\d.]+)', texto)
        if encontrado:
            zona[chave] = float(encontrado.group(1))
    return zona['zona_entrega_x'], zona['zona_entrega_y']


def analisar(ficheiro_metricas):
    """Devolve (texto do resumo, dicionário com os números principais)."""
    prefixo = ficheiro_metricas[:-len('_metricas.csv')]
    linhas = list(csv.DictReader(open(ficheiro_metricas)))
    eventos = []
    if os.path.isfile(prefixo + '_eventos.csv'):
        eventos = list(csv.DictReader(open(prefixo + '_eventos.csv')))
    # o CSV antigo dizia estado_barco em vez de estado_robo
    for evento in eventos:
        evento['origem'] = evento['origem'].replace('estado_barco', 'estado_robo')
    modo = os.path.basename(prefixo).split('_')[-1]
    zona_x, zona_y = zona_entrega(prefixo)

    # primeira vez que cada estado aparece (o POUSADO do drone é o do fim:
    # ele também começa pousado)
    instantes = {}
    for evento in eventos:
        chave = (evento['origem'], evento['estado'])
        if chave == ('estado_drone', 'POUSADO'):
            instantes[chave] = numero(evento['tempo'])
        else:
            instantes.setdefault(chave, numero(evento['tempo']))
    if instantes.get(('estado_drone', 'POUSADO')) == 0.0:
        del instantes[('estado_drone', 'POUSADO')]

    texto = [f'Análise da missão: {os.path.basename(prefixo)}',
             f'Modo de interceção: {modo}', '', 'Linha do tempo (tempo simulado)']
    marcos = sorted((instantes[(origem, estado)], descricao)
                    for origem, estado, descricao in MARCOS if (origem, estado) in instantes)
    for instante, descricao in marcos:
        texto.append(f'  {instante:7.1f} s  {descricao}')

    def entre(a, b):
        if a in instantes and b in instantes:
            return instantes[b] - instantes[a]
        return math.nan

    duracoes = [
        ('procura (descolar até detetar)', entre(('estado_drone', 'DESCOLAR'),
                                                 ('estado_drone', 'ACOMPANHAR'))),
        ('resgate (pedido até confirmar)', entre(('estado_missao', 'RESGATE'),
                                                 ('estado_robo', 'ACOMPANHAR_ALVO'))),
        ('engate', entre(('estado_robo', 'ENGATAR'), ('estado_robo', 'REBOCAR'))),
        ('reboque', entre(('estado_robo', 'REBOCAR'), ('estado_robo', 'ENTREGUE'))),
        ('missão completa', entre(('estado_missao', 'BUSCA'),
                                  ('estado_missao', 'CONCLUIDA'))),
    ]
    texto += ['', 'Durações']
    for nome, valor in duracoes:
        if not math.isnan(valor):
            texto.append(f'  {nome}: {valor:.1f} s')

    recuperacoes = sum(1 for e in eventos
                       if e['origem'] == 'estado_drone' and e['estado'] == 'RECUPERAR')
    final = eventos[-1]['estado'] if eventos else '?'
    estados_missao = [e['estado'] for e in eventos if e['origem'] == 'estado_missao']
    resultado = estados_missao[-1] if estados_missao else final

    # erros e distâncias a partir das linhas por segundo
    tempos = [numero(linha['tempo']) for linha in linhas]
    erro_detecao = [coluna(linha, 'erro_detecao') for linha in linhas]
    erro_estimativa = [coluna(linha, 'erro_estimativa') for linha in linhas]
    robo_alvo = [coluna(linha, 'distancia_robo_alvo', 'distancia_barco_alvo')
                 for linha in linhas]
    vx_estimado = validos([coluna(linha, 'estimativa_vx') for linha in linhas])
    vy_estimado = validos([coluna(linha, 'estimativa_vy') for linha in linhas])
    # a deriva real só se mede antes do engate: depois é o robô que mexe o alvo
    fim_deriva = instantes.get(('estado_robo', 'ENGATAR'), math.inf)
    antes = [(t, linha) for t, linha in zip(tempos, linhas) if t < fim_deriva]
    vx_real = declive([t for t, _ in antes],
                      [coluna(linha, 'alvo_real_x') for _, linha in antes])
    vy_real = declive([t for t, _ in antes],
                      [coluna(linha, 'alvo_real_y') for _, linha in antes])

    texto += ['', 'Perceção',
              f'  erro da deteção:    {resumo(erro_detecao)}',
              f'  erro da estimativa: {resumo(erro_estimativa)}']
    if vx_estimado:
        texto.append(f'  deriva estimada (última): ({vx_estimado[-1]:.2f}, '
                     f'{vy_estimado[-1]:.2f}) m/s')
    if not math.isnan(vx_real):
        texto.append(f'  deriva real (antes do engate): ({vx_real:.2f}, {vy_real:.2f}) m/s')
    texto.append(f'  vezes que o drone perdeu o alvo (RECUPERAR): {recuperacoes}')

    texto += ['', 'Robô marítimo']
    if validos(robo_alvo):
        texto.append(f'  distância mínima ao alvo: {min(validos(robo_alvo)):.1f} m')
    entregue = [linha for linha in linhas
                if linha.get('estado_robo', linha.get('estado_barco')) == 'ENTREGUE']
    if entregue:
        robo_zona = [math.hypot(coluna(linha, 'robo_x', 'barco_x') - zona_x,
                                coluna(linha, 'robo_y', 'barco_y') - zona_y)
                     for linha in entregue]
        alvo_zona = [coluna(linha, 'distancia_alvo_entrega') for linha in entregue]
        texto += [f'  a segurar o alvo durante {len(entregue)} s',
                  f'  robô à zona de entrega: {resumo(robo_zona)}',
                  f'  alvo à zona de entrega: {resumo(alvo_zona)}']
    else:
        texto.append('  não chegou a entregar o alvo')

    bateria = validos([coluna(linha, 'bateria') for linha in linhas])
    if bateria:
        texto += ['', f'Bateria do drone: {bateria[0]:.0%} no início, '
                      f'{bateria[-1]:.0%} no fim']
    texto += ['', f'Resultado: {resultado}']

    numeros = {
        'execucao': os.path.basename(prefixo), 'modo': modo, 'resultado': resultado,
        'detecao': instantes.get(('estado_drone', 'ACOMPANHAR'), math.nan),
        'confirmado': instantes.get(('estado_robo', 'ACOMPANHAR_ALVO'), math.nan),
        'entregue': instantes.get(('estado_robo', 'ENTREGUE'), math.nan),
        'concluida': instantes.get(('estado_missao', 'CONCLUIDA'), math.nan),
        'erro_detecao': statistics.mean(validos(erro_detecao) or [math.nan]),
        'erro_estimativa': statistics.mean(validos(erro_estimativa) or [math.nan]),
    }
    return '\n'.join(texto), numeros


def tabela(todos):
    """Uma linha por execução, para comparar várias (por exemplo os dois modos)."""
    cabecalho = (f'{"execução":<28} {"modo":<11} {"deteção":>8} {"confirm.":>9} '
                 f'{"entregue":>9} {"concluída":>10} {"erro det":>9} {"erro est":>9} '
                 'resultado')
    linhas = [cabecalho, '-' * len(cabecalho)]
    for n in todos:
        linhas.append(
            f'{n["execucao"]:<28.28} {n["modo"]:<11} {n["detecao"]:>7.1f}s '
            f'{n["confirmado"]:>8.1f}s {n["entregue"]:>8.1f}s {n["concluida"]:>9.1f}s '
            f'{n["erro_detecao"]:>8.2f}m {n["erro_estimativa"]:>8.2f}m {n["resultado"]}')
    return '\n'.join(linhas).replace('nans', '   -').replace('nanm', '   -')


def main():
    parser = argparse.ArgumentParser(
        description='Resume os CSV do registo_metricas (por omissão, a última execução).')
    parser.add_argument('caminho', nargs='?', default='metricas',
                        help='pasta com os CSV ou um ficheiro *_metricas.csv')
    parser.add_argument('--todas', action='store_true',
                        help='compara todas as execuções da pasta numa tabela')
    argumentos = parser.parse_args()

    if os.path.isfile(argumentos.caminho):
        ficheiros = [argumentos.caminho]
    else:
        ficheiros = sorted(glob.glob(os.path.join(argumentos.caminho, '*_metricas.csv')))
    if not ficheiros:
        print(f'Não encontrei *_metricas.csv em {argumentos.caminho}')
        return

    if argumentos.todas:
        texto = tabela([analisar(f)[1] for f in ficheiros])
        destino = os.path.join(os.path.dirname(ficheiros[-1]), 'comparacao.txt')
    else:
        # a mais recente (o nome começa pela data)
        texto = analisar(ficheiros[-1])[0]
        destino = ficheiros[-1][:-len('_metricas.csv')] + '_analise.txt'
    print(texto)
    with open(destino, 'w') as saida:
        saida.write(texto + '\n')
    print(f'\n(guardado em {destino})')


if __name__ == '__main__':
    main()
