"""Gera a figura do reboque a partir do CSV do registo_metricas."""
import csv
import math
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

pasta = sys.argv[1]
linhas = list(csv.DictReader(open(f'{pasta}/reboque_metricas.csv')))
eventos = list(csv.DictReader(open(f'{pasta}/reboque_eventos.csv')))
ZONA = (15.0, 10.0)


def numero(texto):
    return float(texto) if texto else math.nan


tempo, robo_zona, alvo_zona, robo_alvo = [], [], [], []
for linha in linhas:
    t = float(linha['tempo'])
    if t < 50 or t > 450:
        continue
    bx, by = numero(linha['robo_x' if 'robo_x' in linha else 'barco_x']), numero(linha['robo_y' if 'robo_y' in linha else 'barco_y'])
    tempo.append(t)
    robo_zona.append(math.hypot(bx - ZONA[0], by - ZONA[1]))
    alvo_zona.append(numero(linha['distancia_alvo_entrega']))
    robo_alvo.append(numero(linha.get('distancia_robo_alvo', linha.get('distancia_robo_alvo'))))

inicio = {e['estado']: float(e['tempo']) for e in eventos if e['origem'] in ('estado_robo', 'estado_barco')}

plt.rcParams.update({'font.family': 'Liberation Serif', 'font.size': 9})
fig, eixo = plt.subplots(figsize=(7.0, 2.6), dpi=200)
fig.patch.set_facecolor('#fcfcfb')
eixo.set_facecolor('#fcfcfb')

# fases do barco como faixas de fundo, com o nome escrito por cima
fases = [('INTERCETAR', 'APROXIMAR'), ('ENGATAR', 'REBOCAR'), ('REBOCAR', 'ENTREGUE'),
         ('ENTREGUE', None)]
nomes = {'INTERCETAR': 'resgate', 'ENGATAR': 'engatar', 'REBOCAR': 'rebocar',
         'ENTREGUE': 'entregue (a segurar)'}
for i, (de, ate) in enumerate(fases):
    x0 = inicio[de]
    x1 = inicio[ate] if ate else tempo[-1]
    if de == 'INTERCETAR':
        x1 = inicio['ENGATAR']
    eixo.axvspan(x0, x1, color='#52514e', alpha=0.06 if i % 2 == 0 else 0.12, lw=0)
    # as faixas curtas ficam lado a lado, por isso as etiquetas alternam de altura
    eixo.text((x0 + x1) / 2, 76 if i % 2 == 0 else 70, nomes[de], ha='center', va='top',
              fontsize=8, color='#52514e')

series = [(robo_zona, 'robô – zona de entrega', '#2a78d6', '-'),
          (alvo_zona, 'alvo – zona de entrega (real)', '#eb6834', '--'),
          (robo_alvo, 'robô – alvo (real)', '#1baf7a', ':')]
for valores, nome, cor, estilo in series:
    eixo.plot(tempo, valores, color=cor, lw=1.8, ls=estilo, label=nome)

eixo.axhline(8, color='#52514e', lw=0.8, ls=(0, (2, 2)))
eixo.text(tempo[-1], 8.5, 'raio de entrega (8 m)', ha='right', va='bottom', fontsize=7.5,
          color='#52514e')
eixo.set_xlim(tempo[0], tempo[-1])
eixo.set_ylim(0, 78)
eixo.set_xlabel('tempo de simulação (s)')
eixo.set_ylabel('distância (m)')
for lado in ('top', 'right'):
    eixo.spines[lado].set_visible(False)
for lado in ('left', 'bottom'):
    eixo.spines[lado].set_color('#8a8984')
eixo.tick_params(colors='#52514e')
eixo.grid(axis='y', color='#d9d8d4', lw=0.6)
eixo.set_axisbelow(True)
eixo.legend(loc='upper right', bbox_to_anchor=(1.0, 0.86), frameon=False, fontsize=8)
fig.tight_layout()
fig.savefig(f'{pasta}/fig_reboque.png', facecolor=fig.get_facecolor())
print('ok', len(tempo), inicio)
