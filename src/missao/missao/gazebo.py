"""Chamadas ao Gazebo pelo "ign" da linha de comandos (sem ROS)."""
import math
import subprocess

# O DetachableJoint cria uma junta fixa entre um link deste modelo e um link de
# outro modelo, na posição relativa em que estão nesse momento. É o mesmo plugin
# que o mundo já usa para prender o painel vermelho ao barco à deriva. Aqui usamo-lo
# nas pontas de uma barra de reboque que criamos com a simulação a correr.
PLUGIN_JUNTA = 'ignition::gazebo::systems::DetachableJoint'
BIBLIOTECA_JUNTA = 'ignition-gazebo-detachable-joint-system'


def correr(comando, tempo_max=10.0):
    """Corre um comando e devolve o texto que escreveu (ou '' se falhar)."""
    try:
        resultado = subprocess.run(comando, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT,
                                   universal_newlines=True, timeout=tempo_max)
    except (OSError, subprocess.TimeoutExpired):
        return ''
    return resultado.stdout


def soltar(topico_soltar):
    """Pede ao DetachableJoint para desfazer a junta."""
    correr(['ign', 'topic', '-t', topico_soltar, '-m', 'ignition.msgs.Empty',
            '-p', 'unused: true'])


def sdf_barra_reboque(comprimento, robo, link_robo, alvo, link_alvo, topico_soltar,
                      massa=10.0):
    """
    SDF da barra de reboque: uma barra com uma articulação em cada ponta.

    A barra fica ao longo do x do modelo, com a ponta do robô em +x e a ponta
    do alvo em -x. Os dois DetachableJoint prendem as pontas ao robô e ao alvo
    quando o modelo aparece, e soltam-se ao mesmo tempo pelo mesmo tópico.
    """
    meio = comprimento / 2.0
    # as pontas são só pontos de articulação; pequenas mas não minúsculas,
    # senão a física fica instável ao lado de barcos de centenas de kg
    inercia_ponta = ("<inertial><mass>{m}</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy>"
                     "<izz>0.1</izz></inertia></inertial>").format(m=massa)
    inercia_barra = massa * comprimento ** 2 / 12.0

    def prender(ponta, modelo, link):
        return (f"<plugin filename='{BIBLIOTECA_JUNTA}' name='{PLUGIN_JUNTA}'>"
                f"<parent_link>{ponta}</parent_link><child_model>{modelo}</child_model>"
                f"<child_link>{link}</child_link><topic>{topico_soltar}</topic></plugin>")

    def articulacao(nome, pai, filho):
        # roda só à volta do eixo vertical, como um cabo preso num gancho
        return (f"<joint name='{nome}' type='revolute'><parent>{pai}</parent>"
                f"<child>{filho}</child><axis><xyz>0 0 1</xyz></axis></joint>")

    return (
        "<sdf version='1.6'><model name='barra_reboque'>"
        f"<link name='ponta_robo'><pose>{meio} 0 0 0 0 0</pose>"
        f"<gravity>false</gravity>{inercia_ponta}</link>"
        f"<link name='ponta_alvo'><pose>{-meio} 0 0 0 0 0</pose>"
        f"<gravity>false</gravity>{inercia_ponta}</link>"
        "<link name='barra'><gravity>false</gravity>"
        f"<inertial><mass>{massa}</mass><inertia><ixx>0.1</ixx>"
        f"<iyy>{inercia_barra}</iyy><izz>{inercia_barra}</izz></inertia></inertial>"
        # só visual (sem colisão), amarelo para se ver bem na janela do Gazebo
        f"<visual name='visual'><pose>0 0 0 0 1.5708 0</pose><geometry><cylinder>"
        f"<radius>0.15</radius><length>{comprimento}</length></cylinder></geometry>"
        "<material><ambient>1 0.8 0 1</ambient><diffuse>1 0.8 0 1</diffuse></material>"
        "</visual></link>"
        # a pose da junta é no referencial do filho: estas ficam nas pontas
        + articulacao('articulacao_robo', 'ponta_robo', 'barra').replace(
            '<axis>', f'<pose>{meio} 0 0 0 0 0</pose><axis>')
        + articulacao('articulacao_alvo', 'barra', 'ponta_alvo')
        + prender('ponta_robo', robo, link_robo)
        + prender('ponta_alvo', alvo, link_alvo)
        + "</model></sdf>")


def criar_modelo(mundo, sdf, nome, x, y, z, rumo):
    """Cria um modelo no mundo com a simulação a correr; devolve True se correu bem."""
    # quaternião só com rotação em z
    meio_rumo = rumo / 2.0
    pedido = (f'sdf: "{sdf}" name: "{nome}" allow_renaming: false '
              f'pose: {{position: {{x: {x}, y: {y}, z: {z}}} '
              f'orientation: {{z: {math.sin(meio_rumo)}, w: {math.cos(meio_rumo)}}}}}')
    resposta = correr([
        'ign', 'service', '-s', f'/world/{mundo}/create',
        '--reqtype', 'ignition.msgs.EntityFactory',
        '--reptype', 'ignition.msgs.Boolean', '--timeout', '5000', '--req', pedido])
    return 'data: true' in resposta


def apagar_modelo(mundo, nome):
    """Tira um modelo do mundo."""
    correr(['ign', 'service', '-s', f'/world/{mundo}/remove',
            '--reqtype', 'ignition.msgs.Entity', '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '5000', '--req', f'name: "{nome}" type: MODEL'])
