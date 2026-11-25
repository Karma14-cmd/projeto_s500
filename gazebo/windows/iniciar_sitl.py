"""Inicia o S500 dinâmico, ponte JSON e ArduCopter no ambiente Pixi Windows."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import psutil

RAIZ = Path(__file__).resolve().parents[2]
PASTA_SCRIPT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--controle', action='store_true', help='Abrir joystick gráfico de teclado; decola a 2 m')
    parser.add_argument('--painel-sensores', action='store_true', help='Abrir painel Pygame de leitura')
    parser.add_argument('--conectar', action='store_true', help='Usar os processos já iniciados')
    parser.add_argument('--parar', action='store_true', help='Encerrar somente os processos registrados desta sessão')
    parser.add_argument('--runtime', type=Path, default=Path.home()/'gz-s500')
    parser.add_argument('--pasta-sessao', type=Path, help='Diretório separado para processos e logs')
    parser.add_argument('--particao', default='s500_voo_local')
    parser.add_argument('--camera-gui', help='Pose inicial da câmera: x y z roll pitch yaw')
    parser.add_argument('--mundo', type=Path, help='SDF de voo gerado para um cenário específico')
    args = parser.parse_args()
    runtime = args.runtime
    pasta = args.pasta_sessao.resolve() if args.pasta_sessao else runtime/'voo'
    pasta.mkdir(exist_ok=True, parents=True)
    binario = runtime/'ardupilot/arducopter.exe'
    if not binario.is_file() or not shutil.which('gz'):
        parser.error('Binários SITL ou ambiente Gazebo ausentes; consulte README_voo.md')
    registro = pasta/'processos.json'
    processos = {}
    if registro.exists():
        processos = json.loads(registro.read_text())
    # Formato antigo de registros é apenas lido; novos registros incluem o
    # instante de criação para nunca confundir um PID reaproveitado pelo Windows.
    ativos = {}
    for nome, info in processos.items():
        try:
            pid = info if isinstance(info,int) else info['pid']
            p = psutil.Process(pid)
            if isinstance(info,dict) and abs(p.create_time()-info['criado'])>.01:
                continue
            cmd = ' '.join(p.cmdline())
            if not any(m in cmd for m in (str(pasta),str(RAIZ),'gz sim -g')):
                continue
            ativos[nome] = {'pid':pid,'criado':p.create_time()}
        except psutil.Error:
            continue
    if args.parar:
        for info in ativos.values():
            try:
                p=psutil.Process(info['pid'])
                for filho in p.children(recursive=True):
                    filho.terminate()
                p.terminate()
            except psutil.Error:
                pass
        registro.write_text('{}')
        print('Sessão de voo encerrada.')
        return
    if args.conectar:
        if not all(n in ativos for n in ('gazebo','ponte','sitl')):
            parser.error('Não há sessão completa ativa.')
    elif ativos:
        parser.error('Já há sessão ativa. Use --conectar ou --parar.')
    mundo = args.mundo.resolve() if args.mundo else pasta/'s500_voo.sdf'
    if not args.conectar:
        if args.mundo and not mundo.is_file():
            parser.error(f'SDF não encontrado: {mundo}')
        if not args.mundo:
            spec = importlib.util.spec_from_file_location('mundo_voo', PASTA_SCRIPT/'gerar_mundo_voo.py')
            m = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(m)
            m.gerar(mundo)
    env = os.environ.copy()
    env['GZ_PARTITION'] = args.particao
    env['GZ_SIM_RESOURCE_PATH'] = (RAIZ/'gazebo/modelos').as_posix()
    env['QT_OPENGL'] = 'desktop'
    env['QT_OPENGL_DLL'] = str(Path(env['CONDA_PREFIX'])/'Library/bin/opengl32.dll')
    env['GALLIUM_DRIVER'] = 'llvmpipe'
    env['QSG_RENDER_LOOP'] = 'basic'
    gz = [shutil.which('ruby'), str(Path(shutil.which('gz')).with_suffix(''))]
    processos = ativos

    def iniciar(nome, comando):
        if nome in ativos:
            return
        flags = subprocess.CREATE_NEW_CONSOLE if nome=='controle' else subprocess.CREATE_NO_WINDOW
        with (pasta/f'{nome}.log').open('w') as log:
            p = subprocess.Popen(comando,cwd=pasta,env=env,stdout=log,stderr=subprocess.STDOUT,
                                 creationflags=flags)
        if p.poll() is not None:
            parser.error(f'{nome} encerrou ao iniciar. Consulte {pasta}/{nome}.log')
        processos[nome] = {'pid':p.pid,'criado':psutil.Process(p.pid).create_time()}
        registro.write_text(json.dumps(processos,indent=2))

    if not args.conectar:
        iniciar('gazebo',gz+['sim','-r','-s','-v','4',str(mundo)])
        iniciar('ponte',[sys.executable,str(RAIZ/'python/comunicacao/ponte_gazebo_json.py')])
        iniciar('sitl',[str(binario),'--model','JSON','--rate','100','--speedup','1',
                    '--home','-5.36,-49.12,100,0','--defaults',str(PASTA_SCRIPT/'s500_sitl.parm'),
                    '--serial0','udpclient:127.0.0.1:14550','--serial1','tcp:1'])
    if args.gui:
        config = Path.home()/'.gz/sim/8/gui.config'
        argumentos = []
        if config.exists():
            import re
            pose = args.camera_gui or '3 -3 2.8 0 0.3 2.35619449'
            texto = re.sub(r'<camera_pose>.*?</camera_pose>',
                           f'<camera_pose>{pose}</camera_pose>',config.read_text(encoding='utf-8'))
            texto=texto.replace('show_again="true"','show_again="false"')
            copia=pasta/'gui_voo.config'
            copia.write_text(texto, encoding='utf-8')
            argumentos=['--gui-config',str(copia)]
        iniciar('gui',gz+['sim','-g','--render-engine-gui','ogre']+argumentos)
    if args.controle:
        iniciar('controle',[sys.executable,'-u',str(RAIZ/'python/controle/joystick_gui.py')])
    if args.painel_sensores:
        iniciar('painel',[sys.executable,'-B','-u',str(RAIZ/'python/controle/painel_sensores.py')])
    print('Sessão SITL iniciada. Logs:',pasta)
    print('MAVLink: saída UDP 127.0.0.1:14550; TCP secundário 127.0.0.1:5761.')


if __name__ == '__main__':
    main()
