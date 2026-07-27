# AGENTS.md

> Briefing para agentes de IA. Mantenha enxuto — carrega em toda sessão. Detalhe em `.ai/docs/`.

## Visão geral do projeto

Kalibr: toolbox de **calibração câmera/IMU** (ETH Zurich) por otimização batch em tempo contínuo
(B-splines). Calibra multi-câmera, **câmera-IMU** (foco deste fork), IMU-IMU e rolling shutter.
Este checkout é um **fork** com a pasta `scripts/` adicionada para calibrar **câmera+IMU da ZED 2i** via Docker.

## Stack

Resumo aqui; detalhes e versões em `.ai/docs/stack.md`.

- Linguagem: C++14 + **Python 3** (lógica de calibração em Python, otimizador em C++)
- Build: workspace **catkin** (`catkin build`, não `catkin_make`) + CMake; **ROS 1** (Kinetic/Melodic/Noetic)
- Otimização: `aslam_backend` próprio (não Ceres/g2o); SuiteSparse para álgebra esparsa
- Sem gerenciador de pacotes pinado — dependências vêm do apt da distro (use Docker)

## Comandos canônicos (não adivinhe)

```bash
# Ambiente Docker (fork), na pasta scripts/
cd scripts && make build && make run          # imagem 'kalibr', container 'kalibr_zed', /data montado

# Build manual (dentro de um container ROS, no /catkin_ws)
catkin build -DCMAKE_BUILD_TYPE=Release

# Rodar uma calibração (dentro do container) — ver domain/cam-imu-calibration.md
rosrun kalibr kalibr_bagcreater --folder /data/.../<SESSAO> --output-bag /data/output/cam_imu.bag
rosrun kalibr kalibr_calibrate_imu_camera --bag ... --cams camchain.yaml --imu imu0.yaml --target target.yaml

# Testes (GTest de error terms)
catkin build kalibr --catkin-make-args run_tests
```

## Estrutura

- Pacote principal / CLIs: `aslam_offline_calibration/kalibr/python/kalibr_*`
- Error terms de IMU (C++): `aslam_offline_calibration/kalibr/src/`
- Otimizador: `aslam_optimizer/` · Splines: `aslam_nonparametric_estimation/` · Visão: `aslam_cv/` · Utilitários: `Schweizer-Messer/`
- Customização do fork: `scripts/` (Makefile Docker, `config/` de exemplo, `docs/kalibr.md`)

## Onde está o contexto detalhado

Leia sob demanda, conforme a tarefa:

- Visão geral: `.ai/docs/overview.md`
- Arquitetura (pacotes, pipeline B-spline, módulos reaproveitáveis): `.ai/docs/architecture.md`
- Stack e ambientes Docker: `.ai/docs/stack.md`
- Convenções (workspace catkin): `.ai/docs/conventions.md`
- Vocabulário: `.ai/docs/glossary.md`
- CLIs (entradas/saídas): `.ai/docs/api.md`
- Formatos de arquivo (YAMLs, CSV, resultados): `.ai/docs/data-model.md`
- Testes: `.ai/docs/testing.md`
- **Fluxo CAM-IMU + integração com AQUA-SLAM:** `.ai/docs/domain/cam-imu-calibration.md`
- **Calibração de DVL (`kalibr_calibrate_dvl`, extensão do fork):** `.ai/docs/domain/dvl-calibration.md`
  (guia de uso: `scripts/docs/kalibr_dvl.md`; spec: `.ai/specs/dvl-calibration/`)

## Regras inegociáveis

- Nunca commitar dados de calibração: `scripts/data/*` está no `.gitignore`. Sem segredos no repo.
- Use **`catkin build`**, nunca `catkin_make`. Ambiente via Docker (preferir 20.04/Noetic).
- Preserve compatibilidade com o upstream `ethz-asl/kalibr`; concentre customizações em `scripts/`.
- `resolution`/`rostopic` dos YAMLs devem casar com os dados reais, senão a calibração falha.

## Diretrizes de PR / commit

- O commit do fork usou `feat:`. `NEEDS CLARIFICATION`: confirmar padrão de commit/branch com o dono.
