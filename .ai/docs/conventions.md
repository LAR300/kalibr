---
description: Convenções de código, layout do workspace catkin e build do Kalibr. Abrir antes de escrever ou revisar código, ou de adicionar um pacote.
sources: ["**/CMakeLists.txt", "**/package.xml", .gitignore, .github/workflows/, scripts/]
---

# Convenções

> Kalibr é um workspace catkin multi-pacote maduro. Siga o padrão do pacote em que você mexe.

## Layout do workspace
- Cada pacote catkin tem seu `package.xml` + `CMakeLists.txt`. Pacotes agrupados por metapacote
  (`Schweizer-Messer`, `aslam_optimizer`, `aslam_cv`, `aslam_nonparametric_estimation`,
  `aslam_incremental_calibration`, `aslam_offline_calibration`). Ver `architecture.md`.
- Núcleo em C++ com **bindings Python** (padrão `<pkg>` + `<pkg>_python`). O ponto de entrada de
  usuário é o pacote `aslam_offline_calibration/kalibr`.
- CLIs de usuário: `aslam_offline_calibration/kalibr/python/kalibr_*` (executadas via `rosrun kalibr <tool>`).
- Módulos Python de apoio: `python/kalibr_common/` (parse de config, leitura de dataset, extração de target).

## Estilo de código
- **C++14**; sem formatter configurado. Mantenha o estilo do arquivo vizinho.
- **Python 3** (exceto o ambiente Kinetic/16.04). Sem lint configurado.

## Build
- Método oficial: **`catkin build`** (catkin_tools), tipicamente com `-DCMAKE_BUILD_TYPE=Release`.
  **Não** use `catkin_make`.
- Ambiente recomendado: Docker 20.04/Noetic (`Dockerfile_ros1_20_04`, usado pelo `scripts/Makefile`).

## Dados e segredos
- `.gitignore` ignora binários, `*.pyc`, `build/`, IDEs e explicitamente **`./scripts/data/*`**
  (dados de calibração — imagens, bags, resultados — **não** são versionados).
- Não versione bags, imagens de calibração nem YAMLs de resultado com dados reais.

## CI
- `.github/workflows/`: builds Docker para 16.04/18.04/20.04. Mudanças que afetam build devem passar
  nos três (ou justificar).

## Commits e PRs
- Upstream segue estilo GitHub (PRs com merge). O commit deste fork usou prefixo `feat:`
  (`feat: utilizacao do kalibr para calibracao de camera e imu`).
- `NEEDS CLARIFICATION`: confirmar com o dono se há padrão de commit/branch imposto para este fork.

## Princípios
- Preserve compatibilidade com o upstream `ethz-asl/kalibr` onde possível (facilita rebase/merge).
  Concentre customizações em `scripts/`. YAGNI, DRY.
