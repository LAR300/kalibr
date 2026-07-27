---
description: Linguagens, frameworks e libs do Kalibr com versões, e ambientes Docker suportados. Abrir ao adicionar dependência, mexer no build ou escolher o ambiente ROS.
sources: [Dockerfile_ros1_16_04, Dockerfile_ros1_18_04, Dockerfile_ros1_20_04, scripts/Makefile, aslam_offline_calibration/kalibr/setup.py, "**/CMakeLists.txt", "**/package.xml"]
---

# Stack

> Kalibr não pina versões de dependências: elas vêm do apt da distro escolhida. O Docker é a
> fonte de verdade do ambiente. Prefira o ambiente **20.04 / Noetic**.

## Linguagens
- **C++14** (`set(CMAKE_CXX_STANDARD 14)`, padrão dominante em ~15 CMakeLists).
- **Python 3** (migração concluída no upstream, PR #515/2022). O `setup.py` usa `catkin_pkg`.
  Exceção: `Dockerfile_ros1_16_04` (Kinetic) ainda usa Python 2.7.

## Build
- **catkin (catkin_tools — `catkin build`)** + CMake. Não há uso de `catkin_make` no upstream.
- Config típica: `catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release`.
- Macros de conveniência: `catkin_simple` (`cs_add_library`, `cs_export`) em alguns pacotes.

## ROS suportado (Dockerfiles na raiz)
| Dockerfile | ROS | Ubuntu | Python |
|---|---|---|---|
| `Dockerfile_ros1_16_04` | **Kinetic** | 16.04 | 2.7 (wxgtk3.0) |
| `Dockerfile_ros1_18_04` | **Melodic** | 18.04 | 3 (mistura libs `python-*`, wxgtk4.0) |
| `Dockerfile_ros1_20_04` | **Noetic** | 20.04 | 3 (`python3-*`, catkin_tools nativo) |

Apenas **ROS 1**. O `scripts/Makefile` deste fork usa o `Dockerfile_ros1_20_04` (Noetic).

## Dependências externas (via apt, sem pin)
- **Eigen3** (`libeigen3-dev`), **Boost** (`libboost-all-dev`; componentes `system`, `thread`, ...),
  **SuiteSparse** (`libsuitesparse-dev`, solver esparso), **OpenCV** (`libopencv-dev`),
  **POCO** (`libpoco-dev`), **TBB**, **BLAS/LAPACK**, **libv4l**.
- Python: **scipy**, **matplotlib**, **wxPython** (GUI dos reports), **python-igraph**, **pyx**
  (geração de PDF), **python-tk**.
- `NEEDS CLARIFICATION`: versões exatas de Eigen/Boost/OpenCV/SuiteSparse dependem do repositório
  apt de cada distro; não estão pinadas no repo.

## Ferramental
- Testes: **GTest** (poucos testes C++ de error terms; ver `testing.md`).
- Lint/format: **nenhum** configurado (sem `.clang-format`, `.editorconfig`, `.flake8`, `pyproject.toml`).
- CI: workflows Docker 1604/1804/2004 em `.github/workflows/` (o `industrial_ci` está `.disable`).

## Empacotamento Docker deste fork (`scripts/`)
- `scripts/Makefile`: imagem `kalibr`, container `kalibr_zed`, a partir de `Dockerfile_ros1_20_04`.
  Targets `build/run/start/stop/exec/clean/status`, X11 forwarding e volume `scripts/data → /data`.
- Ver o fluxo completo em `scripts/docs/kalibr.md` e `domain/cam-imu-calibration.md`.
