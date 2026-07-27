---
description: Testes do Kalibr — o que existe (GTest de error terms) e como rodar. Abrir ao mexer nos error terms C++ ou adicionar testes.
sources: [aslam_offline_calibration/kalibr/test/TestErrorTerms.cpp, aslam_offline_calibration/kalibr/test/test_main.cpp, catkin_simple/test/]
---

# Testes

> Cobertura de testes é **escassa** e concentrada no C++. Não há suíte Python abrangente.

## O que existe
- `aslam_offline_calibration/kalibr/test/TestErrorTerms.cpp` + `test_main.cpp` — **GTest** para os
  error terms de IMU em C++ (`AccelerometerError`, `GyroscopeError`, `EuclideanError`).
- `catkin_simple/test/` — testes da infraestrutura de build.
- Não há teste de ponta a ponta do pipeline de calibração Python.

## Como rodar
Dentro do workspace catkin (container ROS), os testes GTest são construídos e executados via catkin:
```bash
catkin build kalibr --catkin-make-args run_tests   # constrói e roda os testes do pacote
catkin_test_results                                 # agrega os resultados
```
`NEEDS CLARIFICATION`: confirmar o alvo exato de teste no `CMakeLists.txt` do pacote `kalibr`
(o upstream expõe os testes via `catkin_add_gtest`); não há doc do projeto sobre execução de testes.

## Convenções
- Testes C++ ficam em `<pacote>/test/`, nomeados `Test*.cpp`, com um `test_main.cpp` de entrada GTest.
- Ao alterar um error term (`src/{Accelerometer,Gyroscope,Euclidean}Error.cpp`), atualize/rode
  `TestErrorTerms.cpp`.

## O que NÃO é testado automaticamente
- O pipeline de alto nível (`IccSensors`, `IccCalibrator`, CLIs) — validado na prática rodando as
  ferramentas sobre datasets e inspecionando o report PDF / erro de reprojeção (ver `data-model.md`).
