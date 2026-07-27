---
description: Visão geral do Kalibr em uma página — o que é, para quem e como as peças se encaixam. Abrir primeiro, para situar-se antes de mergulhar nos docs específicos.
sources: [README.md, scripts/docs/kalibr.md, aslam_offline_calibration/kalibr/python/]
---

# Visão geral

> O "mapa" do projeto em uma página. O `AGENTS.md` aponta pra cá; o detalhe vive nos docs específicos.

## O que é

Kalibr é uma **toolbox de calibração** de câmeras e IMUs, do Autonomous Systems Lab (ETH Zurich).
Estima, por otimização batch em tempo contínuo com B-splines, os parâmetros espaciais e temporais
entre sensores. Resolve quatro problemas (README):

1. **Multi-Camera** — intrínsecos e extrínsecos de sistemas multi-câmera (mesmo sem FOV sobreposto global).
2. **Visual-Inertial (CAM-IMU)** — calibração espacial e temporal de IMU vs. câmeras + intrínsecos da IMU.
3. **Multi-Inertial (IMU-IMU)** — IMU vs. IMU de referência (com 1 câmera de apoio).
4. **Rolling Shutter** — intrínsecos completos (projeção, distorção, shutter) de câmeras rolling shutter.

> **Sobre este checkout:** é um fork do `ethz-asl/kalibr`. O último commit adicionou a pasta
> `scripts/` (Makefile Docker + configs de exemplo + guia em português) para calibrar **câmera+IMU
> de uma ZED 2i**. O resto é o Kalibr upstream. O fluxo do fork foca em **CAM-IMU**.

## Problema que resolve

Fusão visual-inercial (VIO/SLAM: VINS, MSCKF, ORB-SLAM3, OKVIS, ROVIO) só funciona se souber, com
precisão, a transformação câmera↔IMU (`T_cam_imu`), o offset temporal entre relógios
(`timeshift`) e os intrínsecos da câmera. Kalibr estima tudo isso a partir de um dataset onde o
sensor observa um alvo conhecido (AprilGrid/checkerboard) enquanto é movido nos 6 graus de liberdade.

## Como as peças se encaixam

```
dados brutos (imagens por timestamp + imu.csv)
        │  kalibr_bagcreater
        ▼
   ROS bag  ──►  kalibr_calibrate_cameras  ──►  camchain.yaml (intrínsecos + extrínsecos câmera-câmera)
        │                                                │
        └──────────────►  kalibr_calibrate_imu_camera  ◄─┘  + imu.yaml + target.yaml
                                    │
                                    ▼
              camchain-imucam.yaml (T_cam_imu, timeshift) + imu.yaml + report.pdf
```

Por baixo, cada ferramenta monta um problema de otimização: a trajetória do sensor é uma **B-spline
de pose contínua**, os biases da IMU são B-splines, e os resíduos (reprojeção + acelerômetro +
giroscópio) são minimizados pelo `aslam_backend`. Detalhes em `architecture.md`.

## Estado atual

Ferramenta madura e amplamente usada na comunidade de robótica (upstream ativo). Este fork acrescenta
apenas o empacotamento Docker + guia CAM-IMU para ZED 2i em `scripts/`. Roda como CLIs `rosrun kalibr *`
dentro de um container ROS 1.

## Por onde começar a ler

- Arquitetura e o porquê → `architecture.md`
- Stack e versões → `stack.md`
- Convenções → `conventions.md`
- Formatos de entrada/saída (YAMLs) → `data-model.md`
- Ferramentas de linha de comando → `api.md`
- Fluxo CAM-IMU passo a passo → `domain/cam-imu-calibration.md` (e o guia completo em `scripts/docs/kalibr.md`)
