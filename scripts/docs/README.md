# Calibração completa (câmera + IMU + DVL) — passo a passo

> Documento **mestre**: amarra as 4 etapas de calibração do rig subaquático (câmera estéreo ZED 2i,
> IMU da ZED, IMU externa Microstrain, DVL Water Linked A50), na ordem correta. Cada etapa tem um guia
> detalhado próprio — este doc é o índice/fluxo.

## Visão geral: 4 produtos, nesta ordem

Cada etapa depende da anterior (do sensor "mais interno" ao "mais externo"):

```
1. Intrínsecos + estéreo da câmera   →  camchain.yaml
             │  (usa os intrínsecos)
2. Ruído das IMUs (Allan variance)   →  microstrain.yaml, zed.yaml
             │  (usa camchain + ruído)
3. Câmera–IMU + IMU–IMU               →  camchain-imucam.yaml (T_cam_imu, timeshifts) + imu.yaml
             │  (reusa tudo acima FIXO — Modo A)
4. DVL                                →  dvl-results.yaml (T_dvl_imu, velocity_scale, timeshift)
```

**Referência de frame:** a **Microstrain** é a IMU de referência (`--imu` #1). Todos os extrínsecos
saem no frame dela; compõem-se para qualquer outro frame depois.

## Quantos bags gravar

Para a primeira vez (do zero), recomenda-se **4 bags** — isola cada variável:

| Bag | Conteúdo | Ambiente | Movimento | Duração |
|---|---|---|---|---|
| **1 — Intrínsecos** | estéreo **raw** | submerso, alvo visível | **lento**, cobrindo todo o FOV | 1–2 min |
| **2 — Ruído IMU** | Microstrain + ZED IMU | **parado** (pode em ar) | estático | horas (Allan) |
| **3 — Câmera-IMU** | estéreo **retificado** + 2 IMUs | submerso, alvo visível | **6-DOF vigoroso** | 1–2 min |
| **4 — DVL** | estéreo **retificado** + IMU + **DVL** | submerso, alvo + bottom-lock | **6-DOF** dentro do alcance do DVL | 1–2 min |

> Depois de confortável, os bags 3 e 4 podem virar um só (rodar etapa 3 e depois 4 no mesmo bag; ou
> Modo B para co-otimizar tudo). Ver `kalibr_dvl.md`.

## Passo a passo (com os guias detalhados)

### Etapa 1 — Intrínsecos + estéreo  ·  guia: `kalibr.md`
```bash
rosrun kalibr kalibr_calibrate_cameras \
  --bag /data/bag1_intrinsics.bag \
  --topics /zed/left/image_raw /zed/right/image_raw \
  --models pinhole-radtan pinhole-radtan \
  --target /data/config/target.yaml
```
→ `camchain.yaml`. Confira o erro de reprojeção (<0,5 px é ótimo).

### Etapa 2 — Ruído das IMUs (Allan variance)
Ferramenta **separada** (o Kalibr não traz Allan variance): `allan_variance_ros` (ROS 1/2, recomendado),
`imu_utils` ou `kalibr_allan`. Bag da IMU **parada** por horas (pode em ar). Costuma-se **inflar** o
`noise_density` (~5–10×) antes de usar no Kalibr. Uma vez por IMU.
→ `microstrain.yaml`, `zed.yaml`.

### Etapa 3 — Câmera-IMU + IMU-IMU  ·  guia: `kalibr.md`
```bash
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag /data/bag3_cam_imu.bag \
  --cams /data/camchain_rectified.yaml \
  --imu /data/microstrain.yaml /data/zed.yaml \   # 1a = REFERÊNCIA (Microstrain)
  --target /data/config/target.yaml
```
→ `camchain-imucam.yaml` (com `T_cam_imu`) + `imu.yaml`.

### Etapa 4 — DVL  ·  guia: `kalibr_dvl.md`
```bash
# (preparo) bag ROS 2 -> ROS 1 + extrair dvl0.csv (16 colunas) do dvl_msgs/DVL
rosrun kalibr kalibr_calibrate_dvl \
  --bag /data/bag4_cam_imu_dvl.bag \
  --cams /data/camchain-imucam.yaml \   # reusado FIXO (Modo A)
  --imu  /data/imu.yaml \
  --target /data/config/target.yaml \
  --dvl  /data/config/dvl0.yaml \
  --dvl-csv /data/dvl0.csv
```
→ `dvl-results.yaml` (`T_dvl_imu`, `velocity_scale`, `timeshift_dvl_imu`).

## Resultado final

Todos os extrínsecos amarrados à **Microstrain**:
`camchain.yaml` (intrínsecos+estéreo) · `camchain-imucam.yaml` (`T_cam_imu`) · `imu.yaml` (`T_zed_micro`) ·
`dvl-results.yaml` (`T_dvl_imu`, escala, timeshift). Compõem-se para qualquer par
(ex.: `T_dvl_cam = T_dvl_imu · T_imu_cam`) — útil para alimentar o AQUA-SLAM (trabalho futuro).

## Guias detalhados
- Câmera + IMU: `kalibr.md`
- DVL: `kalibr_dvl.md`
- Design/decisões da calibração de DVL: `../../.ai/specs/dvl-calibration/`
