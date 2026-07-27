# Kalibr — Calibração de DVL (câmera + IMU + DVL)

> Extensão deste fork: a ferramenta **`kalibr_calibrate_dvl`** estima, offline, o extrínseco de um DVL
> (Water Linked A50) em relação à IMU de referência, junto com a escala de velocidade (sound-speed) e o
> offset temporal — reusando a calibração câmera-IMU submersa já feita pelo Kalibr.
>
> Contexto e decisões de projeto: `.ai/specs/dvl-calibration/`. Fluxo geral de calibração dos 4 sensores:
> ver também `scripts/docs/kalibr.md` (câmera + IMU).

## Sumário

1. [O que a ferramenta calibra](#o-que-a-ferramenta-calibra)
2. [Pré-requisitos](#pré-requisitos)
3. [Coleta de dados no tanque](#coleta-de-dados-no-tanque)
4. [Preparo dos dados (ROS 2 → ROS 1 + CSV do DVL)](#preparo-dos-dados)
5. [Arquivos de entrada](#arquivos-de-entrada)
6. [Rodando a calibração](#rodando-a-calibração)
7. [Arquivos de resultado](#arquivos-de-resultado)
8. [Como interpretar os resultados](#como-interpretar-os-resultados)
9. [Modo A vs Modo B](#modo-a-vs-modo-b)
10. [Troubleshooting](#troubleshooting)

---

## O que a ferramenta calibra

| Parâmetro | Descrição |
|---|---|
| **`T_dvl_imu`** | Transformação rígida (SE3) da IMU de referência para o DVL (rotação + lever-arm). |
| **`velocity_scale`** | Fator de escala da velocidade do DVL (captura erro de velocidade do som na água). |
| **`timeshift_dvl_imu`** | Offset temporal entre os relógios do DVL e da IMU. |

**Como funciona (resumo):** o DVL não vê o alvo — só mede velocidade. A ferramenta reconstrói a
**B-spline de pose** do corpo a partir de câmera+IMU (do próprio bag), e compara a velocidade medida
pelo DVL com a **derivada da spline** transformada por `T_dvl_imu·velocity_scale`. Minimiza esse resíduo
para estimar os parâmetros do DVL. Detalhes: `.ai/specs/dvl-calibration/decisions.md`.

---

## Pré-requisitos

1. **Calibração câmera-IMU submersa já feita** (pelo `kalibr_calibrate_imu_camera`), produzindo:
   - `camchain-imucam.yaml` (intrínsecos + `T_cam_imu`),
   - `imu.yaml` (ruído da IMU; e `T_i_b` se multi-IMU).
   A **Microstrain** deve ter sido a IMU de referência (`--imu` #1).
2. **Ruído da IMU** já estimado (Allan variance) — ver `scripts/docs/kalibr.md`.
3. **Ambiente Docker do Kalibr** (`scripts/Makefile` → `make build && make run`).
4. **Config do DVL** (`dvl0.yaml`) — ver `scripts/config/dvl0.yaml`.

---

## Coleta de dados no tanque

Grave **um bag combinado** (ROS 2) com **estéreo retificado + IMU (Microstrain) + DVL**, submerso:

- **Alvo visível E bottom-lock ao mesmo tempo:** o cilindro precisa apontar a câmera ao AprilGrid
  submerso e o DVL a uma superfície refletora (fundo do tanque), com `altitude` dentro do alcance do
  A50 (~0,05–50 m).
- **Excitação 6-DOF:** movimente em translação **e** rotação em todos os eixos — a rotação é o que
  torna o **lever-arm** (translação de `T_dvl_imu`) observável (`v_dvl = v_imu + ω × r`).
- **Sem perder bottom-lock:** movimentos vigorosos, mas mantendo o fundo à vista (trechos sem
  bottom-lock são descartados automaticamente pelo gating).
- **Duração:** ~60–120 s costuma bastar.
- **Anote o `sound_speed`** configurado no A50 durante a captura (define o valor esperado de `velocity_scale`).

---

## Preparo dos dados

Os bags são gravados em **ROS 2**; o Kalibr é **ROS 1**:

```bash
# 1) Converter o bag ROS 2 -> ROS 1 (tópicos de imagem e IMU)
rosbags-convert meu_bag_ros2/  --dst  cam_imu_dvl.bag

# 2) Extrair o stream do DVL (dvl_msgs/DVL) para CSV (schema em scripts/config/dvl0_example.csv)
#    -> dvl0.csv com 16 colunas: timestamp_ns, vx,vy,vz, cov00..cov22, velocity_valid, fom, altitude
#    (script extrator de referência: ver plano, tarefa 3.4 — roda no lado ROS 2)
```

> **Por que CSV?** `dvl_msgs/DVL` é mensagem custom; extrair para CSV evita ter de regerar a mensagem
> em ROS 1. Alternativa: regerar `dvl_msgs` em ROS 1 e usar `rostopic` no `dvl0.yaml`.

---

## Arquivos de entrada

- **`camchain-imucam.yaml`** — a calibração câmera-IMU existente (reusada). Passar em `--cams`.
- **`imu.yaml`** — ruído/extrínsecos da IMU. Passar em `--imu`.
- **`target.yaml`** — o mesmo AprilGrid da calibração de câmera. Passar em `--target`.
- **`dvl0.yaml`** — config do DVL (ver `scripts/config/dvl0.yaml`): fonte (CSV), `sound_speed`,
  `T_dvl_imu` inicial (do xacro), `velocity_scale` inicial, flags e **gating**.
- **`dvl0.csv`** — stream do DVL (16 colunas). Passar em `--dvl-csv` (ou no campo `csv` do `dvl0.yaml`).

---

## Rodando a calibração

Dentro do container (com o bag e os yamls em `/data`):

```bash
rosrun kalibr kalibr_calibrate_dvl \
  --bag      /data/cam_imu_dvl.bag \
  --cams     /data/camchain-imucam.yaml \
  --imu      /data/imu.yaml \
  --target   /data/config/target.yaml \
  --dvl      /data/config/dvl0.yaml \
  --dvl-csv  /data/dvl0.csv \
  --max-iter 30
```

Flags úteis:

| Flag | Efeito |
|---|---|
| `--recompute-cam-imu` | **Modo B** (co-otimiza câmera-IMU-DVL). Padrão é **Modo A** (cam-IMU fixo). |
| `--no-time-calibration` | Desabilita a calibração temporal câmera-IMU. |
| `--recover-covariance` | Recupera covariância (do cam-IMU; DVL é reportado por resíduo). |
| `--huber-dvl <w>` | M-estimator de Huber nos resíduos do DVL (robustez a outliers). |
| `--dont-show-report` | Não abrir o relatório na tela ao final. |

---

## Arquivos de resultado

Gerados ao lado do bag (`<bag>` = nome do bag sem extensão):

| Arquivo | Conteúdo |
|---|---|
| `<bag>-dvl-results.yaml` | `T_dvl_imu` (SE3 4×4), `velocity_scale`, `timeshift_dvl_imu`, `sound_speed`. |
| `<bag>-results-dvl.txt` | Resumo textual + estatísticas de resíduo (RMS, por-eixo, nº usados/descartados). |
| `<bag>-camchain-imucam.yaml` | Calibração câmera-IMU reusada (re-otimizada no Modo B). |
| `<bag>-imu.yaml` | Parâmetros da IMU. |
| `<bag>-report-dvl.pdf` | Relatório com o gráfico de resíduos de velocidade do DVL. |

---

## Como interpretar os resultados

- **`T_dvl_imu` — translação:** deve bater com a montagem física (o xacro), dentro de poucos cm.
  Valores muito diferentes indicam pouca excitação rotacional ou dados ruins.
- **`velocity_scale`:** deve ficar próximo do esperado para o `sound_speed` configurado. Um desvio grande
  sugere erro de velocidade do som (salinidade/temperatura) — que é justamente o que este fator corrige.
- **RMS do resíduo de velocidade [m/s]:** quanto menor, melhor. Deve ficar da ordem do ruído do DVL
  (compare com a `covariance`/`fom` reportada pelo A50). Outliers no gráfico de resíduos indicam trechos
  com bottom-lock ruim.
- **`timeshift_dvl_imu`:** pequeno para sensores bem sincronizados; valores grandes sugerem problema de
  sincronização/latência.
- **nº usados vs descartados:** muitas amostras descartadas → bottom-lock intermitente ou gating apertado.

---

## Modo A vs Modo B

- **Modo A (padrão):** reusa a calibração câmera-IMU submersa **fixa**; estima só o DVL. Mais robusto,
  respeita a calibração já validada. Requer que o `--cams` (camchain-imucam.yaml) contenha `T_cam_imu`.
- **Modo B (`--recompute-cam-imu`):** co-otimiza câmera-IMU-DVL num único batch. Máxima consistência,
  mais parâmetros; use se quiser refinar tudo junto.

---

## Troubleshooting

- **"nenhuma fonte de dados do DVL (csv)":** informe `--dvl-csv` ou o campo `csv` no `dvl0.yaml`.
- **Muitas amostras descartadas:** afrouxe o `gating` no `dvl0.yaml` (`min_altitude`, `max_fom`,
  `max_speed`) ou verifique o bottom-lock na captura.
- **Translação de `T_dvl_imu` estranha / lever-arm mal estimado:** falta excitação **rotacional** —
  recapture com mais rotação em todos os eixos.
- **`velocity_scale` muito longe de 1:** confira o `sound_speed` configurado no A50 vs. o real da água.
- **Modo A avisa "no T_cam_imu in --cams":** passe o `camchain-imucam.yaml` (saída do
  `kalibr_calibrate_imu_camera`), não o `camchain.yaml` (só intrínsecos).
- **RMS alto:** cheque a sincronização (offset), a qualidade do bottom-lock e a calibração câmera-IMU base.
