# Plano de execução — dataset v2 (`cam_imu_dvl_v2`, 2026-09-18)

> Nova bateria de 3 bags, mesmo cenário (submerso, mesmo alvo). Este doc: o que mudou em relação à
> v1, o que ainda é premissa, e a ordem de execução.
> Contexto: `premissas-e-fontes-de-erro.md` · `testes-reprocessamento.md` · `analise-crossbag-camimu.txt`

## Os dados

| bag | duração | tamanho | vazão | imagens | `/imu/data` | `/dvl/data` |
|---|---|---|---|---|---|---|
| `11-39-12` | 240.9 s | 25.6 GB | 106 MB/s | 3468 @ 14.4 Hz | 48160 @ 199.9 Hz | 2577 @ 10.7 Hz |
| `11-44-41` | 233.6 s | 24.9 GB | 107 MB/s | 3371 @ 14.4 Hz | 46308 @ 198.3 Hz | 2428 @ 10.4 Hz |
| `11-50-20` | 215.9 s | 23.1 GB | 107 MB/s | 3125 @ 14.5 Hz | 43185 @ 200.0 Hz | 2330 @ 10.8 Hz |

Imagens `bgra8` 1280×720 cruas (não comprimidas), `sensor_msgs/Image`, timestamps em epoch Unix
(→ rebasing obrigatório, D8). Também gravados: `/zed/zed_node/imu/data` (99 Hz),
`/ekf/imu/data` (100 Hz, **nunca usar** — saída de EKF), `camera_info`, `/lar/bar/depth`.

## O que MELHOROU vs. v1 ✅

### Os gaps de gravação sumiram

| | IMU: gaps / perdido / maior | câmera: gaps / perdido / maior |
|---|---|---|
| v1 bag 03 | 4 / **69.2 s** / 40.8 s | 4 / 69.3 s / 40.9 s |
| v1 bag 04 | 21 / **51.4 s** / 9.3 s | 19 / 52.0 s / 9.3 s |
| **v2 11-39** | 3 / **0.0 s** / 0.02 s | 7 / 1.4 s / 0.21 s |
| **v2 11-44** | 2 / **0.0 s** / 0.02 s | 12 / 2.5 s / 0.35 s |
| **v2 11-50** | 2 / **0.0 s** / 0.02 s | 8 / 1.6 s / 0.21 s |

IMU praticamente perfeita (maior buraco = 4 amostras). Câmera perde frames isolados (≤0.35 s), não
os buracos de 40 s da v1. **Mesma vazão (107 MB/s), gravador aguentando** — algo melhorou no sistema
de gravação.

**Consequências diretas:**
- A B-spline não tem mais trechos sem observação.
- O **prior de timeshift volta a ser confiável**: o defeito do `np.mean(np.diff(times))`
  (`IccSensors.py:253`) era proporcional à inflação do `dT`, que na v1 chegou a 31.5%; aqui é ~0.15%.
- **Não é preciso `--bag-from-to`** — o teste 3 da v1 perde o motivo de existir.

### O gating do DVL está bem calibrado (bag 11-39)

| campo | min | p50 | p95 | max |
|---|---|---|---|---|
| `fom` | 0.000 | 0.002 | 0.003 | 10.000 |
| `altitude` | −1.000 | 0.361 | 0.503 | 1.037 |
| `\|v\|` | 0.000 | 0.122 | 0.258 | 0.451 |

`velocity_valid` em 98.8%. Com os limiares atuais do `dvl0.yaml`, **98.8% das amostras sobrevivem**
(descartes: 3 por `fom`, 3 por `altitude`<0.1, 0 por velocidade). Os limiares estão adequados — não
precisam de ajuste. **Fecha o teste 2 do DVL.**

## O que NÃO melhorou ❌

### A excitação rotacional continua igual (ligeiramente menor)

| | giro RMS por eixo (°/s) | \|ω\| (rad/s) | sensib. centrípeta (m/s²)/m |
|---|---|---|---|
| v1 (4 bags) | 5.9 – 9.2 | 0.19 – 0.23 | 0.05 – 0.15 |
| **v2 11-39** | 5.8 / 3.8 / 5.5 | **0.155** | 0.053 / 0.048 / 0.032 |
| **v2 11-44** | 7.0 / 3.9 / 6.2 | **0.177** | 0.076 / 0.068 / 0.043 |
| **v2 11-50** | 7.6 / 5.0 / 5.8 | **0.188** | 0.098 / 0.098 / 0.045 |

A recomendação de subir para ~20 °/s RMS não foi aplicada. Como o sinal do lever-arm escala com **ω²**,
a v2 tem *menos* informação sobre a translação câmera-IMU que a v1.

> **Expectativa calibrada:** espere **melhor consistência** (sem gaps corrompendo spline e prior), mas
> **não** espere precisão de lever-arm melhor que a v1. Se a dispersão da translação continuar em
> ~2 cm, a causa não são mais os gaps — é o nível de sinal.

## O que continua sendo premissa não verificada ⚠️

| # | Premissa | Estado |
|---|---|---|
| 1 | `tagSize = 0.088 m` | **não medido** — define a escala métrica e contamina `velocity_scale` |
| 2 | `sound_speed = 1500 m/s` | **não confirmado** no A50 |
| 3 | Intrínsecos da ZED (arquivo, submerso) | reusados, **k3 dropado** (D7) |
| 4 | Warm-up antes de gravar | bags às 11:39, 11:44, 11:50 — **5 min de intervalo, igual à v1**. Houve warm-up antes do primeiro? |
| 5 | Ruído das IMUs | datasheet provisório (D3) |
| 6 | `config01/` da v2 | **é cópia da v1** — `dvl0.yaml` aponta para `dvl0_calib01.csv` (CSV da v1!), `update_rate: 9` (real: 10.7) |

### 🆕 Achado: os intrínsecos do arquivo divergem muito do SDK da ZED

O `camera_info` gravado traz a calibração do SDK (ar):

| | arquivo (submerso) | SDK (ar) | razão |
|---|---|---|---|
| `fx` | 1372.12 | 957.79 | **1.433** |
| `fy` | 1352.89 | 958.18 | **1.412** |
| `cx` | 615.33 | 640.88 | −25.5 px |
| `cy` | 378.29 | 354.27 | +24.0 px |
| modelo | `radtan` (4) | `rational_polynomial` (**8**) | — |

- A razão ~1.42 é **coerente com refração** (índice da água ≈1.333), confirmando que o arquivo é
  mesmo uma calibração submersa. Mas está ~7% acima do previsto por `n_água` puro.
- O **ponto principal se deslocou ~25 px** nos dois eixos. Refração por porta plana bem alinhada não
  deveria mover tanto o ponto principal — pode indicar desalinhamento da porta ou erro de calibração.
- A ZED precisa de **8 coeficientes racionais** para modelar a própria lente; nós usamos **4 radtan**.
  É um descasamento de modelo grande, consistente com a reprojeção de 1.3–1.5 px.

> Reforça a prioridade de **calibrar os intrínsecos com o próprio Kalibr** — e agora temos uma
> referência independente (o SDK, no ar) para checar sanidade: `fx_água / fx_ar` deve dar ~1.33–1.43.

### 🔄 Correção: o `time_of_validity` do DVL **não** é usável como timestamp absoluto

Registrei antes que usar `header.stamp` no `ros2_dvl_to_csv.py` era um bug (violando R5). **A medição
desmente isso:**

```
time_of_validity[0] = 1716816236492214 us -> 2024-05-27
header.stamp[0]     = 1789742354.741 s    -> 2026-09-18
diferença de época  = 72.926.118 s = 2.32 anos
```

O `time_of_validity` está no **relógio interno do A50**, sem sincronia com o sistema. Usá-lo
diretamente como timestamp seria catastrófico. **O uso atual de `header.stamp` está correto.**

O refinamento que *faria* sentido: usar o `tov` só para o espaçamento **relativo**, ancorado à época do
header. Ganho medido: o jitter do intervalo cai de **20.6 ms → 17.5 ms** (std), com offset
`header − tov` praticamente constante (std 8.9 ms). É melhoria modesta, opcional.

---

## Plano de execução

### Fase 0 — Preparo (host, sem calibração) — ~1 h

- [ ] **0.1** Atualizar `cam_imu_dvl_v2/config01/`: `dvl0.yaml` (`csv:` para o CSV novo de cada bag,
      `update_rate: 11`), `imu.yaml` (comentário de taxa). `camchain.yaml`/`target.yaml` seguem por ora.
- [ ] **0.2** Converter os 3 bags ROS 2 → ROS 1 (`ros2_to_ros1_kalibr.py`, mono8, `--zero-start`).
      Anotar o `T0_NS` de cada. ~15 min/bag; saída ~6.5 GB/bag (~20 GB no total).
- [ ] **0.3** Extrair os CSVs do DVL com o **mesmo `t0`** de cada bag.
      **Verificação:** contagem ≈ 2577 / 2428 / 2330.

### Fase 1 — Cam-IMU e repetibilidade — ~1,5 h  ← **o teste principal**

- [ ] **1.1** `kalibr_calibrate_imu_camera` nos 3 bags (Microstrain referência, `--timeoffset-padding 0.1`,
      `MPLBACKEND=Agg`). Usar `run_v2_camimu.sh`, que já grava no layout `runs/<bag>__<variante>/` com symlink.
- [ ] **1.2** `compare_calibrations.py` nos 3 → **a dispersão da translação cai abaixo dos 1.7 cm da v1?**
  - **Se cair** → os gaps eram uma causa real; a calibração final sai daqui.
  - **Se não cair** → confirma que o limite é o nível de sinal (ω²), não a qualidade do dado.
    A conclusão passa a ser: *sem excitação maior, ~2 cm é o piso prático desta montagem*.
- [ ] **1.3** Comparar o viés vs. xacro com o da v1 (13.5 cm). Dois datasets independentes concordando
      no mesmo viés é evidência forte de erro de cota no CAD.

### Fase 2 — Diagnósticos de timing — ~1 h

- [ ] **2.1** Timeshift dos 3 bags vs. horário de gravação. **Agora é um teste limpo do D9**: sem gaps
      corrompendo o prior, se a queda monotônica reaparecer, a hipótese de assentamento se sustenta;
      se não reaparecer, o efeito da v1 era artefato dos gaps.
- [ ] **2.2** Split-half do melhor bag (metades iguais) → variação intra-bag vs. inter-bag.

### Fase 3 — Intrínsecos (maior ganho potencial) — ~2 h

- [ ] **3.1** `kalibr_calibrate_cameras` num bag: `pinhole-radtan` **×** `pinhole-equi` (`--bag-freq 4`).
- [ ] **3.2** Sanidade contra o SDK: `fx_estimado / 957.79` deve cair em ~1.33–1.43.
- [ ] **3.3** Re-rodar a Fase 1 com o melhor modelo → a reprojeção cai de ~1.3 px? A dispersão aperta?

### Fase 4 — DVL — depois que a Fase 1/3 estabilizar

- [ ] **4.1** `kalibr_calibrate_dvl` (Modo A) nos 3, reusando o cam-IMU do **mesmo** bag.
- [ ] **4.2** Comparação cross-bag de `T_dvl_imu` / `velocity_scale` / timeshift.

### Custo zero — fora do computador (destrava interpretação)

- [ ] **A** Medir o `tagSize` do AprilGrid com paquímetro, **molhado**. Sem isso, `velocity_scale` é
      inseparável do erro de escala do alvo.
- [ ] **B** Confirmar o `sound_speed` configurado no A50 durante a gravação.
- [ ] **C** Confirmar se houve warm-up antes das 11:39 (fecha ou reabre o D9).
- [ ] **D** Conferir no CAD a cota da Microstrain em x (xacro: −0.09439; dados: ≈ +0.02).

## Decisões em aberto para o usuário

1. **Excitação:** aceitamos ~2 cm de precisão no lever-arm, ou vale uma 4ª coleta com rotação
   ~3× mais forte (~20 °/s RMS)? O ganho seria ~9× em sinal.
2. **Ordem:** ir direto ao DVL depois da Fase 1 (objetivo do projeto), ou passar pela Fase 3
   (intrínsecos) antes, aceitando ~2 h a mais para uma base melhor?
3. **Só 3 bags** (v1 tinha 4). Com 3 pontos a estatística de dispersão fica frágil — vale gravar mais?
