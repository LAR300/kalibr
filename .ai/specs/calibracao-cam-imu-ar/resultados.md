# Resultados — Calibração câmera-IMU fora d'água (dataset v3)

> Requisitos em `spec.md`, escolhas técnicas em `decisions.md`, tarefas em `plan.md`.
> **Estado: CONCLUÍDO.** 11 execuções da matriz + 1 variante extra (ruído corrigido).

---

## 1. Resposta à pergunta central (R10 / CA7)

> **O pipeline do Kalibr funciona fora d'água? SIM — no bag `raw`, de forma inequívoca.**

| | v1 submerso (melhor) | v2 submerso | **v3 ar (raw)** |
|---|---|---|---|
| iterações | 20 | 17+ sem convergir | **5** |
| `lambda` final | 0.18 | **33 209** | **0.12** |
| parou por | tolerância | não parou | **tolerância** |
| reprojeção | 1.29 px | — | **0.67 px** |
| resíduos normalizados | 1.2 / 2.2 / 0.67 | ~4.7 | **0.67 / 0.68 / 0.66 / 0.52** |
| timeshift | 4.3 ms | 4–9 ms | **1.67 ms** |

Mesmo código, mesmos sensores, mesmas flags. **Cinco ordens de grandeza** de diferença no
condicionamento. A hipótese de que o problema anterior era a água (refração não modelada +
intrínsecos de calibração submersa) fica confirmada.

**Qualificação importante:** isto vale para o bag **raw**. O bag **rect** falha, por motivo
*diferente* e ainda não totalmente estabelecido (§4).

---

## 2. Matriz completa

> **Visão resumida** abaixo; a tabela com *todos* os valores por cenário
> (incluindo rotação, deslocamento e baseline estimado) está em §2.1.

| execução | repr c0 | repr c1 | giroN | accelN | iter | `lambda` | timeshift |
|---|---|---|---|---|---|---|---|
| `raw__micro-fabrica` (baseline congelado) | 3.257 | 3.186 | 0.822 | 0.544 | 8 | 0.0046 | 1.91 ms |
| **`raw__micro-fabrica`** (baseline livre) | **0.676** | **0.683** | 0.645 | 0.517 | **5** | **0.12** | 1.64 ms |
| **`raw__micro-kalibr`** | **0.673** | **0.682** | 0.660 | 0.520 | **5** | **0.12** | 1.67 ms |
| `raw__zed-fabrica` | 0.570 | 0.573 | 0.422 | **0.222** | 30 | 0.91 | −4.67 ms |
| `raw__zed-kalibr` | 0.563 | 0.569 | 0.424 | **0.224** | 30 | 1.10 | −4.55 ms |
| 🔴 `rect__micro-fabrica` | 0.762 | 0.709 | **1.215** | 0.878 | 30 | **11 919** | 0.05 ms |
| 🔴 `rect__micro-kalibr` | 0.763 | 0.708 | **1.270** | 0.944 | 30 | **4 918** | −0.93 ms |
| 🔴 `rect__zed-fabrica` | 0.596 | 0.588 | 2.015 | 0.710 | 30 | 69.3 | **−40.49 ms** |
| 🔴 `rect__zed-kalibr` | 0.596 | 0.585 | **1.940** | 0.721 | 30 | **425.8** | — |
| `raw__zed-ruidocorrigido` *(extra)* | 0.592 | 0.598 | 0.474 | **0.163** | 30 | 2.02 | — |

### `t_ic` — posição da câmera no frame da IMU (metros)

| execução | cam0 | \|t\| |
|---|---|---|
| `raw__micro-fabrica` (congelado) | [0.1968, 0.0570, −0.0029] | 0.2049 |
| `raw__micro-fabrica` (livre) | [0.1960, 0.0575, −0.0025] | 0.2042 |
| `raw__micro-kalibr` | [0.1904, 0.0542, −0.0040] | 0.1980 |
| `rect__micro-fabrica` | [0.2419, 0.0466, 0.0109] | 0.2466 |
| `rect__micro-kalibr` | [0.2276, 0.0506, 0.0077] | 0.2333 |
| `raw__zed-fabrica` | [−0.0228, 0.0390, 0.0108] | 0.0465 |
| `raw__zed-kalibr` | [−0.0250, 0.0399, 0.0108] | 0.0484 |
| 🔴 `rect__zed-fabrica` | [0.4244, 0.0609, 0.0252] | **0.4295** |
| `rect__zed-kalibr` | — | 0.0405 (t_ci) |
| `raw__zed-ruidocorrigido` | — | 0.0582 (t_ci) |

---

## 2.1 Tabela completa — todos os valores por cenário

Convenção: `T_ci` é a transformação **IMU → câmera**; a translação é a **posição da IMU no frame
óptico da câmera esquerda**. `eixo-âng` é o ângulo da rotação em forma eixo-ângulo. `desvPerm` é o
desvio dessa rotação em relação a uma **permutação exata de eixos** (`cam_x=−imu_y`, `cam_y=−imu_z`,
`cam_z=+imu_x`) — é a medida do desalinhamento mecânico real. `baseline` é a distância cam0↔cam1
estimada, contra 0.12009 m de fábrica.

### Grupo `raw` + Microstrain (IMU externa)

| cenário | reprC0 | reprC1 | giroN | accN | it | `lambda` | tshift | t_ci [m] | \|t\| | eixo-âng | desvPerm | baseline |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `micro-fabrica` (congelado) | 3.257 | 3.186 | 0.822 | 0.544 | 8 | 0.0046 | 1.91 ms | [0.060, −0.005, −0.196] | 204.9 mm | 120.16° | 1.16° | 0.12009 |
| **`micro-fabrica`** (livre) | 0.676 | 0.683 | 0.645 | 0.517 | **5** | **0.124** | 1.64 ms | [0.060, −0.005, −0.195] | 204.2 mm | 119.99° | 1.13° | 0.11982 |
| **`micro-kalibr`** | 0.673 | 0.682 | 0.660 | 0.520 | **5** | **0.124** | 1.67 ms | [0.058, −0.006, −0.189] | **198.0 mm** | 120.19° | 1.27° | 0.12030 |

### Grupo `raw` + IMU da ZED (interna)

| cenário | reprC0 | reprC1 | giroN | accN | it | `lambda` | tshift | t_ci [m] | \|t\| | eixo-âng | desvPerm | baseline |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `zed-fabrica` | 0.570 | 0.573 | 0.422 | **0.222** | 30 | 0.910 | −4.67 ms | [0.039, 0.011, 0.022] | 46.5 mm | 120.55° | 1.63° | 0.11977 |
| `zed-kalibr` | 0.563 | 0.569 | 0.424 | **0.224** | 30 | 1.100 | −4.55 ms | [0.040, 0.011, 0.025] | **48.4 mm** | 120.74° | 1.44° | 0.12024 |
| `zed-ruidocorrigido` | 0.592 | 0.598 | 0.474 | **0.163** | 30 | 2.019 | −2.88 ms | [0.047, 0.017, 0.029] | 58.2 mm | 120.68° | 1.17° | 0.12026 |

### Grupo `rect` — todos comprometidos 🔴

| cenário | reprC0 | reprC1 | giroN | accN | it | `lambda` | tshift | t_ci [m] | \|t\| | eixo-âng | desvPerm | baseline |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `micro-fabrica` | 0.762 | 0.709 | **1.215** | 0.878 | 30 | **11 919** | 0.05 ms | [0.049, 0.009, −0.241] | 246.6 mm | 120.15° | 0.88° | 0.11970 |
| `micro-kalibr` | 0.763 | 0.708 | **1.270** | 0.944 | 30 | **4 918** | −0.93 ms | [0.052, 0.007, −0.227] | 233.3 mm | 120.22° | 0.57° | 0.11983 |
| `zed-fabrica` | 0.596 | 0.588 | **2.015** | 0.710 | 30 | 69.25 | **−40.49 ms** | [0.057, 0.035, −0.424] | **429.5 mm** | 120.97° | 1.62° | 0.11975 |
| `zed-kalibr` | 0.596 | 0.585 | **1.940** | 0.721 | 30 | 425.8 | **−40.50 ms** | [0.002, 0.001, −0.040] | 40.5 mm | 120.59° | 1.85° | 0.11989 |

### O que as colunas de rotação e baseline revelam

**A rotação é estável em TODAS as execuções — inclusive nas quebradas.** O eixo-ângulo fica entre
119.99° e 120.97° nas dez, e o desvio de permutação entre 0.57° e 1.85°. Até a execução que
posicionou a IMU a 429 mm da câmera acertou a rotação.

**O baseline também é imune:** 0.11970 a 0.12030 m, contra 0.12009 de fábrica — dispersão de 0.6 mm,
sem correlação alguma com o `lambda`.

> **Rotação e translação têm observabilidade muito diferente.** A rotação vem da comparação direta
> entre a direção da gravidade e a orientação do alvo — sinal forte e abundante. A translação depende
> do termo `ω × r`, que é fraco. Quando o problema degrada, **a translação quebra primeiro**; a
> rotação e o baseline seguem certos e dão uma falsa sensação de que está tudo bem.

**Os 120° não são coincidência:** uma permutação cíclica de eixos com dois sinais trocados é
exatamente uma rotação de 120° em torno de `[0.577, −0.577, 0.577]`. Os desvios de ~1.2° em relação
a ela são o desalinhamento mecânico real entre os sensores — compatível com tolerância de montagem.

**Rotação derivada entre as duas IMUs:** 2.44° (`micro → câmera → ZED`). Como acumula os erros das
duas composições, é consistente com dois desalinhamentos independentes de ~1.3°.

**O `rect__zed-kalibr` é o caso mais traiçoeiro da tabela:** 40.5 mm é uma faixa plausível, mas com
`lambda` 426, giroN 1.94 e timeshift de −40.5 ms contra −4.6 ms do mesmo par no `raw`. **Valor
plausível com diagnóstico ruim é coincidência, não resultado.**

**O timeshift separa os grupos com nitidez:**

```
raw  + micro :  +1.64 a +1.91 ms
raw  + ZED   :  −2.88 a −4.67 ms
rect + micro :  −0.93 a +0.05 ms
rect + ZED   :  −40.5 ms          <- implausivel para o mesmo hardware
```

Sinais opostos entre Microstrain e ZED são esperados (caminhos de aquisição distintos). Os −40 ms
do `rect+ZED`, não.

### 2.2 Mesma tabela em ângulos de Euler (RPY) — ⚠️ com ressalva de gimbal lock

Formato de leitura humana, com translação nos 3 eixos e rotação nos 3 ângulos (convenção ROS/URDF,
`R = Rz(yaw)·Ry(pitch)·Rx(roll)`).

| execução | reprC0 | reprC1 | giroN | accelN | iter | `lambda` | timeshift | tx [m] | ty [m] | tz [m] | roll° | pitch° | yaw° | roll+yaw° | ⊿rot |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `raw__micro-fabrica` (congelado) | 3.257 | 3.186 | 0.822 | 0.544 | 8 | 0.0046 | 1.91 ms | 0.0600 | −0.0053 | −0.1959 | −48.73 | −88.85 | 138.89 | 90.16 | 0.16° |
| **`raw__micro-fabrica`** (livre) | 0.676 | 0.683 | 0.645 | 0.517 | **5** | **0.124** | 1.64 ms | 0.0600 | −0.0053 | −0.1952 | −39.59 | −88.88 | 129.71 | 90.12 | ref |
| **`raw__micro-kalibr`** | 0.673 | 0.682 | 0.660 | 0.520 | **5** | **0.124** | 1.67 ms | 0.0575 | −0.0065 | −0.1893 | −52.44 | −88.74 | 142.53 | 90.09 | 0.30° |
| `raw__zed-fabrica` | 0.570 | 0.573 | 0.422 | **0.222** | 30 | 0.910 | −4.67 ms | 0.0392 | 0.0112 | 0.0223 | 130.93 | −88.78 | −39.86 | 91.07 | ref |
| `raw__zed-kalibr` | 0.563 | 0.569 | 0.424 | **0.224** | 30 | 1.100 | −4.55 ms | 0.0400 | 0.0112 | 0.0248 | 145.74 | −88.98 | −54.72 | 91.02 | 0.36° |
| 🔴 `rect__micro-fabrica` | 0.762 | 0.709 | **1.215** | 0.878 | 30 | **11 919** | 0.05 ms | 0.0492 | 0.0086 | −0.2415 | −47.71 | −89.14 | 137.91 | 90.20 | — |
| 🔴 `rect__micro-kalibr` | 0.763 | 0.708 | **1.270** | 0.944 | 30 | **4 918** | −0.93 ms | 0.0525 | 0.0068 | −0.2272 | −61.56 | −89.45 | 151.72 | 90.16 | — |
| 🔴 `rect__zed-fabrica` | 0.596 | 0.588 | **2.015** | 0.710 | 30 | 69.3 | **−40.49 ms** | 0.0567 | 0.0351 | −0.4243 | 157.80 | −88.71 | −66.83 | 90.97 | — |
| 🔴 `rect__zed-kalibr` | 0.596 | 0.585 | **1.940** | 0.721 | 30 | 425.8 | −40.50 ms | 0.0022 | 0.0010 | −0.0404 | 150.39 | −88.18 | −60.05 | 90.34 | — |
| `raw__zed-ruidocorrigido` | 0.592 | 0.598 | 0.474 | **0.163** | 30 | 2.019 | −2.88 ms | 0.0475 | 0.0165 | 0.0294 | 154.19 | −89.12 | −63.41 | 90.78 | — |

#### ⚠️ Por que `roll` e `yaw` desta tabela NÃO devem ser lidos isoladamente

O `pitch` fica entre −88.2° e −89.5° em **todas** as execuções, ou seja a ~1° do **gimbal lock**
(−90°). Nessa região a decomposição RPY é singular: `roll` e `yaw` deixam de ser independentes e só
a **soma** deles é determinada.

Demonstração com duas execuções do mesmo bag e mesma IMU, variando só a fonte de intrínsecos:

```
                roll     pitch      yaw   |  roll+yaw
  fabrica     -39.59    -88.88   129.71   |    90.12
  kalibr      -52.44    -88.74   142.53   |    90.09
  DIFERENCA   -12.85     +0.14   +12.82   |    -0.03

  rotacao REAL entre as duas (eixo-angulo): 0.301 graus
```

O RPY sugere **12.85°** de diferença onde a rotação real difere em **0.30°** — inflação de ~40×,
puro artefato da singularidade. Note que `roll+yaw` fica estável.

**Como ler, então:**

| coluna | confiável? |
|---|---|
| `pitch` | ✅ sim, isoladamente |
| `roll`, `yaw` | ❌ **não** isoladamente |
| `roll+yaw` | ✅ é a combinação determinada (90.09–91.07 nas dez execuções) |
| `⊿rot` | ✅ rotação real vs. a execução de referência do grupo — **use esta para repetibilidade** |

> **Para registrar ou transmitir a calibração, use a matriz de rotação ou o quaternion** do
> `camchain-imucam.yaml`, nunca o RPY. O YAML do Kalibr já traz a matriz 4×4 sem ambiguidade. O RPY
> aqui serve só para leitura humana, e com esta ressalva.

---

## 3. CA5 — recalibrar os intrínsecos **não** vale a pena

Este é o resultado mais acionável da bateria.

| par | fábrica | Kalibr | diferença |
|---|---|---|---|
| `raw__micro` | 0.676 / 0.683 px | 0.673 / 0.682 px | **0.003 px** |
| `raw__zed` | 0.570 / 0.573 px | 0.563 / 0.569 px | **0.007 px** |
| `rect__micro` | 0.762 / 0.709 px | 0.763 / 0.708 px | **0.001 px** |

Estimar os intrínsecos com o Kalibr deu ganho **desprezível**. Ou seja: o `radtan`-4 ajustado ao
modelo racional do SDK (`scripts/rational_to_radtan.py`) é **tão bom quanto** uma calibração
completa feita nas próprias imagens.

> **Validação forte do `rational_to_radtan.py`.** E contraste com o histórico: o que custava
> reprojeção no fluxo submerso não era "usar os intrínsecos de fábrica", era **truncar** o modelo
> racional (989% de erro na borda). Ajustado corretamente, o de fábrica basta.

### ⚠️ Ressalva: "equivalente" vale para o AJUSTE, não para a RESPOSTA

Os intrínsecos das duas fontes **não são iguais** — e as diferenças são estatisticamente enormes:

| | fábrica | Kalibr | diferença | sigmas |
|---|---|---|---|---|
| `cx` | 640.875 | 647.720 ± 0.70 | **+6.85 px** | ~10σ |
| `fy` | 958.185 | 960.827 ± 0.33 | +2.64 | ~8σ |
| `k1` | −0.0699 | −0.0514 ± 0.0015 | +0.0185 | ~12σ |

A reprojeção não vê isso porque **o erro é absorvido pela pose**: 6.85 px de deslocamento do ponto
principal a `fx`=958 equivalem a 0.41° de giro da trajetória. Ponto principal e orientação da câmera
são quase degenerados — deslocar um parece girar o outro. (Evidência de que é giro comum e não
ruído: os dois `cx` se moveram na mesma direção, +6.85 e +6.95 px.)

**Mas o erro não some — ele vai parar no extrínseco**, que é o que queremos medir:

```
raw__micro-fabrica :  |t_ic| = 0.2042 m
raw__micro-kalibr  :  |t_ic| = 0.1980 m
                      diferenca = 6.6 mm
```

> **Qualificação necessária:** para a **qualidade do ajuste** as duas fontes são equivalentes. Para a
> **resposta final** elas divergem em 6.6 mm, e não há como dizer qual está mais perto da verdade.
> Os 6.6 mm são da mesma ordem da precisão perseguida — "tanto faz" seria leitura errada.

**Recomendação:** usar os intrínsecos de fábrica com o ajuste racional→radtan quando o objetivo for
ter uma calibração funcional rápido — economiza ~1 h por bag. Se o objetivo for o valor mais exato
possível do lever-arm, a escolha entre as duas fontes **permanece indecidida** por falta de
referência externa.

### Comparação dos intrínsecos (bag raw)

| | fábrica (K) | ajuste racional→radtan | Kalibr estimado |
|---|---|---|---|
| fx | 957.790 | — | 958.153 ± 0.34 |
| fy | 958.185 | — | 960.827 ± 0.33 |
| cx | 640.875 | — | 647.720 ± 0.70 |
| cy | 354.266 | — | 356.860 ± 0.47 |
| k1 | *(racional)* | −0.0699 | −0.0514 ± 0.0015 |
| baseline | 0.120093 | — | **0.120093** ± 0.0001 |

O **baseline de fábrica é exato** (coincide em <0.01 mm). O ponto principal de fábrica erra ~6.8 px
em `cx`. Mesmo assim, o efeito na reprojeção final é desprezível.

---

## 4. 🔴 O bag `rect` não é utilizável — e minha hipótese inicial foi refutada

### O sintoma

`lambda` de 4 918 a 11 919 (contra 0.12 do raw), resíduo normalizado de giroscópio **acima de 1**
(1.215 / 1.270 — os únicos da bateria), e 30 iterações sem parar por tolerância.

### Falha silenciosa: `rect__zed-fabrica`

`t_ic` = [0.4244, 0.0609, 0.0252] → **429.5 mm** entre a câmera e a IMU da ZED. As duas ficam **no
mesmo corpo**, a ~23 mm uma da outra. É fisicamente impossível.

E a reprojeção desse run é **0.596 px** — ótima. O timeshift saiu em −40.49 ms contra −4.67 ms do
mesmo par de sensores no bag raw.

> **A reprojeção não detecta este erro.** Só o `lambda`, o resíduo inercial normalizado e a
> plausibilidade física revelam. É o modo de falha que a spec previu no risco do camchain do rect:
> *"converge e dá número errado sem avisar"*.

### O que o diagnóstico (tarefa 3.2) mostrou

O `camera_info` do tópico `rect` **não descreve** as imagens retificadas:

| | fábrica declarada | Kalibr estimado (cam0) | significância |
|---|---|---|---|
| `cy` | 357.195 | 364.274 ± 0.65 | **10.9σ** (~8 px) |
| `k1` | 0.0 | 0.0120 ± 0.0011 | **11σ** |
| `p1` | 0.0 | 0.0030 ± 0.0002 | **15σ** |
| `fx` vs `fy` | iguais (947.799) | 954.28 vs 956.08 | diferem |
| `cy` cam0 vs cam1 | devem ser iguais | 364.27 vs 366.51 | 2.5σ |

Há **distorção residual real** nas imagens ditas retificadas, e o ponto principal está ~8 px fora.
A última linha é a mais reveladora: num par retificado o `cy` das duas câmeras **tem** que ser
idêntico — é a definição de alinhamento por linhas. Não é.

### ⚠️ Mas a hipótese dos intrínsecos foi REFUTADA

A execução `rect__micro-kalibr` usou os intrínsecos **estimados** (com o `cy` correto e a distorção
residual modelada) — e mesmo assim ficou com `lambda` **4 918** e deu `t_ic` de 0.2333 m.

> **Corrigir os intrínsecos não corrigiu o mal-condicionamento.** A causa raiz do problema do bag
> `rect` **continua não estabelecida**. O que sobra de hipótese: algo no próprio fluxo de
> retificação do SDK (interpolação, crop, ou timestamp), ou algo específico daquela gravação
> (17-15-26), que não é o modelo de câmera.

**Nuance, com a matriz completa — o efeito é misto, não nulo:**

| célula | `lambda` fábrica → Kalibr | geometria fábrica → Kalibr |
|---|---|---|
| `rect__micro` | 11 919 → 4 918 (melhorou 2.4×, ainda ruim) | 0.2466 → 0.2333 m |
| `rect__zed` | 69 → 426 (**piorou**) | **429.5 → 40.5 mm** (consertou) |

Corrigir os intrínsecos **consertou a geometria absurda** do par rect+ZED (de 430 mm para 40.5 mm,
faixa plausível) mas **não** o condicionamento. As quatro células do rect ficaram com `lambda` de
69, 426, 4 918 e 11 919 — **nenhuma combinação de IMU ou intrínsecos salva esse bag**.

**Recomendação operacional:** gravar em **raw** e deixar o Kalibr lidar com a distorção. O stream
retificado do SDK não é confiável para calibração.

---

## 5. CA3 — consistência entre as duas gravações: **não atingida**

| | `raw` | `rect` | diferença |
|---|---|---|---|
| \|t_ic\| com Microstrain | 0.198 – 0.204 m | 0.233 – 0.247 m | **~43 mm** |

Mas isto **não é um teste justo de repetibilidade**: o bag rect está mal-condicionado. O que o teste
de fato fez foi **sinalizar uma configuração quebrada** — que era um dos seus propósitos.

**Consistência interna, onde é possível medir, é boa:**
- `raw` fábrica × Kalibr: 0.2042 vs 0.1980 m → **6 mm**
- `rect` fábrica × Kalibr: 0.2466 vs 0.2333 m → 13 mm

### O baseline estéreo é recuperado corretamente em **todas** as execuções

Distância cam0↔cam1 medida: **0.1197 – 0.1198 m** contra 0.1201 m de fábrica, inclusive na execução
que deu o lever-arm de 430 mm. A geometria estéreo é sólida em todo lugar; o que varia é o
**braço câmera↔IMU**.

---

## 6. CA6 — verificação geométrica: ⚠️ **a referência não é confiável**

> **Rebaixado depois de verificar a origem do número.** Esta seção começou como "a única âncora
> geométrica confiável". Não é.

### Por que a referência não vale

O valor nominal `[-0.002, -0.023, -0.002]` que usei **não vem do `zed_macro` do fabricante** — o
macro oficial (`zed-ros2-wrapper/zed_wrapper/urdf/zed_macro.urdf.xacro`) **não define `imu_link`
nenhum**. O valor vem do `petro_rov.urdf.xacro` do próprio projeto, cujo comentário diz:

> *"IMU interna da ZED 2i (posição **aproximada** relativa à câmera esquerda)."*

E esse arquivo é de **2026-08-11**, cinco semanas **antes** das gravações do v3 (18/09) — ou seja,
é da estrutura antiga.

**Duas falhas na premissa:** o valor é uma aproximação feita à mão, não uma cota de fabricante; e o
arquivo é anterior à reestruturação.

### Os números, para registro

Nominal assumido (frame óptico): [0.023, 0.002, −0.002] m = **23.2 mm**.

| execução | t_ci medido | \|t\| | "erro" |
|---|---|---|---|
| `raw__zed-fabrica` | [0.0392, 0.0112, 0.0223] | 46.5 mm | 30.6 mm |
| `raw__zed-kalibr` | [0.0400, 0.0112, 0.0248] | 48.4 mm | 33.0 mm |
| `raw__zed-ruidocorrigido` | [0.0475, 0.0165, 0.0294] | **58.2 mm** | 42.4 mm |
| 🔴 `rect__zed-fabrica` | [0.0567, 0.0351, −0.4243] | 429.5 mm | descartar |
| `rect__zed-kalibr` | [0.0022, 0.0010, −0.0404] | 40.5 mm | 43.7 mm |

**O que dá para afirmar:** as execuções do bag `raw` concordam entre si em **2 mm** (46.5 / 48.4 mm)
— a *repetibilidade* é boa. Se a discrepância de ~25 mm contra o nominal é erro da estimação ou do
nominal aproximado, **não é possível decidir** com a referência disponível.

**Para fechar isto de verdade** seria preciso uma cota medida da IMU interna da ZED 2i — do
datasheet do fabricante ou por medição física.

## 7. 🔄 Ruído da IMU da ZED: a correção **refutou** minha hipótese

Na primeira rodada o ruído da ZED saiu inflado demais (resíduo normalizado do acelerômetro em
**0.222**, ~4.5× abaixo de 1). Minha hipótese: com a IMU sub-pesada, a câmera domina e o lever-arm
— que é observado justamente pela IMU — fica mal determinado. Previsão: dividir o ruído por ~4.5
levaria o resíduo normalizado para ~1 e o lever-arm para perto do nominal.

**Executei o teste (`raw__zed-ruidocorrigido`). A previsão falhou nos dois pontos:**

| | inflado (original) | corrigido (÷4.5) | previsto |
|---|---|---|---|
| accel normalizado | 0.222 | **0.163** | ~1.0 |
| giro normalizado | 0.422 | 0.474 | ~1.0 |
| lever-arm | 46.5 mm | **58.2 mm** | ~23.2 mm |
| reprojeção | 0.570 px | 0.592 px | — |
| `lambda` final | 0.91 | 2.02 | — |

**O resíduo normalizado CAIU em vez de subir.** Reduzir o σ declarado por 4.5× deveria multiplicar
o resíduo normalizado por 4.5. Que ele tenha caído para 0.163 significa que o **erro bruto** do
acelerômetro despencou ~3.3× quando o otimizador passou a confiar mais na IMU — ele reajustou a
trajetória para casar com os dados inerciais.

**E o lever-arm afastou-se do nominal** (46.5 → 58.2 mm), em vez de se aproximar.

> **Conclusão honesta:** a ponderação da IMU **não** explica a discrepância do §6. Combinada com a
> descoberta de que a própria referência é uma aproximação feita à mão (§6), a questão do lever-arm
> da ZED fica **em aberto** — e não é resolúvel com os dados e referências atuais.

O que permanece válido da D6: inflar é mais seguro que subestimar. O que se aprendeu de novo é que
o resíduo normalizado **não é um termômetro confiável** para ajustar o ruído quando a trajetória é
livre — o otimizador compensa movendo a spline, não só os parâmetros.

## 8. ⚠️ Comparação com o CAD: suspensa

Durante a execução reportei que o v3 concordava com o CAD em 2.7 cm (contra 13.5 cm do submerso), o
que sugeria derrubar o **D10** da spec de tanque. **Isso não vale:** o v3 foi gravado com uma
**estrutura nova**, com a IMU externa reposicionada, e comparei contra as cotas antigas.

`compare_calibrations.py` agora exige `--nominal <arquivo>` e avisa quando cai no built-in antigo.

**Falta do usuário:** cotas novas em `base_link` de `zed_node_camera_link` e `imu_link` (e a
orientação, se o joint não for mais `rpy="0 0 0"`).

---

## 9. Incidente: travamento da máquina

Durante a última execução da matriz (`rect__zed-kalibr`) a máquina travou e reiniciou
(boot encerrado 09:00:51, próximo às 09:01:57, sem sequência de desligamento).

**Evidência:** 30 s antes, `gdm-x-session: client bug: event processing lagging behind by 51ms,
your system is too slow`. Não há mensagem de OOM killer — consistente com congelamento por
inanição de CPU/thrashing **antes** de o killer agir. Evidência circunstancial forte, não prova.

**Causa estrutural, essa sim confirmada:** o container `kalibr_zed` rodava **sem limite nenhum**
(`Memory: 0`, `NanoCpus: 0`, `privileged`), podendo tomar os 15 GB e os 20 cores da máquina.
Aquela execução rodou **3 h** contra ~25 min das demais.

**Correção aplicada** (via `docker update`, sem recriar o container):

```
Memory: 9 GB · MemorySwap: 9 GB · CPUs: 14
```

`MemorySwap == Memory` dá **swap zero** ao container: se estourar, o kernel mata o processo dele em
vez de arrastar a máquina para thrashing. E 6 cores + 6 GB ficam reservados ao host, mantendo a
interface viva.

**Medido depois da correção:** container em 4.9 GiB / 9 GiB e **1373% de CPU** (~14 de 14 cores).
Confirma que, sem teto, ele saturava os 20 cores — o que explica diretamente a mensagem do X.

---

## 10. Premissas ainda abertas

| premissa | estado | impacto |
|---|---|---|
| `tagSize = 0.088 m` | **não medido** | escala métrica de todas as translações |
| ruído da IMU da ZED | **comprovadamente inflado ~4.5×** | qualidade do lever-arm; §6 e §7 |
| ruído da Microstrain | datasheet provisório (resíduos ~0.5–0.65, razoáveis) | menor |
| cotas novas do xacro | **faltando** | bloqueia só a comparação com o CAD |
| causa raiz do bag `rect` | **não estabelecida** | o stream retificado é inutilizável até entender |

---

## 11. Recomendações

1. **Gravar em `raw`, não em `rect`.** O stream retificado do SDK não é confiável para calibração:
   tem distorção residual significativa, ponto principal fora por 8 px, `cy` desalinhado entre as
   câmeras — e o mal-condicionamento persiste mesmo com os intrínsecos corrigidos.
2. **Usar os intrínsecos de fábrica** com o ajuste racional→radtan. Recalibrar com o Kalibr dá
   ganho de 0.003 px e custa ~1 h por bag.
3. **Corrigir o ruído da IMU da ZED** (dividir por ~4) antes de comparar as duas IMUs.
4. **Manter os limites de recurso do container.** Sem eles, um run patológico derruba a máquina.
5. **Para o DVL:** a base está pronta do lado câmera-IMU **com o bag raw e a Microstrain** —
   `lambda` 0.12, reprojeção 0.67 px, resíduos todos < 1, timeshift 1.67 ms.

---

## 12. Seguir para o DVL? (tarefa 6.2)

### A tensão que este resultado expõe

O sucesso do v3 **não desbloqueia o DVL diretamente**, e vale ser explícito sobre o porquê:

- A calibração do DVL **exige água** — o A50 precisa de bottom-lock, e não há dado de DVL nestes bags.
- A água é exatamente onde o pipeline falhava.
- O extrínseco câmera-IMU medido aqui **não é reaproveitável** submerso: o modelo de câmera muda
  (refração), e a estrutura do ROV também mudou entre as coletas.

> O que o v3 entrega não é uma calibração utilizável no SLAM. É a **prova de que o método e a
> ferramenta estão corretos** — o que restringe o problema submerso a uma causa: o modelo óptico.

### O que fazer diferente na próxima coleta submersa

O v3 converteu suspeitas em certezas, e três delas mudam o procedimento:

1. **Calibrar os intrínsecos DEBAIXO D'ÁGUA com o próprio Kalibr**, nos mesmos bags — em vez de
   reusar o arquivo de calibração submersa antigo. Essa etapa nunca foi executada (era a Fase 3 do
   plano do v2, que não chegou a rodar). É a mudança de maior impacto: é o único parâmetro que a
   água altera e que hoje entra como premissa.
   - **Ressalva:** no ar, recalibrar deu ganho desprezível (§3). Submerso a expectativa é oposta —
     lá o modelo de partida está errado por construção, não apenas impreciso.
2. **Testar `pinhole-equi` além de `pinhole-radtan`.** A refração de porta plana produz uma
   distorção que o `radtan` não representa bem (e que depende da distância). O modelo equidistante
   pode absorvê-la melhor. Barato de testar: são dois runs de `kalibr_calibrate_cameras`.
3. **Gravar em `raw` e em cinza**, nunca no stream retificado (§4), e manter a excitação em ~11 °/s
   isotrópica, que foi o que funcionou aqui.

### Checklist de sanidade para qualquer calibração futura

Derivado dos modos de falha encontrados nesta investigação:

| verificar | bandeira vermelha | por quê |
|---|---|---|
| `lambda` final do LM | > ~10 | mal-condicionamento; o v2 chegou a 33 209 |
| parada | por `max-iter`, não por tolerância | não convergiu de fato |
| resíduos inerciais **normalizados** | > 1 (mal ajustado) ou << 1 (ruído inflado) | devem ficar perto de 1 |
| plausibilidade física | lever-arm fora da ordem esperada | `rect__zed` deu 430 mm com reprojeção de 0.6 px |
| baseline estéreo estimado | difere do de fábrica | foi o único indicador sólido em todas as execuções |
| gaps nos tópicos | qualquer perda > alguns segundos | corrompe a spline e o prior de timeshift |
| excitação rotacional | < ~10 °/s RMS, ou anisotrópica | lever-arm mal observável; prior de tempo ruidoso |

> **A reprojeção sozinha não serve como critério de qualidade.** Duas execuções desta bateria têm
> reprojeção excelente e resultado inutilizável.

### Recomendação

**Não voltar ao DVL ainda.** A ordem que faz sentido:

1. Medir o `tagSize` (custo zero, destrava a escala métrica e a interpretação da `velocity_scale`).
2. Fornecer as cotas novas do xacro (custo zero, destrava a validação geométrica).
3. Coletar submerso **com calibração de intrínsecos submersa pelo Kalibr** e excitação ~11 °/s.
4. Validar que o cam-IMU submerso converge com `lambda` < 10 e resíduos normalizados ~1.
5. **Só então** o DVL, em Modo A, reusando esse cam-IMU.

Pular o passo 4 é repetir o ciclo do v1/v2.

---

## 13. Fechamento — o que ficou estabelecido e o que não

### ✅ Estabelecido com confiança

1. **O pipeline do Kalibr funciona fora d'água.** Bag `raw` + Microstrain: 5 iterações, `lambda`
   0.12, parada por tolerância, reprojeção 0.67 px, todos os resíduos normalizados < 1. Contra o v2
   submerso travado em `lambda` 33 209. **O problema anterior era a água.**
2. **Recalibrar intrínsecos não vale a pena** (0.001–0.007 px de diferença). O ajuste
   racional→radtan dos valores de fábrica é equivalente a uma calibração completa.
3. **Truncar o `rational_polynomial` é catastrófico** (989% de erro na borda) — e era um erro fácil
   de cometer sem perceber.
4. **O stream retificado do SDK não serve para calibração.** Quatro células, `lambda` de 69 a
   11 919, nenhuma configuração salva.
5. **O baseline estéreo é recuperado corretamente em todas as 11 execuções** (0.1197–0.1198 m
   contra 0.1201 de fábrica) — inclusive nas que produziram geometria absurda.
6. **A reprojeção não é critério de qualidade suficiente.** Duas execuções com reprojeção < 0.6 px
   entregaram resultado inutilizável.
7. **O container precisa de limites de recurso.** Sem eles derrubou a máquina; a fase de extração
   satura 20 cores.

### ❌ NÃO estabelecido — questões que ficam abertas

1. **A causa raiz do mal-condicionamento do bag `rect`.** Os intrínsecos errados eram uma parte
   (consertaram a geometria do par rect+ZED) mas não a causa do `lambda` alto.
2. **O lever-arm da IMU interna da ZED.** As execuções `raw` concordam entre si em 2 mm (46–48 mm)
   mas divergem ~25 mm do nominal. Não é possível decidir de quem é o erro: a referência é uma
   aproximação feita à mão num xacro anterior à estrutura atual (§6), e a correção do ruído da IMU
   **refutou** a explicação que eu havia proposto (§7).
3. **A comparação com o CAD do ROV.** Bloqueada pela falta das cotas novas (§8).
4. **A escala métrica absoluta.** O `tagSize` nunca foi medido; todas as translações herdam o erro.

### 🔄 Hipóteses minhas que os dados derrubaram

Registradas porque o processo de eliminá-las é parte do resultado:

| hipótese | desfecho |
|---|---|
| o CAD errava 13 cm na posição da Microstrain (D10) | comparação inválida — frame errado, depois xacro errado |
| o v3 concordava com o CAD em 2.7 cm | inválido — comparei com cotas de outra estrutura |
| o estéreo estava dessincronizado em 31 ms | erro meu de medição (tempo de bag vs. `header.stamp`) |
| o `T_cn_cnm1` do `camera_info` servia | 0.389° de erro; não usar para isso |
| os intrínsecos errados causavam o problema do `rect` | parcial — consertam a geometria, não o `lambda` |
| o ruído inflado da ZED explicava o lever-arm | refutado por experimento direto |
| o `time_of_validity` do DVL estava sendo ignorado indevidamente | não era bug — está em outra época |

### Insumos necessários do usuário

1. **`tagSize` medido** com paquímetro (custo zero, destrava a escala métrica).
2. **Cotas novas do xacro**: `zed_node_camera_link` e `imu_link` em `base_link`. O arquivo no repo
   é de 2026-08-11, anterior às gravações.
3. **Cota de fabricante da IMU interna da ZED 2i**, se existir, para fechar o §6.
