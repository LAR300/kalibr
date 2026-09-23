---
description: Mapa dos parâmetros ASSUMIDOS (não estimados) no fluxo de calibração câmera-IMU-DVL submerso, com impacto, degenerescências e como resolver cada um. Abrir antes de planejar nova coleta de dados ou ao investigar um resultado suspeito.
sources: [scripts/config/sensors/zed2i01/zed_opencv_calibration.yaml, scripts/data/calibration/cam_imu_dvl/config01/, scripts/ros2_dvl_to_csv.py, aslam_offline_calibration/kalibr/python/kalibr_calibrate_imu_camera, aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccSensors.py, .ai/specs/validacao-calibracao-tanque/decisions.md]
---

# Premissas e fontes de erro (calibração submersa)

> Sistema: ZED 2i (estéreo + IMU) + MicroStrain 3DM-GV7 (IMU de referência) + Water Linked A50 (DVL),
> num cilindro, para SLAM subaquático. Bags gravados em ROS 2, submersos; calibração da ZED feita
> submersa; Kalibr roda em ROS 1.
>
> Este doc lista o que o pipeline **assume** em vez de estimar. Cada premissa errada vira erro
> sistemático que **nenhuma quantidade de dados corrige** — por isso vale mapear antes de coletar mais.

## O que o Kalibr realmente estima (verificado no código)

| Grandeza | Estimada? | Onde |
|---|---|---|
| `T_cam0_imu0` (extrínseco câmera-IMU) | ✅ sim | `IccSensors.py:524-525` (`camNr==0` → DVs ativos) |
| `timeshift_cam_imu` | ✅ sim | `cameraTimeToImuTimeDv`, ativo salvo `--no-time-calibration` |
| Trajetória (B-spline), biases da IMU, gravidade | ✅ sim | — |
| **Intrínsecos da câmera** | ❌ **não** | `kalibr_calibrate_imu_camera` não tem DV de intrínseco |
| **Baseline estéreo `T_cn_cnm1`** | ❌ **não** (default) | `noChainExtrinsics=True` salvo `--recompute-camera-chain-extrinsics` |
| **Escala/desalinhamento da IMU** | ❌ **não** (default) | `--imu-models` default `calibrated` |
| **Incerteza dos parâmetros** | ❌ **não** (default) | `--recover-covariance` não usado |

> Ou seja: intrínsecos e baseline entram como **verdade absoluta** e qualquer erro neles é absorvido
> pelo extrínseco e pela trajetória.

---

## 1. Escala métrica — o grupo mais crítico

A escala de todo o sistema vem do **tamanho físico do alvo**. Ela se propaga para todas as translações
e, no passo do DVL, para a `velocity_scale`.

| Parâmetro | Valor atual | Status | Impacto |
|---|---|---|---|
| `tagSize` (AprilGrid) | 0.088 m | **ASSUMIDO, não verificado** | Define a escala métrica. Erro de 1% → 1% em todas as translações **e** 1% na `velocity_scale` |
| `tagSpacing` | 0.3 | assumido | Afeta a detecção, não a escala |
| Baseline estéreo | 0.1211 m | **FIXO** (ZED, submersa) | Não re-otimizado; erro vira viés de pose |
| `sound_speed` (A50) | 1500 m/s | **ASSUMIDO** (default do A50) | Escala a velocidade medida: `v_reportada ∝ c_assumido` |

### ⚠️ Degenerescência crítica: `tagSize` ↔ `sound_speed`

Os dois caem **no mesmo parâmetro** (`velocity_scale`) e o otimizador **não consegue separá-los**:

```
tagSize errado  →  escala da trajetória errada  →  velocidade da spline errada  ─┐
                                                                                 ├─→ velocity_scale
sound_speed errado  →  velocidade medida pelo DVL errada  ──────────────────────┘
```

Ambos os efeitos são da mesma ordem (~1%): água doce a 20 °C tem c ≈ 1482 m/s contra os 1500 assumidos
(~1.2% de erro), e um alvo impresso que incha submerso erra facilmente 0.5–1%.

**Consequência:** a `velocity_scale` só é interpretável como "erro de velocidade do som" **se o `tagSize`
for medido com precisão**. Caso contrário ela é uma mistura inseparável dos dois.
**Ação (barata, alto valor):** medir o AprilGrid com paquímetro — de preferência **molhado**, já que é
nessa condição que ele é usado — e conferir o `sound_speed` configurado no A50.

---

## 2. Modelo óptico — refração de porta plana

| Parâmetro | Valor atual | Status | Impacto |
|---|---|---|---|
| Intrínsecos (`fx,fy,cx,cy`) | ZED submersa, 2026-05-07 | **FIXO**, nunca otimizado | Erro vira viés de pose/extrínseco |
| `k3` | **descartado** (D7) | erro conhecido | Reprojeção 1.3–1.5 px; k3 era grande (−2.64 / −4.12) |
| Modelo de distorção | `radtan` (4 params) | assumido | Não representa refração de porta plana |
| Validade temporal | maio → bags de julho | **assumido** | ~3 meses; se houve remontagem, mudou |

**O ponto físico:** atrás de uma porta plana, a refração não é um pinhole. Ela multiplica a distância
focal efetiva por ~1.33 (isso a calibração submersa já absorve) **mas também depende da distância do
objeto** — um modelo pinhole+radtan ajustado a um alcance fica enviesado em outro. Num tanque, com o
alvo a ~0.5–2 m, isso é um erro sistemático residual que o `radtan` não consegue representar,
independentemente de quantas imagens você colete.

Reforçando: como o Kalibr **não otimiza intrínsecos** nesta ferramenta, esse erro não tem para onde ir
a não ser contaminar o extrínseco.

---

## 3. IMU

| Parâmetro | Valor atual | Status | Impacto |
|---|---|---|---|
| `accelerometer_noise_density` | 0.002 m/s²/√Hz | **PROVISÓRIO** (datasheet, D3) | Peso relativo inercial × visual |
| `gyroscope_noise_density` | 0.0002 rad/s/√Hz | **PROVISÓRIO** | idem |
| `accelerometer_random_walk` | 4e-5 | **PROVISÓRIO** | Rigidez da spline de bias |
| `gyroscope_random_walk` | 2e-6 | **PROVISÓRIO** | idem |
| `update_rate` | 200 Hz | assumido (≈197 medido) | Discretização do ruído; ~1.5%, desprezível |
| `model` | `calibrated` | **ASSUMIDO** | Assume eixos ortogonais, **sem erro de escala nem desalinhamento** |
| ZED IMU | fora | decisão (D5) | Não é fonte de erro; é oportunidade não usada |

O ruído da IMU não muda o *ponto* de convergência tanto quanto muda a **ponderação** entre resíduos
visuais e inerciais — subestimar o ruído faz o otimizador confiar demais na IMU. Os valores atuais são
chutes conservadores de datasheet.

**Allan variance não precisa de piscina:** é um bag longo (idealmente 2–12 h) com a IMU **parada, em
bancada, em ambiente termicamente estável**. É a coleta de melhor custo-benefício da lista.

---

## 4. Tempo e sincronização

| Parâmetro | Status | Impacto |
|---|---|---|
| Timestamp da imagem = instante de exposição | **ASSUMIDO** | Se for o instante de recepção, vira offset variável |
| Sincronização por hardware ZED ↔ MicroStrain | **provavelmente inexistente** | Evidência: timeshift assenta com τ≈9 min (D9) |
| Timestamp do DVL | **usa `header.stamp`** | ⚠️ ver abaixo |
| `--timeoffset-padding 0.1` | parâmetro do solver | Não físico; só precisa ser folgado |

### 🔄 NÃO é bug: o extrator do DVL usa `header.stamp` — e está certo

Registrei antes que usar `header.stamp` em vez do `time_of_validity` (exigido pela R5 da spec
`dvl-calibration`) era um bug. **A medição desmente.** O `time_of_validity` do A50 está no relógio
**interno** dele, sem sincronia com o sistema:

```
time_of_validity[0] = 1716816236492214 us -> 2024-05-27
header.stamp[0]     = 1789742354.741 s    -> 2026-09-18
diferença de época  = 72.926.118 s = 2.32 anos
```

Usá-lo como timestamp absoluto seria catastrófico. **O uso atual está correto.**

O refinamento que *faria* sentido é usar o `tov` só para o espaçamento **relativo**, ancorado à
época do header: o jitter do intervalo cai de **20.6 ms → 17.5 ms** (std), com o offset
`header − tov` praticamente constante (std 8.9 ms). Melhoria modesta, opcional.

### O acoplamento timeshift ↔ lever-arm (já observado)

Erro de sincronização é absorvido pela **translação** (a rotação é insensível). Foi exatamente o que se
viu nos 4 bags: rotação estável em 1.52°±0.22° enquanto a translação em x dançava 5 cm — e excluir o
bag 01 (relógio não assentado) derrubou essa dispersão para 0.7 cm (D9).

---

## 5. DVL

| Parâmetro | Valor atual | Status | Impacto |
|---|---|---|---|
| `T_dvl_imu` inicial | **identidade** | chute | ⚠️ risco de mínimo local se a rotação real for ~180° |
| `velocity_noise_density` | 0.02 (fallback) | assumido | Peso dos resíduos quando a covariância do A50 falta |
| `max_fom` | 1.0 | **assumido** | Gating; nunca confrontado com a distribuição real |
| `max_speed` | 1.5 m/s | assumido | idem |
| `min/max_altitude` | 0.1 / 50 m | assumido | idem |
| Geometria dos feixes (α, β) | de fábrica | fora de escopo (D3) | A velocidade 3D do A50 já vem dela |
| Bottom-lock válido no tanque | **assumido** | `NEEDS CLARIFICATION` da spec | Se o fundo estiver fora de alcance, não há dado |

**Sobre o chute inicial:** o xacro comenta uma rotação de "180° em Z" na Microstrain, mas o joint tem
`rpy="0 0 0"` (ambiguidade registrada no `scratch.md`). Começar de identidade quando a verdade pode ser
uma rotação de 180° é o cenário clássico de convergência para mínimo local. Vale rodar também a partir
do chute do xacro e comparar o resíduo final.

---

## 6. Geometria de referência (CAD/xacro)

| Item | Status |
|---|---|
| Orientação da Microstrain (`rpy=0 0 0`) | ✅ **confere** — 1.52°±0.22° nos 4 bags |
| Posição da Microstrain | ❌ **erra ~13 cm**, dominado por x (D10) |
| `zed_node_camera_link` = centro do corpo? | **ASSUMIDO** (é o que o `zed_macro` supõe) |
| Lever-arm nominal Microstrain→DVL | `[0.094, 0, -0.129]` — **herda o erro acima** |

Como a calibração mede posição **relativa**, ela não distingue se quem está deslocado é a ZED ou a
Microstrain. Precisa de olho na peça.

---

## 7. Ausência de incerteza formal

`--recover-covariance` nunca foi usado. Não temos σ por parâmetro — a dispersão cross-bag foi usada como
substituto, o que só funciona com ≥3 bags e mistura erro de estimação com variação entre gravações.
Ativar a flag é grátis em coleta (custa só tempo de CPU) e diria, por bag, quanto o próprio otimizador
acredita em cada número.

---

## Plano de ação, por custo

### A. Custo zero — medição física / conferência (fazer primeiro)
1. **Medir o `tagSize` do AprilGrid com paquímetro, molhado.** Resolve a degenerescência do §1.
2. **Conferir o `sound_speed` configurado no A50** durante as gravações.
3. **Conferir no CAD** a cota da Microstrain em x (xacro: −0.09439; dados: ≈ +0.02) e o que
   `zed_node_camera_link` representa.
4. **Confirmar se há sincronização por hardware** ZED ↔ MicroStrain.

### B. Reprocessamento dos bags que já existem (sem piscina)
5. **Calibrar os intrínsecos com o próprio Kalibr** (`kalibr_calibrate_cameras`, `pinhole-radtan` e
   `pinhole-equi`) nos bags atuais e comparar com a ZED-4params. Resolve o k3 descartado e ajusta o
   modelo às imagens reais, submersas.
6. **Corrigir `ros2_dvl_to_csv.py` para usar `time_of_validity`** (R5).
7. **Rodar com `--recover-covariance`** para ter incerteza formal.
8. **Testar `--imu-models scale-misalignment`** e ver se o resíduo cai.
9. **Testar `--recompute-camera-chain-extrinsics`** e ver se o baseline se move.
10. **Ajustar o gating do DVL** olhando a distribuição real dos CSVs já extraídos.
11. **Rodar o DVL a partir do chute do xacro**, além da identidade, e comparar resíduos.

### C. Coleta nova em bancada (sem piscina)
12. **Allan variance das duas IMUs** — 2–12 h paradas, ambiente estável.

### D. Coleta nova na piscina (a mais cara — fazer só depois de A e B)
13. **Warm-up de 15–20 min** antes de gravar (D9).
14. **Excitação rotacional agressiva** nos 3 eixos — é o que torna o lever-arm observável.
15. **Alvo a várias distâncias** dentro da mesma sequência, para expor (e quantificar) a dependência de
    distância da refração.
16. **Confirmar bottom-lock** simultâneo à visão do alvo (pré-condição da CA5 do DVL).

> **Ordem importa.** Coletar mais dados antes de fechar A e B só produz mais bags com os mesmos vieses
> sistemáticos. Nenhuma quantidade de dados corrige um `tagSize` errado.

---

## Atualização — o que o dataset v3 (fora d'água) confirmou e derrubou

> Fonte: `.ai/specs/calibracao-cam-imu-ar/resultados.md`. Dois bags gravados **fora d'água**
> olhando o alvo, justamente para separar os efeitos da água dos do método.

### ✅ CONFIRMADO — a refração era o problema dominante

O mesmo código, com os mesmos sensores e as mesmas flags, converge de forma radicalmente diferente:

| | v2 submerso | v3 ar (bag raw) |
|---|---|---|
| iterações | 17+ sem convergir | **5** |
| `lambda` final | **33 209** | **0.12** |
| reprojeção | — | **0.67 px** |

Cinco ordens de grandeza de diferença no condicionamento. A seção "Modelo óptico" acima previa que
a refração de porta plana não é representável por `radtan` e é dependente da distância; a v3
confirma que esse era o fator dominante, não um detalhe.

### ✅ CONFIRMADO — excitação rotacional escala com ω², e importa

Subir de 3.8–7.6 °/s (v2) para ~11 °/s (v3) e tornar a excitação **isotrópica** (razão forte/fraca
de 2.4× para 1.4×) teve efeito mensurável além do lever-arm: o **prior de timeshift**, que no v2
saía inconsistente entre as duas câmeras (95 vs 130 ms, com o estéreo sincronizado a 1 ms), passou
a sair **idêntico** (−15.000 ms). O prior vem de correlação cruzada de ‖ω‖, cujo pico achata com
rotação suave.

### 🔄 DERRUBADO (parcialmente) — "usar intrínsecos de fábrica custa reprojeção"

O registro anterior atribuía a reprojeção alta ao uso dos intrínsecos de fábrica com o k3 dropado.
A v3 mostra que o problema não era **usar** o de fábrica, era **truncar** o modelo:

- Truncar o `rational_polynomial` (8 coef) para `radtan` (4) dá **989% de erro na borda** —
  numerador e denominador quase se cancelam, e jogar o denominador fora faz o polinômio explodir.
- **Ajustar** um `radtan`-4 que reproduza o mapeamento racional (`scripts/rational_to_radtan.py`,
  resíduo 0.16 px) dá resultado **equivalente a recalibrar com o Kalibr**: diferença de
  **0.001–0.007 px** na reprojeção final.

> **Recomendação revisada:** usar os intrínsecos de fábrica **com o ajuste racional→radtan**.
> Rodar `kalibr_calibrate_cameras` custa ~1 h por bag e não paga.

### 🔴 NOVO — o stream retificado do SDK da ZED não serve para calibração

O `camera_info` do tópico `*/gray/rect/` **não descreve** as imagens que ele acompanha:

| | declarado | estimado | significância |
|---|---|---|---|
| `cy` | 357.195 | 364.274 ± 0.65 | **10.9σ** (~8 px) |
| `k1` | 0.0 | 0.0120 ± 0.0011 | **11σ** |
| `cy` cam0 vs cam1 | devem ser iguais | 364.27 vs 366.51 | 2.5σ |

A última linha é contradição de definição: num par retificado o `cy` das duas câmeras **é** igual,
por construção. Não é.

**E corrigir os intrínsecos NÃO resolveu:** a execução com os valores estimados ainda ficou com
`lambda` 4 918. A causa raiz do mal-condicionamento do bag retificado **segue não estabelecida**.

**Regra prática:** gravar em **raw** e deixar o Kalibr lidar com a distorção.

### 🔴 NOVO — modo de falha silenciosa: reprojeção boa com geometria absurda

A execução `rect__zed-fabrica` produziu reprojeção de **0.596 px** (ótima) e um lever-arm
câmera↔IMU-da-ZED de **429.5 mm** — as duas ficam no mesmo corpo, a ~23 mm.

> **A reprojeção não detecta este erro.** Os indicadores que detectam: o `lambda` final do
> Levenberg-Marquardt, o resíduo inercial **normalizado** (>1 é bandeira) e a plausibilidade
> física. Incluir os três em qualquer checagem de sanidade.

Em contraste, o **baseline estéreo** foi recuperado corretamente em **todas** as execuções
(0.1197–0.1198 m contra 0.1201 de fábrica), inclusive nessa. A geometria estéreo é robusta; o que
degrada é o braço câmera↔IMU.

### 🔴 NOVO — inflar o ruído da IMU tem custo, não só benefício

A orientação anterior era "inflar é o erro seguro". A v3 mostra o outro lado: com o ruído da IMU da
ZED inflado ~4.5× demais (resíduo normalizado de acelerômetro em **0.222**), o otimizador sub-pesa
os termos inerciais e deixa a câmera dominar. Como **é a IMU que observa o lever-arm**, isso o
deixa mal determinado — o lever-arm da ZED saiu em 2× o nominal do `zed_macro`.

> Inflar continua mais seguro que subestimar, mas **inflar demais degrada justamente o parâmetro de
> interesse**. O resíduo normalizado é o termômetro: deve ficar perto de 1.

### 🔴 NOVO — o container Docker sem limites derruba a máquina

O `kalibr_zed` rodava com `Memory: 0` e `NanoCpus: 0` (sem teto). Durante a bateria noturna a
máquina **travou e reiniciou**. Medição posterior: a fase de extração do alvo usa ~5 GB e
**1373% de CPU** — sem teto, saturava os 20 cores e deixava o servidor X sem fatia (o log registrou
`your system is too slow` 30 s antes do congelamento).

**Correção:** `docker update --memory 9g --memory-swap 9g --cpus 14 kalibr_zed`.
`MemorySwap == Memory` dá swap zero ao container: se estourar, o kernel mata o processo dele em vez
de arrastar a máquina inteira para thrashing.
