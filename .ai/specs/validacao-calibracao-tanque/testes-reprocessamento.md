# Testes de reprocessamento (câmera e câmera-IMU) — dados existentes

> Bateria de testes sobre os 4 bags já gravados, para **melhorar os resultados sem coletar dados novos**.
> Premissas que motivam cada teste: `.ai/docs/domain/premissas-e-fontes-de-erro.md`.
> Resultados do cam-IMU original: `data/output/analise-crossbag-camimu.txt`.
>
> **Legenda:** ✅ concluído · ⏳ rodando · ⏸️ não iniciado

## Matriz de testes

| # | Teste | Status | Veredito |
|---|---|---|---|
| 1 | Excitação rotacional / observabilidade do lever-arm | ✅ | **Excitação NÃO é o gargalo** — mas é a direção certa |
| 2 | Qualidade dos dados (cobertura + continuidade) | ✅ | 🔴 **Gaps graves de gravação em todos os bags** |
| 3 | Recalibrar nas janelas contínuas (consequência do #2) | ⚠️ | 2 de 4 **não convergiram**; teste confunde "sem gaps" com "menos dados" |
| — | *(bônus)* prior de timeshift corrompido pelos gaps | ✅ | 🔴 **defeito no `IccSensors.py:253`** — anotado, não corrigido |
| — | *(bônus)* Kalibr sobrescreve saídas ao lado do bag | ✅ | 🔴 armadilha operacional — procedimento definido |
| 4 | Split-half (bag 04) — metades iguais | ⏸️ | — |
| 5 | Split-half (bag 01) — testa D9 | ⏸️ | — |
| 6 | `--no-time-calibration` — mede acoplamento timeshift↔lever-arm | ⏸️ | — |
| 7 | `--recover-covariance` | ⏸️ | — |
| 8 | Intrínsecos com Kalibr (`radtan` × `equi`) | ⏸️ | — |
| 9 | Re-rodar cam-IMU com intrínsecos novos | ⏸️ | depende do #8 |
| 10 | `--recompute-camera-chain-extrinsics` | ⏸️ | — |
| 11 | ZED IMU como 2ª IMU | ⏸️ | — |
| 12 | `--imu-models scale-misalignment` | ⏸️ | — |

---

## Teste 1 — Observabilidade do lever-arm ✅

**Script:** `scripts/analyze_excitation.py` (roda no container).

**Método.** Câmera e IMU são rígidas, então
`a_cam = a_imu + R·([ω̇]ₓ + [ω]ₓ[ω]ₓ)·r`. O lever-arm `r` **só** é observável através de
`S = [ω̇]ₓ + [ω]ₓ[ω]ₓ`. Acumulando `M = Σ SᵀS`, os autovalores de `M` dão a informação por direção
e o autovetor mais fraco é a direção **cega** da calibração. Reportamos duas versões: **centrípeta**
(só `[ω]ₓ[ω]ₓ`, não precisa derivar o giro — imune a ruído de diferenciação) e **completa**
(com `ω̇` suavizado — mais fiel, porém otimista se ruído vazar).

**Resultados.**

| bag | giro RMS (rad/s) | sensib. centrípeta (m/s²)/m | razão forte/fraca | direção mais fraca |
|---|---|---|---|---|
| 01 | [0.161 0.096 0.140] | [0.15 0.13 0.08] | 1.8× | **x** do `base_link` (99%) |
| 02 | [0.150 0.097 0.135] | [0.15 0.14 0.07] | 2.1× | **x** (97%) |
| 03 | [0.102 0.106 0.123] | [0.09 0.08 0.06] | 1.5× | **x** (94%) |
| 04 | [0.133 0.101 0.124] | [0.12 0.12 0.05] | 2.4× | **x** (97%) |

**Achado 1.A — a direção cega é o eixo x, exatamente onde a dispersão apareceu.** Em 4 de 4 bags a
direção de pior observabilidade é o x do `base_link` (94–99% alinhada), e x é o eixo com a maior
dispersão entre bags (2.1 cm, contra 0.9 cm em z). Qualitativamente, casa.

**Achado 1.B — mas a excitação NÃO é o fator limitante.** A razão forte/fraca é de apenas 1.5–2.4×
(anisotropia leve, não degenerescência), e o **piso teórico de incerteza** do lever-arm é de
**0.9–2.3 mm** por direção — contra os **21 mm** de dispersão observada em x. Ou seja: há informação
suficiente nos dados para 10× mais precisão do que estamos obtendo.

> **Conclusão do teste 1: o erro é SISTEMÁTICO, não falta de excitação.** Reprocessar não vai ajudar
> pelo lado da excitação; o que sobra é timing, modelo óptico e qualidade dos dados.

**Achado 1.C — por que o resultado é tão frágil a erro sistemático.** Com `|ω|`≈0.15 rad/s e
`r`≈0.19 m, o sinal centrípeto vale ~0.01–0.03 m/s² por amostra, enquanto o ruído discreto do
acelerômetro é ~0.028 m/s² — ou seja, **o sinal do lever-arm está NO nível do ruído por amostra**.
A recuperação só funciona por média sobre ~50 mil amostras. Erro aleatório cai com √N; **erro
sistemático não cai**. Daí o resultado ser dominado por vieses.

**Consequência para coleta futura:** o sinal cresce com **ω²**. Dobrar a taxa de rotação **quadruplica**
o sinal do lever-arm. Sair dos ~7–9 °/s RMS atuais para ~20 °/s RMS levantaria o sinal ~5× acima do
piso sistemático. É a recomendação de maior impacto para a próxima captura.

---

## Teste 2 — Qualidade dos dados ✅

**Método.** Leitura do relatório já gerado (`piscina_calib_04-report-imucam.pdf`, pág. 6) + varredura
direta dos bags ROS 1 procurando descontinuidades (`Δt > 2.5×` a mediana).

### 🔴 Achado 2.A — gaps de gravação graves, simultâneos em câmera e IMU

| bag | taxa IMU nominal | taxa efetiva | nº de gaps | tempo perdido | maior gap | **maior janela contínua** |
|---|---|---|---|---|---|---|
| 01 | 200 Hz | 197.4 Hz | 1 | 3.0 s | 3.0 s | **185.4 s (81%)** |
| 02 | 200 Hz | 186.4 Hz | 7 | 17.0 s | 5.1 s | 79.5 s (32%) |
| 03 | 200 Hz | 152.1 Hz | 4 | 69.2 s | **40.8 s** | 69.3 s (24%) |
| 04 | 200 Hz | 167.8 Hz | **21** | 51.4 s | 9.3 s | 113.1 s (35%) |

**Os gaps são simultâneos nos dois sensores** (bag 03: 69.2 s no IMU vs. 69.3 s na câmera; contagens
idênticas). Sensores independentes não falham em sincronia — **quem travava era o sistema de gravação**,
não os sensores. Provável causa: vazão de disco / contenção de CPU no gravador ROS 2.

**Por que isso corrompe a calibração:** a B-spline de pose tem ~100 nós/s. Num gap de 40.8 s (bag 03)
são ~4000 nós **sem nenhuma observação** — trecho totalmente livre, que o otimizador preenche com o que
quiser, contaminando biases e trajetória vizinhos. O Kalibr **não detecta nem avisa** sobre isso.

### Achado 2.B — a degradação piora ao longo da sessão

Gaps por bag: 1 → 7 → 4 → 21; tempo perdido: 3.0 → 17.0 → 69.2 → 51.4 s. O gravador foi piorando
durante a sessão.

> ⚠️ **Isso enfraquece a explicação térmica do D9.** O timeshift cair monotonicamente (16.8 → 4.3 ms)
> continua sendo fato, mas agora há uma segunda hipótese com evidência independente: um gravador sob
> estresse crescente também produz latência de timestamp variável. A causa do D9 volta a ser **questão
> aberta** — o teste 6 (`--no-time-calibration`) e o teste 5 (split-half do bag 01) ajudam a decidir.

### 🔄 Achado 2.C — a escolha do "melhor bag" se inverte

A recomendação anterior (bag 04, por ter menor reprojeção e relógio assentado) **não considerava
continuidade**. Por continuidade, o **bag 01 é de longe o melhor**: 185 s contínuos (81%) contra 113 s
(35%) do bag 04.

Fica um dilema real, ainda sem resposta: **bag 01 = dados contínuos + relógio não assentado**;
**bag 04 = relógio assentado + dados fragmentados**. O teste 3 ataca exatamente isso ao remover a
variável "gaps" de todos.

### Achado 2.D — trajetória e alcance

Do relatório (pág. 5): a trajetória cobre ~3 m × 2.5 m × 1.7 m, com o alvo a **1.7–3.4 m**. A variação
de ~2× em distância é relevante para a refração de porta plana (cujo erro é dependente de distância) e
é boa notícia para o teste 8 — há diversidade de alcance para ajustar intrínsecos.

---

## Teste 3 — Recalibração nas janelas contínuas ⏳

**Motivação:** consequência direta do achado 2.A. Remove a variável "gaps" antes de investir nos testes
caros de intrínsecos.

**Janelas usadas** (`--bag-from-to`, relativo ao início do bag — ver `ImuDatasetReader.py:93`):

| bag | janela | duração |
|---|---|---|
| 01 | 1.0 → 184.0 | 183 s |
| 02 | 1.5 → 79.0 | 77.5 s |
| 03 | 1.0 → 68.0 | 67 s |
| 04 | 1.5 → 112.0 | 110.5 s |

Saídas em `data/output/clean/`. Comparar com `python3 scripts/compare_calibrations.py --outdir data/output/clean`.

### Resultado parcial (2 de 4 falharam)

| bag | janela | resultado |
|---|---|---|
| 01 | 183 s | ❌ **falhou** — `J` travado em 2.58e+06, LM saturando λ até 1e8, sem descida |
| 02 | 77.5 s | ✅ convergiu |
| 03 | 67 s | ❌ **falhou** — `J` = 9.56e+08 |
| 04 | 110.5 s | (ver abaixo) |

Não há correlação com o tamanho da janela: a **maior** (bag 01) falhou e uma das menores (bag 02)
convergiu.

**Bag 02, completo × janela limpa** (mesma gravação, subconjunto contíguo):

| | full (249 s) | limpa (77 s) | Δ |
|---|---|---|---|
| `t_ic` x | 0.0816 | 0.0819 | **0.2 mm** |
| `t_ic` y | 0.1092 | 0.1141 | 4.9 mm |
| `t_ic` z | 0.0218 | 0.0433 | **21.5 mm** |
| reprojeção cam0 | 1.39 | 1.36 | ~igual |
| accel | 0.0272 | 0.0229 | melhor |
| giro | 0.0044 | 0.0060 | pior |

**z mudou 2.1 cm dentro da MESMA gravação** — a mesma ordem da dispersão *entre* bags. Ou seja, a
variação intra-bag ≈ inter-bag, o que aponta para instabilidade de estimação / erro de modelo, e não
para diferenças reais entre gravações.

> ⚠️ **Defeito de desenho deste teste:** a janela limpa do bag 02 usa só 1/3 dos dados, então
> "remoção de gaps" e "menos dados" ficam **confundidos**. O split-half propriamente dito (metades de
> tamanho igual, ambas contendo gaps) separaria os dois efeitos. Refazer assim.

---

## Achado transversal — o prior de timeshift é corrompido pelos gaps 🔴

Descoberto ao investigar as falhas do teste 3. Em `IccSensors.py:248-254`:

```python
corr = np.correlate(omega_predicted_norm, omega_measured_norm, "full")
discrete_shift = corr.argmax() - (np.size(omega_measured_norm) - 1)
dT = np.mean(np.diff(times))      # <-- MEDIA dos intervalos da IMU
shift = -discrete_shift * dT
```

**Dois defeitos empilhados quando há gaps:**

1. **Escala errada.** `dT` usa a **média** dos intervalos, que os gaps inflam:

   | bag | dT mediano | dT médio | inflação |
   |---|---|---|---|
   | 01 | 5.000 ms | 5.066 ms | 1.3% |
   | 02 | 5.000 ms | 5.365 ms | 7.3% |
   | 03 | 5.000 ms | 6.575 ms | **31.5%** |
   | 04 | 5.000 ms | 5.960 ms | 19.2% |

2. **Pior: `np.correlate` pressupõe amostragem uniforme.** Com buracos de até 40 s, o índice de lag
   não corresponde a um deslocamento temporal consistente — a correlação em si fica corrompida.

**Evidência:** priors de 30–100 ms contra valores convergidos de 4–17 ms (fator 3–20×). Nas janelas
limpas os priors saíram erráticos (60 / 100 / 90 ms para bags 01/02/03), cada subconjunto com um
perfil de gaps diferente.

**Correção possível (NÃO aplicada — decisão do usuário de não mexer no código agora):**
`np.mean` → `np.median` na linha 253 resolve o defeito 1 em uma linha. O defeito 2 exigiria reamostrar
num grid uniforme antes de correlacionar.

> **Não fecha o caso do D9.** Os priors (55.7 / ? / 78.9 / 29.8 ms) **não** seguem a tendência
> monotônica dos valores finais (16.8 / 9.1 / 6.0 / 4.3 ms), e o valor final é otimizado, não o prior.
> Isso explica a instabilidade e as falhas de convergência; a queda monotônica do timeshift segue
> **sem causa estabelecida**.

---

## Armadilha operacional — o Kalibr grava ao lado do BAG 🔴

`kalibr_calibrate_imu_camera` deriva o caminho de saída do caminho **do bag**, ignorando o `cwd`, e
**não há flag** para mudar isso. Rodar uma variante (janela, modelo, flags) sobre o mesmo bag
**sobrescreve silenciosamente** o resultado anterior.

**Aconteceu nesta bateria:** o run da janela limpa do bag 02 sobrescreveu o resultado do bag completo
(recuperado por re-run; os números estavam preservados em `analise-crossbag-camimu.txt`).

**Procedimento obrigatório:** mover as saídas (`*-camchain-imucam.yaml`, `*-imu.yaml`,
`*-results-imucam.txt`, `*-report-imucam.pdf`) para o diretório da variante **imediatamente após cada
run**. Organização adotada:

- `data/output/` — resultado corrente (bag completo)
- `data/output/full/` — backup dos resultados de bag completo
- `data/output/clean/` — resultados das janelas contínuas
