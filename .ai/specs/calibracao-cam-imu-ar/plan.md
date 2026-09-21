# Plano — Calibração câmera-IMU fora d'água (v3)

> Micro-tarefas verificáveis. O **porquê** das escolhas está em `decisions.md` (D1–D8); os
> requisitos em `spec.md` (R1–R10).
>
> **Legenda:** 💻 host · 🐳 container `kalibr_zed` · 🔴 bloqueado por insumo do usuário
> **Status:** não iniciado.

**Nomenclatura das execuções:** `v3_<bag>__<imu>-<intrinsecos>`, com `<bag>` ∈ {`raw`, `rect`},
`<imu>` ∈ {`micro`, `zed`}, `<intrinsecos>` ∈ {`fabrica`, `kalibr`}. Saídas em
`data/output/runs/<nome>/` (D8).

---

## Fase 0 — Diagnóstico barato e preparo (sem calibração) — ~40 min

- [ ] **0.1 💻 Continuidade dos 2 bags.** Gaps por tópico (imagem L/R, `/imu/data`,
      `/zed/…/imu/data`), como já feito para v1/v2.
  - **Verificação:** tabela com nº de gaps, tempo perdido e maior gap. Explicar a diferença de
    contagem L/R já observada (4245/4294 no raw, 3251/3325 no rect).
- [ ] **0.2 💻 Excitação rotacional** nos 2 bags (`analyze_excitation.py`, adaptado para ler mcap) — D7.
  - **Verificação:** giro RMS por eixo e sensibilidade centrípeta, comparados com v1 (5.9–9.2 °/s) e
    v2 (3.8–7.6 °/s). Registrar a expectativa de precisão do lever-arm **antes** de calibrar.
- [ ] **0.3 💻 Extrair os intrínsecos de fábrica** do `camera_info` de cada bag: `K`, `D`, `P` e o
      `distortion_model`, para as duas câmeras.
  - **Verificação:** valores anotados; conferir que `fx` fica na casa de ~957 (ar) e **não** ~1372
    (submerso). Resolução 1280×720.
- [ ] **0.4 💻 Converter os 2 bags ROS 2 → ROS 1** (`ros2_to_ros1_kalibr.py`, `--zero-start`).
      As imagens já são cinza; confirmar que o conversor lida com o encoding sem reconverter.
  - **Verificação:** contagens no bag ROS 1 idênticas às do mcap; `T0_NS` anotado por bag.
      Saída esperada ~4 GB (raw) e ~3 GB (rect).

## Fase 1 — Piloto: uma execução para validar o pipeline (D1) — ~40 min

- [ ] **1.1 💻 Montar `camchain` de fábrica do bag raw** (`K` + `D`, `T_cn_cnm1` real) — D3.
  - **Verificação:** parse OK; `rostopic` batem com o bag ROS 1; resolução confere.
- [ ] **1.2 💻 Montar `imu.yaml` da Microstrain** (`/imu/data`, 200 Hz, ruído provisório D6).
  - **Verificação:** parse OK.
- [ ] **1.3 🐳 Executar `v3_raw__micro-fabrica`** (`--timeoffset-padding 0.3`, `MPLBACKEND=Agg`).
  - **Verificação — este é o CA4:** a otimização converge de forma **saudável**?
    - `lambda` final baixo (ordem 0.1–1, como os 0.18 do run saudável da v1) e **não** milhares;
    - parou por **tolerância**, não por `--max-iter`;
    - resíduos normalizados ~1;
    - sem `Spline Coefficient Buffer Exceeded`.
  - **🚦 PORTÃO:** se falhar, **parar a matriz** e diagnosticar (o pipeline não funciona nem no ar —
    achado grande, muda a conclusão da spec). Se passar, seguir.

## Fase 2 — Matriz com intrínsecos de fábrica (R3a) — ~2 h

- [ ] **2.1 💻 `imu.yaml` da ZED** (`/zed/zed_node/imu/data`, ~99 Hz, ruído provisório D6).
- [ ] **2.2 💻 `camchain` de fábrica do bag rect** (intrínsecos de `P`, distorção **zerada**,
      `T_cn_cnm1` com rotação identidade) — D3.
  - **Verificação:** parse OK; conferir que os coeficientes de distorção estão zerados.
- [ ] **2.3 🐳 `v3_raw__zed-fabrica`**
- [ ] **2.4 🐳 `v3_rect__micro-fabrica`**
- [ ] **2.5 🐳 `v3_rect__zed-fabrica`**
  - **Verificação (2.3–2.5):** cada uma conclui **ou** tem o motivo da falha registrado no `cmd.txt`
    (R6). Reprojeção, erros de giro/accel, gravidade, timeshift e comportamento da convergência
    anotados por execução (R7).

## Fase 3 — Intrínsecos estimados pelo Kalibr (R3b) — ~2 h

- [ ] **3.1 🐳 `kalibr_calibrate_cameras` no bag raw**, `pinhole-radtan` (usar `--bag-freq` para
      subamostrar se necessário).
  - **Verificação:** conclui; reprojeção reportada; comparar com a de fábrica.
- [ ] **3.2 🐳 `kalibr_calibrate_cameras` no bag rect**, `pinhole-radtan`.
  - **Verificação:** os coeficientes de distorção estimados devem sair **próximos de zero** — se não
    saírem, a retificação do SDK está suspeita. É um teste da retificação, não só dos intrínsecos.
- [ ] **3.3 💻 Sanidade contra o SDK:** conferir que os `fx`/`fy` estimados ficam na mesma ordem dos
      de fábrica (~957) e que o ponto principal não migra absurdamente.
  - **Verificação:** desvios explicados ou registrados como achado.

## Fase 4 — Matriz com intrínsecos do Kalibr — ~2 h

- [ ] **4.1 🐳 `v3_raw__micro-kalibr`**
- [ ] **4.2 🐳 `v3_raw__zed-kalibr`**
- [ ] **4.3 🐳 `v3_rect__micro-kalibr`**
- [ ] **4.4 🐳 `v3_rect__zed-kalibr`**
  - **Verificação:** mesma da Fase 2.

## Fase 5 — Comparação e conclusão — ~1 h

- [ ] **5.1 💻 Comparação cross-bag** (`compare_calibrations.py --outdir data/output/runs`), agrupando
      por IMU e por fonte de intrínsecos (R5/CA3).
  - **Verificação:** tabela com `T_cam_imu` e `timeshift` por execução, e a dispersão **entre os dois
    bags** para cada combinação.
- [ ] **5.2 💻 Fábrica × Kalibr** (CA5): a reprojeção cai o suficiente para justificar recalibrar? A
      dispersão entre bags aperta?
- [ ] **5.3 💻 Verificação geométrica independente** (R8/CA6): confrontar a pose câmera↔IMU-da-ZED
      estimada com o nominal do `zed_macro` (`[-0.002, -0.023, -0.002]` do frame da câmera esquerda).
  - **Verificação:** diferença em cm e graus. **É a primeira referência confiável** do projeto — não
    depende do xacro do ROV.
- [ ] **5.4 💻 Raw × rect** (D3): os dois bags convergem para o mesmo extrínseco? Divergência aponta
      erro em um dos camchains.
- [ ] **5.5 💻 Conclusão explícita** (R10/CA7) em `resultados.md`: o pipeline funciona fora d'água? O
      que isso implica para o caso submerso? Fixar as tolerâncias de "consistente" (pergunta aberta
      da spec) com base nos números obtidos.

## Fase 6 — Fechamento

- [ ] **6.1 💻 Atualizar** `.ai/docs/domain/premissas-e-fontes-de-erro.md` com o que a v3 confirmou ou
      derrubou (sobretudo sobre refração e intrínsecos).
- [ ] **6.2 💻 Registrar a decisão de seguir (ou não) para o DVL**, com o critério que a sustenta.

---

## Riscos

- **O piloto falhar como o v2 falhou** (`lambda` explodindo). Seria o achado mais importante da spec:
  o problema não é a água. Mitigado pelo portão da 1.3, que impede gastar ~8 h antes de saber disso.
- **Excitação baixa de novo** (D7): translação imprecisa. Não invalida o teste de convergência, mas
  limita a conclusão sobre o lever-arm. Medido na 0.2, antes de qualquer calibração.
- **`camchain` do bag rect montado errado** (D3): erro **silencioso** — converge e dá número errado.
  Mitigado pela 3.2 (distorção estimada deve dar ~0) e pela 5.4 (raw × rect devem concordar).
- **Frames faltando na câmera esquerda** (49 no raw, 74 no rect): investigar na 0.1; se for
  descasamento estéreo, afeta a triangulação.
- **`tagSize` não medido:** os valores absolutos de translação herdam o erro de escala. Não afeta a
  comparação entre bags (mesmo alvo nos dois).
- **Tempo de máquina:** a matriz completa são ~8 h. A ordem entrega resultado útil já na Fase 2, antes
  de as fases caras começarem.
