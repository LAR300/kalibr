# Plano — Calibração câmera-IMU fora d'água (v3)

> Micro-tarefas verificáveis. O **porquê** das escolhas está em `decisions.md` (D1–D9); os
> requisitos em `spec.md` (R1–R10). **Resultados consolidados em `resultados.md`.**
>
> **Legenda:** 💻 host · 🐳 container `kalibr_zed` · 🔴 bloqueado por insumo do usuário
>
> **Status:** Fases 0–4 ✅ (10 de 11 execuções; a 11ª em re-execução). Fase 5 em andamento.
>
> **↔️ Reordenação (durante a execução):** a Fase 3 foi promovida para antes do fim da Fase 2,
> porque o `rect__micro-fabrica` mostrou-se mal-condicionado e a suspeita era o camchain. Na
> prática a cadeia noturna acabou executando tudo; a reordenação virou irrelevante.

**Nomenclatura:** `v3_<bag>__<imu>-<intrinsecos>`, com `<bag>` ∈ {`raw`, `rect`}, `<imu>` ∈
{`micro`, `zed`}, `<intrinsecos>` ∈ {`fabrica`, `kalibr`}. Saídas em `data/output/runs/<nome>/`.

---

## Fase 0 — Diagnóstico barato e preparo ✅

- [x] **0.1 💻 Continuidade dos 2 bags.**
  ✓ **Excelente:** `/imu/data` com **0 gaps e 0.0 s perdidos** nos dois; imagens com 1–2 gaps e
  ≤0.4 s. A diferença de contagem L/R (49 e 74) são **quedas isoladas de 1 frame** (121/71 no raw,
  118/53 no rect), no máximo 2 consecutivas. Ferramenta nova: `scripts/inspect_bag.py`.
- [x] **0.2 💻 Excitação rotacional** (D7).
  ✓ **Dobrou e ficou isotrópica:** raw 11.3/10.8/11.0 °/s, rect 9.1/9.6/9.6 °/s; razão forte/fraca
  **1.3–1.4×** (era 2.4× no v2) — a direção cega dominante sumiu. Piso teórico do lever-arm:
  0.5–0.7 mm (raw), 1.0–1.2 mm (rect).
  ✓ **Efeito mensurável:** o prior de timeshift, inconsistente no v2 (95 vs 130 ms entre câmeras
  sincronizadas), saiu **idêntico** aqui (−15.000 ms nas duas). Fecha a explicação que estava aberta.
- [x] **0.3 💻 Intrínsecos de fábrica** do `camera_info`.
  ✓ raw: `fx` 957.79/957.75, `rational_polynomial` de 8 coef, baseline 0.120093 m.
  ✓ rect: `fx=fy=947.799`, `D` zerado, baseline 0.120088 m.
  🔴 **Achado crítico:** truncar o racional para `radtan`-4 dá **989% de erro na borda**. Resolvido
  com `scripts/rational_to_radtan.py`, que **ajusta** um radtan-4 reproduzindo o mapeamento racional
  (resíduo 0.16 px médio). Sem isso a matriz inteira teria rodado sobre lixo.
- [x] **0.4 💻 Converter ROS 2 → ROS 1.**
  ✓ `v3_raw.bag` 7.9 GB, `v3_rect.bag` 6.1 GB, contagens exatas, ambas as IMUs.
  ✓ Conversor ganhou filtro de frames vazios (o ZED publicou 1 imagem 0×0 em 3325 no rect).

## Fase 1 — Piloto e portão ✅

- [x] **1.1 / 1.2 💻 `camchain` de fábrica do raw + `imu.yaml` da Microstrain.**
- [x] **1.3 🐳 `v3_raw__micro-fabrica`** — **🚦 PORTÃO: PASSOU.**
  ✓ 8 iterações, `lambda` **0.0046**, parada **por tolerância**, resíduos normalizados < 1.
  Contra o v2 (17+ iterações sem convergir, `lambda` 33 209): **cinco ordens de grandeza**.
  **→ R10/CA4 respondidos: o pipeline funciona fora d'água.**
  ✓ **Achado (D9):** reprojeção de 3.2 px porque o `T_cn_cnm1` derivado do campo `R` do
  `camera_info` tinha **0.389°** de erro (= 6.5 px, batendo com os 6.73 px iniciais do cam1). Com
  `--recompute-camera-chain-extrinsics` caiu para **0.68 px**, e o extrínseco câmera-IMU mudou só
  **1.0 mm** — era robusto ao erro.

## Fase 2 — Matriz com intrínsecos de fábrica ✅

- [x] **2.1 / 2.2 💻 `imu.yaml` da ZED + `camchain` do rect.**
- [x] **2.3 🐳 `v3_raw__zed-fabrica`** — 0.570/0.573 px.
  ⚠️ resíduos normalizados **0.222** (accel) e 0.422 (giro): ruído da ZED **inflado ~4.5× demais**
  (D6). A reprojeção boa é artefato do viés, não mérito da ZED.
- [x] **2.4 🐳 `v3_rect__micro-fabrica`** — 🔴 **mal-condicionado**: `lambda` **11 919**, giroN
  **1.215** (>1), 30 iterações sem tolerância.
- [x] **2.5 🐳 `v3_rect__zed-fabrica`** — 🔴 **falha silenciosa**: reprojeção ótima (0.596 px) mas
  `t_ic` de **429.5 mm** entre câmera e IMU da ZED, que ficam a ~23 mm no mesmo corpo. Timeshift
  −40.49 ms contra −4.67 ms do mesmo par no raw.

## Fase 3 — Intrínsecos estimados pelo Kalibr ✅

- [x] **3.1 🐳 `kalibr_calibrate_cameras` no raw** — reprojeção ±0.34 px; baseline 0.120093 m,
  **idêntico ao de fábrica**. Ponto principal de fábrica erra ~6.8 px em `cx`.
- [x] **3.2 🐳 `kalibr_calibrate_cameras` no rect** — **o diagnóstico**: o `camera_info` do tópico
  rect **não descreve** as imagens. `cy` 364.27 ± 0.65 contra 357.195 declarado (**10.9σ**),
  `k1` 0.0120 ± 0.0011 contra zero (**11σ**), e `cy` **difere entre as câmeras** (364.27 vs 366.51),
  o que num par retificado é contradição de definição.
- [x] **3.3 💻 Sanidade vs. SDK** — `fx` estimado 958.15 contra 957.79 de fábrica (~1σ, consistente).

## Fase 4 — Matriz com intrínsecos do Kalibr ✅ (3 de 4)

- [x] **4.1 🐳 `v3_raw__micro-kalibr`** — 0.673/0.682 px, `lambda` 0.12, 5 iterações. ✅ o melhor.
- [x] **4.2 🐳 `v3_raw__zed-kalibr`** — 0.563/0.569 px, `lambda` 1.10.
- [x] **4.3 🐳 `v3_rect__micro-kalibr`** — 🔴 `lambda` **4 918** mesmo com os intrínsecos corretos.
  ⚠️ **Refuta a hipótese** de que o camchain era a causa do mal-condicionamento do rect.
- [ ] **4.4 🐳 `v3_rect__zed-kalibr`** — ⏳ em re-execução (a 1ª tentativa coincidiu com o
  travamento da máquina; rodou 3 h contra ~25 min das demais).

## Fase 5 — Comparação e conclusão

- [x] **5.1 💻 Comparação cross-bag** — tabela completa em `resultados.md` §2.
- [x] **5.2 💻 Fábrica × Kalibr (CA5)** — **diferença de 0.001–0.007 px**. Recalibrar **não vale a
  pena**; o ajuste racional→radtan do de fábrica é equivalente. Economiza ~1 h por bag.
- [x] **5.3 💻 Verificação contra o `zed_macro` (R8/CA6)** — `scripts/check_zed_imu.py`.
  As duas execuções `raw` concordam em **2 mm** entre si mas ficam em **2×** o nominal (46–48 mm
  contra 23.2 mm). Provável causa: o ruído inflado da ZED (§7 do `resultados.md`). **Conclusão
  pendente** de refazer com ruído corrigido.
- [x] **5.4 💻 Raw × rect (D3)** — **divergem em ~43 mm** (0.198–0.204 m contra 0.233–0.247 m). Não
  é teste justo de repetibilidade, porque o rect está quebrado — mas cumpriu o papel de **sinalizar
  a configuração ruim**.
  ✓ **O baseline estéreo é recuperado corretamente em todas as execuções** (0.1197–0.1198 m contra
  0.1201 de fábrica), inclusive na que deu lever-arm de 430 mm. A geometria estéreo é sólida; o que
  varia é o braço câmera↔IMU.
- [ ] **5.5 💻 Conclusão explícita (R10/CA7)** — escrita em `resultados.md` §1 e §11. Falta fechar
  depois da 4.4.

## Fase 6 — Fechamento

- [ ] **6.1 💻 Atualizar** `premissas-e-fontes-de-erro.md` com o que a v3 confirmou/derrubou.
- [ ] **6.2 💻 Decisão de seguir para o DVL** — base pronta **com o bag raw + Microstrain**.

---

## Riscos — como se materializaram

| risco previsto | o que aconteceu |
|---|---|
| o piloto falhar como o v2 | ✅ **não ocorreu** — convergiu melhor que qualquer run submerso |
| excitação baixa de novo | ✅ **não ocorreu** — dobrou e ficou isotrópica |
| camchain do rect errado (erro silencioso) | 🔴 **ocorreu, e pior que o previsto**: o `camera_info` do SDK está errado, e corrigi-lo **não** resolveu |
| frames faltando na esquerda | ✅ inofensivo — quedas isoladas de 1 frame |
| `tagSize` não medido | ⏸️ segue aberto; não afeta comparação entre bags |
| tempo de máquina (~8 h) | 🔴 **a máquina travou** por falta de limites no container (ver `resultados.md` §9) |

**Risco não previsto que se materializou:** o container rodava sem limite de RAM/CPU e derrubou a
máquina. Corrigido com `docker update --memory 9g --memory-swap 9g --cpus 14`. Medido depois:
pico de 4.98 GiB e **1373% de CPU** na fase de extração — sem teto, saturava os 20 cores e matava
o servidor X.
