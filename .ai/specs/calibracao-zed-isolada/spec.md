# Spec — Calibração câmera-IMU da ZED 2i isolada (fora d'água)

> **O quê e por quê.** Fonte da verdade da intenção. Sem decisão técnica nem passos de
> implementação (esses vão em `decisions.md` e `plan.md`).

## Problema

Três coletas (v1, v2, v3) e nenhuma produziu uma calibração **verificável**. O motivo não foi falta
de dados nem de ferramenta — foi falta de **referência**:

- **v1/v2 (submersos):** descartados por falhas de gravação e por mal-condicionamento causado pela
  refração não modelada.
- **v3 (fora d'água):** o pipeline **funcionou** — convergiu em 5 iterações, `lambda` 0.12, resíduos
  normalizados todos < 1. Mas os valores absolutos ficaram sem validação: o xacro do ROV estava
  desatualizado, e o único nominal disponível para a IMU da ZED era uma **aproximação feita à mão**
  num arquivo anterior à estrutura atual. O lever-arm saiu em 2× o nominal e **não foi possível
  decidir** se o erro era da estimação ou da referência.

Além disso, duas premissas atravessaram as três coletas sem nunca serem verificadas: o **`tagSize`**
(que define a escala métrica de todas as translações) e o **ruído das IMUs** (que pondera os termos
inerciais contra os visuais, e portanto determina a qualidade do lever-arm).

**Esta coleta fecha as três lacunas de uma vez.** Com a ZED 2i isolada:

1. O **SDK fornece a transformação câmera↔IMU de fábrica** — a referência externa que nunca houve.
2. A **Allan variance** fica barata (a câmera parada na mesa por uma noite) — e é o teste que
   fecha a incerteza do ruído inercial, aberta desde o v3.
3. O **alvo será medido** com paquímetro.

Com as três fechadas, pela primeira vez é possível responder se o pipeline produz o **valor certo**,
e não apenas se ele converge.

## Contexto

- **Hardware:** apenas a **ZED 2i** (câmera estéreo + IMU interna). Sem Microstrain, sem DVL.
- **Ambiente:** fora d'água, onde o v3 provou que o pipeline funciona.
- **Ferramentas prontas** no repo, validadas no v3: `inspect_bag.py`, `ros2_to_ros1_kalibr.py`,
  `rational_to_radtan.py`, `compare_calibrations.py`, `check_zed_imu.py`, `run_v3_matriz.sh`.
- **Allan variance:** ferramenta, ambiente e passo a passo já definidos em
  **[`runbook-allan-variance.md`](runbook-allan-variance.md)** — `ori-drs/allan_variance_ros` no
  container `kalibr_zed` (todas as dependências já presentes), com gravação em ROS 2 e conversão via
  `rosbags-convert`. O runbook é autocontido.
- **Boas práticas do OpenVINS** ([guia](https://docs.openvins.com/gs-calibration.html)), que o
  usuário trouxe e que ajustam vários pontos:
  - ordem: **intrínsecos → ruído da IMU → IMU-câmera**, em gravações **separadas**;
  - intrínsecos com o alvo cobrindo **toda a imagem**, em várias orientações e distâncias;
    IMU-câmera com o alvo **predominantemente centralizado**;
  - Allan variance com dataset estacionário de **20+ h**;
  - **inflar o ruído da Allan variance em 10–20×** para absorver erros não modelados;
  - câmera a **20–30 Hz**, IMU a **200–500 Hz**;
  - movimento **suave, não brusco**, excitando todos os eixos; ≥1 translação + ≥2 graus de mudança
    de orientação;
  - **30–60 s por dataset** costuma bastar;
  - reprojeção de **< 0.2–0.5 px** para os intrínsecos;
  - validar pelos gráficos: erros e biases dentro de 3-sigma, e conferência contra medida manual.

- **Aprendizados que condicionam a coleta** (ver `calibracao-cam-imu-ar/resultados.md` e
  `premissas-e-fontes-de-erro.md`):
  - o stream **retificado** do SDK é inutilizável para calibração;
  - vazão alta (imagens coloridas) causou perdas de até 69 s num bag;
  - o offset de relógio leva ~15 min para assentar depois de ligar;
  - o sinal do lever-arm escala com **ω²**, e uma excitação anisotrópica cria direção cega;
  - o **alvo menor favorece** esta coleta: a precisão da pose é angular, então aproximar-se mantendo
    a mesma ocupação da imagem melhora a precisão **em milímetros** — o que interessa para medir um
    lever-arm de ~25 mm. E sobra mais margem angular para girar antes de o alvo sair de vista.
  - **a reprojeção não detecta** os modos de falha que importam.

## Requisitos

- **R1 — Ruído da IMU medido, não assumido.** Gravar a IMU da ZED parada por **20+ h** (OpenVINS) e
  estimar densidades de ruído e random walk por Allan variance. Aplicar o **fator de inflação de
  10–20×** recomendado pela prática consolidada.
  > 🔄 **Corrige um raciocínio anterior:** no v3 o resíduo normalizado baixo (0.222) foi lido como
  > "inflei demais" e o ruído foi **reduzido** — o resultado piorou. A direção correta é inflar
  > **mais**, não menos.
  > 📋 **Como executar:** [`runbook-allan-variance.md`](runbook-allan-variance.md).
  > ⚠️ O risco principal é o `imu_rate` do config divergir da taxa real do bag: dá erro de **escala
  > silencioso** no ruído, mesma classe de falha do `tagSpacing` em R3. Medir antes de rodar.

- **R1b — Taxa da IMU.** Verificar se a IMU da ZED pode publicar acima de 99 Hz (o ZED 2i suporta até
  400 Hz) e, se puder, subir para ≥200 Hz. 99 Hz é **metade do mínimo** recomendado.
- **R2 — Referência de fábrica extraída.** Obter do SDK a transformação câmera↔IMU declarada pelo
  fabricante para esta unidade, **antes** da coleta, e registrá-la.
- **R3 — Alvo medido e configurado corretamente.** Alvo novo: **AprilGrid 5×7**, tag de **4.5 cm**,
  gap de **0.9 cm**. Medir fisicamente antes de usar e registrar o valor.
  > ⚠️ **`tagSpacing` no Kalibr é uma RAZÃO, não distância:** 0.9 ÷ 4.5 = **0.2**. Colocar `0.009`
  > daria escala errada, e o sintoma seria silencioso — converge normal e entrega translações
  > erradas por um fator fixo.
  > **Conferir a grade inteira**, não só um tag: 5 colunas devem dar `0.045 × 5.8 = 261.0 mm` e
  > 7 linhas `0.045 × 8.2 = 369.0 mm`. Divergência = a impressora escalou; ajustar o `tagSize` pela
  > razão medida.

- **R3b — Distância de trabalho compatível com o alvo menor.** O alvo tem 26 × 37 cm (contra 66 × 66
  do anterior), então a câmera precisa ficar mais perto para manter a mesma ocupação da imagem.
  Faixa de trabalho: **0.6 a 1.2 m**. Não descer abaixo de ~0.5 m — com baseline de 12 cm, a 0.4 m as
  duas câmeras veem cenas muito distintas e o overlap estéreo fica comprometido.
- **R4 — Gravação dedicada aos intrínsecos.** Um bag com o alvo cobrindo **toda a área da imagem**,
  em várias orientações e distâncias, com exposição curta para evitar blur.

- **R5 — Série de gravações para IMU-câmera.** 4 bags de **30–60 s**, com warm-up prévio, imagens
  **não retificadas** em escala de cinza, movimento **suave** excitando todos os eixos, e o alvo
  **predominantemente centralizado** na imagem.
- **R6 — Verificação no local, antes de desmontar.** Conferir continuidade, excitação e sincronia
  enquanto ainda é possível regravar.
- **R7 — Calibração câmera + IMU da ZED** nos 4 bags.
- **R8 — Repetibilidade.** Comparar os resultados entre os 4 bags e reportar a dispersão.
- **R9 — Exatidão contra a referência de fábrica.** Confrontar o extrínseco estimado com o valor do
  SDK. **É o requisito central desta spec** — o que nenhuma coleta anterior permitiu.
- **R10 — Julgamento pelos critérios corretos.** Avaliar por condicionamento (`lambda`), resíduos
  **normalizados** e plausibilidade física — **não** por erro de reprojeção.
- **R11 — Conclusão explícita.** Ao final, uma resposta clara: o pipeline produz o valor **correto**,
  dentro de que tolerância? Se sim, o método está validado de ponta a ponta e pode voltar ao sistema
  completo com confiança.

## Não-objetivos

- **Qualquer coisa submersa.** Esta spec é só ar; o retorno à água é a etapa seguinte.
- **DVL e Microstrain.** Não estão disponíveis nesta coleta.
- ~~Recalibrar os intrínsecos com o Kalibr.~~ **Revertido pelo guia do OpenVINS.** O v3 mediu ganho
  de 0.001–0.007 px, mas ali a recalibração usava **o mesmo bag** da calibração IMU-câmera, cujo
  movimento não cobre a imagem inteira. O OpenVINS recomenda uma gravação **dedicada**, com o alvo
  em toda a área da imagem — condição que nunca testamos. Passa a ser **objetivo** (R4).
- **Converter para o formato do AQUA-SLAM.** O objetivo escolhido é **validar**, não entregar
  calibração para consumo. A conversão fica para quando houver calibração do sistema completo.
- **Gravar no stream retificado.** Descartado por evidência (ver `calibracao-cam-imu-ar` §4).
- **Recalibrar a extrínseca estéreo como objetivo.** Ela é estimada durante a calibração
  (`--recompute-camera-chain-extrinsics`) e serve de verificação cruzada contra o baseline de
  fábrica, mas não é o entregável.

## Critérios de aceitação

- [ ] **CA1:** Existe estimativa de ruído da IMU por Allan variance (20+ h), com o fator de inflação
      aplicado e registrado (R1).
- [ ] **CA2:** A taxa da IMU foi verificada e, se possível, elevada para ≥200 Hz (R1b).
- [ ] **CA3:** A transformação câmera↔IMU de fábrica está registrada **antes** da coleta (R2).
- [ ] **CA4:** As dimensões medidas do alvo estão registradas, o `target.yaml` usa `tagSpacing`
      como **razão** (0.2, não 0.009), e a distância de trabalho ficou em 0.6–1.2 m (R3, R3b).
- [ ] **CA5:** Existe o bag dedicado a intrínsecos, com cobertura de toda a área da imagem, e a
      calibração dele atinge reprojeção **< 0.5 px** (R4).
- [ ] **CA6:** Os 4 bags de IMU-câmera foram gravados no formato correto (cinza, não retificado) e
      **verificados no local**: sem gaps relevantes, excitação isotrópica, alvo centralizado (R5, R6).
- [ ] **CA7:** As 4 calibrações concluem com condicionamento saudável — `lambda` < 1, parada por
      tolerância, resíduos normalizados entre ~0.5 e 1.5, e erros/biases dentro de 3-sigma (R7, R10).
- [ ] **CA8 (repetibilidade):** A dispersão do extrínseco entre os 4 bags está reportada, em
      translação (mm) e rotação (graus) (R8).
- [ ] **CA9 (exatidão — o teste central):** O extrínseco estimado é confrontado com o de fábrica, e a
      diferença está quantificada em mm e graus (R9).
- [ ] **CA10:** Está escrito se o pipeline produz o valor correto e dentro de que tolerância, com a
      implicação para o retorno ao sistema completo (R11).

## Perguntas em aberto

- **NEEDS CLARIFICATION (tolerância de CA7):** que diferença contra a referência de fábrica conta
  como "validado"? A fixar depois de ver os números — mas vale antecipar que o nominal do fabricante
  também tem incerteza (é por modelo, não medido por unidade), então **concordância perfeita não é
  esperada nem necessária**.
- **NEEDS CLARIFICATION (rotação de referência):** o SDK fornece a rotação câmera↔IMU além da
  translação? Se fornecer apenas a translação, o CA7 cobre só o lever-arm.
- **NEEDS CLARIFICATION (excitação alcançável):** ~20 °/s RMS é a meta; o v3 chegou a 11 °/s. Se o
  alvo precisar ficar no campo de visão o tempo todo, pode haver um teto prático — a verificar na
  tarefa de campo.
