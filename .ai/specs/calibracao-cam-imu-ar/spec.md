# Spec — Calibração câmera-IMU fora d'água (dataset v3)

> **O quê e por quê.** Fonte da verdade da intenção. Sem decisão técnica nem passos de
> implementação (esses vão em `decisions.md` e `plan.md`).

## Problema

Todas as tentativas de calibrar câmera-IMU **submerso** deram problema, e os sintomas se acumularam
sem que nenhuma causa fosse isolada:

- **v1** (4 bags, julho): falhas graves de gravação — até 69 s perdidos num bag, buraco único de
  40.8 s, gaps simultâneos em câmera e IMU. Descartado (ver `validacao-calibracao-tanque/`).
- **v2** (3 bags, setembro): gaps resolvidos, mas a otimização ficou **mal-condicionada** —
  `lambda` do Levenberg-Marquardt em 33 209 contra 0.18 de um run saudável, `J` 11× maior, sem
  convergir em 17 iterações. Priors de timeshift inconsistentes (95 ms para cam0 e 130 ms para cam1,
  com o estéreo comprovadamente sincronizado a 1 ms).
- Em ambos, viés de ~13 cm da translação câmera-IMU contra o CAD, sem conseguir distinguir erro de
  cota de erro de estimação.

O ambiente submerso empilha variáveis difíceis de separar: **refração de porta plana** (não modelada
pelo `radtan` e dependente da distância), intrínsecos de uma calibração submersa antiga, excitação
rotacional baixa e ruído de IMU provisório. Com tudo junto, cada falha tem várias explicações
possíveis e nenhuma é testável isoladamente.

**A decisão é recuar para um cenário controlado:** calibrar **fora d'água**, onde a refração
desaparece e o modelo pinhole+radtan é adequado. Se o pipeline não funcionar nem aqui, o problema é
de método ou de ferramenta, não de água. Só depois de ter um cam-IMU sólido é que se volta ao DVL.

## Contexto

- **Dataset `cam_imu_dvl_v3`**, 2 gravações fora d'água olhando o alvo de calibração:

  | bag | duração | tópicos de imagem | imagens L/R | IMUs |
  |---|---|---|---|---|
  | `2026-09-18_16-57-01_calib_imu` | 290.7 s | `gray/**raw**` | 4245 / 4294 | `/imu/data` 200 Hz · `/zed/…/imu/data` 99 Hz |
  | `2026-09-18_17-15-26_calib_imu2` | 224.6 s | `gray/**rect**` | 3251 / 3325 | idem |

  Ambos a 27 MB/s (imagens em cinza — 4× menos que os 107 MB/s dos anteriores). Sem DVL, esperado.
  Também gravado `/ekf/imu/data`, que **não** entra em calibração (saída de EKF, não sensor bruto).

- **Os dois bags diferem no tipo de imagem:** o primeiro gravou **raw** (não retificada), o segundo
  **retificada**. São modelos ópticos diferentes e exigem tratamento distinto.

- **Duas IMUs disponíveis:** a MicroStrain 3DM-GV7 (externa, 200 Hz) e a IMU interna da ZED (99 Hz).
  A posição da IMU da ZED relativa à câmera esquerda é conhecida pelo `zed_macro`
  (`[-0.002, -0.023, -0.002]`), o que dá uma **referência geométrica independente do xacro do ROV** —
  exatamente o que faltava para desempatar o viés de 13 cm.

- **Os intrínsecos disponíveis hoje são de calibração submersa** (`fx = 1372`). Fora d'água o valor
  correto é da ordem de 957 (razão água/ar ≈ 1.43, coerente com refração). Reusá-los nos bags de ar
  seria erro grosseiro. Há duas fontes possíveis de intrínsecos de ar, e o usuário quer **ambas,
  comparadas**: os de fábrica da ZED (via `camera_info`) e os estimados pelo próprio Kalibr.

- **Ferramentas e convenções já estabelecidas** (ver `validacao-calibracao-tanque/` e
  `premissas-e-fontes-de-erro.md`): conversão ROS 2 → ROS 1 com rebasing de timestamps,
  layout `data/output/runs/<bag>__<variante>/` com symlink, comparador cross-bag
  (`scripts/compare_calibrations.py`).

## Requisitos

- **R1 — Calibrar câmera + IMU externa (MicroStrain)** nos dois bags do v3.
- **R2 — Calibrar câmera + IMU da ZED** nos dois bags, em execuções **separadas** de R1, para que uma
  falha aponte inequivocamente qual IMU a causou.
- **R3 — Duas fontes de intrínsecos, comparadas.** Executar tudo com (a) os intrínsecos de fábrica da
  ZED e (b) intrínsecos estimados pelo próprio Kalibr a partir destes bags. Os resultados das duas
  fontes devem ficar lado a lado.
- **R4 — Tratar corretamente raw e retificado.** Cada bag precisa do modelo óptico que corresponde às
  suas imagens; misturar os dois produz erro silencioso.
- **R5 — Consistência entre as duas gravações.** Comparar `T_cam_imu` (rotação e translação) e
  `timeshift` entre os dois bags, para cada combinação de IMU e fonte de intrínsecos.
- **R6 — Detectar e registrar erros de processo.** Falhas de convergência, estouro de buffer de
  spline, priors inconsistentes e afins são **resultado**, não acidente: precisam ficar registrados
  com o motivo, não apenas descartados.
- **R7 — Qualidade por execução.** Reportar erro de reprojeção, erros de giroscópio e acelerômetro,
  gravidade estimada, timeshift, e o comportamento da convergência (`lambda` final, nº de iterações,
  se parou por tolerância ou por limite).
- **R8 — Verificação geométrica independente.** Confrontar a pose câmera↔IMU-da-ZED estimada com o
  valor nominal do `zed_macro`, que não depende do xacro do ROV.
- **R9 — Saídas organizadas para comparação.** Uma execução por diretório, identificável por bag,
  IMU e fonte de intrínsecos, com a config exata usada preservada junto do resultado.
- **R10 — Conclusão explícita sobre o pipeline.** Ao final, uma resposta clara: o pipeline funciona
  fora d'água? Se sim, o problema anterior era da água/refração. Se não, é de método ou ferramenta.

## Não-objetivos

- **Calibrar DVL.** Fora de escopo aqui por definição — é a etapa seguinte, e nem há dado de DVL
  nestes bags. Volta-se a `dvl-calibration` quando o cam-IMU estiver sólido.
- **Recalibrar nada submerso.** Os datasets v1 e v2 não são reprocessados nesta spec.
- **Allan variance / estimar ruído das IMUs.** Continua-se com valores de datasheet; é spec futura.
- **Calibrar a IMU-IMU (`T_zed_micro`) como objetivo.** Se cair de brinde de alguma execução, ótimo,
  mas não é requisito — as execuções são separadas por IMU (R2).
- **Fechar o viés de 13 cm contra o xacro do ROV.** Aqui se usa a referência do `zed_macro` (R8); a
  cota do ROV é assunto da spec de validação em tanque.
- **Corrigir defeitos conhecidos do Kalibr.** O prior de timeshift calculado com `np.mean` e a
  gravação de saídas ao lado do bag ficam registrados, não corrigidos (decisão do usuário).
- **Medir o `tagSize`.** Segue como premissa aberta (ver abaixo).

## Critérios de aceitação

- [x] **CA1:** As execuções de R1–R3 rodam nos dois bags e cada uma termina com resultado **ou** com
      o motivo da falha registrado. ✓ 10 de 11 concluídas; a 11ª em re-execução após o travamento
      da máquina (causa e correção em `resultados.md` §9).
- [x] **CA2:** Existe, para cada execução, a config exata usada preservada junto do resultado, e o
      conjunto é navegável por bag / IMU / fonte de intrínsecos (R9). ✓ layout
      `data/output/runs/v3_<bag>__<imu>-<intrinsecos>/` com `config/` e symlink (D8).
- [x] **CA3:** Há uma comparação lado a lado de `T_cam_imu` e `timeshift` entre os dois bags, para
      cada combinação, com a dispersão explicitada. ✓ tabela em `resultados.md` §2.
      ⚠️ **A consistência entre os bags NÃO foi atingida** (~43 mm) — mas o bag `rect` está
      mal-condicionado, então não é teste justo de repetibilidade. Serviu para sinalizar a
      configuração quebrada. A consistência **interna** do `raw` (fábrica × Kalibr) é de 6 mm.
- [x] **CA4 (o teste central):** Fica claro se a calibração **converge de forma saudável** fora
      d'água — em contraste com o `lambda` de 33 209 e a não-convergência do v2.
      ✓ **SIM, no bag `raw`:** 5 iterações, `lambda` **0.12**, parada por tolerância, resíduos
      normalizados todos < 1. Cinco ordens de grandeza melhor que o v2.
      ⚠️ **NÃO no bag `rect`:** `lambda` 4 918–11 919 mesmo com os intrínsecos corrigidos.
- [x] **CA5:** O erro de reprojeção com os intrínsecos estimados pelo Kalibr é reportado ao lado do
      obtido com os de fábrica, permitindo julgar se vale recalibrar.
      ✓ **Diferença de 0.001–0.007 px: NÃO vale a pena recalibrar.** O `radtan`-4 ajustado ao modelo
      racional do SDK é equivalente a uma calibração completa. Economiza ~1 h por bag.
- [x] **CA6:** A pose câmera↔IMU-da-ZED é confrontada com o nominal do `zed_macro` (R8).
      ✓ `scripts/check_zed_imu.py`. As duas execuções `raw` concordam em **2 mm** entre si mas ficam
      em **2×** o nominal (46–48 mm contra 23.2 mm). ⚠️ **Conclusão pendente**: o ruído da IMU da ZED
      foi inflado ~4.5× demais, o que sub-pesa justamente quem observa o lever-arm.
- [x] **CA7:** Está escrito, em uma conclusão explícita, se o pipeline funciona fora d'água e o que
      isso implica para o caso submerso (R10). ✓ `resultados.md` §1 e §11.

## Perguntas em aberto

- **🔴 NEEDS CLARIFICATION (xacro novo) — BLOQUEIA a comparação com o CAD:** as gravações do v3 foram
  feitas com uma **nova estrutura**; o xacro do ROV mudou e a **posição da IMU externa (Microstrain)
  é outra**. A câmera ZED é o mesmo modelo, então a geometria interna dela (`zed_macro`) não muda.
  Precisamos das cotas novas em `base_link`: `zed_node_camera_link` e `imu_link`.
  - **Consequência:** qualquer comparação do v3 contra o nominal antigo é **sem significado**. O
    `compare_calibrations.py` agora exige `--nominal <arquivo>` e emite aviso quando cai no built-in
    da estrutura antiga.
  - **Não bloqueia** o resto da spec: convergência (CA4), consistência entre os dois bags (CA3),
    fábrica × Kalibr (CA5) e a verificação contra o `zed_macro` (CA6) independem do xacro do ROV.

- **NEEDS CLARIFICATION (tagSize):** os 0.088 m do AprilGrid nunca foram medidos fisicamente. Define
  a escala métrica de tudo. **Não bloqueia esta spec** — como os dois bags usam o mesmo alvo, um erro
  de escala afeta os dois igualmente e não atrapalha o teste de consistência (R5). Mas os valores
  absolutos de translação herdam o erro.
- **NEEDS CLARIFICATION (raw vs rect):** o usuário não sabia que os bags diferiam nisso. Resolvido
  tratando cada um adequadamente (R4), e a comparação entre eles vira teste extra — raw e rect
  deveriam dar o mesmo extrínseco. Confirmar depois se a diferença foi intencional.
- **NEEDS CLARIFICATION (ruído da IMU da ZED):** não temos valores para ela. Precisa de um chute de
  datasheet até a Allan variance.
- **NEEDS CLARIFICATION (excitação):** nos bags submersos a rotação estava em 5.8–7.6 °/s RMS, abaixo
  do necessário para o lever-arm ser bem observável. Ainda não medido nos bags do v3.
- **NEEDS CLARIFICATION (tolerâncias):** o que conta como "consistente" entre as duas gravações
  (R5/CA3) — a fixar depois dos primeiros números.
