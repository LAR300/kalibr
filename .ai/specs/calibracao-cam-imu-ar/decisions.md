# Decisões — Calibração câmera-IMU fora d'água (v3)

> ADR leve. Registra o **porquê** técnico. Durável.

### D1 — Um caso piloto antes da matriz completa

- **Contexto:** a matriz da spec são 8 execuções de cam-IMU. No v2, uma única execução rodou **1h40**
  e terminou sem convergir; outra morreu na iteração 3 por estouro de buffer. Disparar 8 às cegas
  arrisca ~10 h de máquina para descobrir no fim que todas falharam pelo mesmo motivo.
- **Decisão:** rodar **um** caso primeiro (bag raw + Microstrain + intrínsecos de fábrica), validar
  que converge de forma saudável, e só então abrir o leque.
- **Por quê:** o critério central da spec (CA4) é justamente *se converge bem*. Um caso responde isso.
  Se falhar, diagnostica-se com uma execução em vez de oito.
- **Consequências:** o caminho crítico fica sequencial no começo; em compensação, cada falha custa
  uma execução e não a bateria inteira.

### D2 — Execuções separadas por IMU, não um run multi-IMU

- **Contexto:** o Kalibr aceita `--imu microstrain.yaml zed.yaml` (a primeira vira a referência) e
  isso daria `T_zed_micro` de brinde.
- **Decisão:** uma execução por IMU, como o usuário pediu.
- **Por quê:** o propósito desta spec é **diagnosticar um pipeline que vinha falhando**. Com as duas
  IMUs no mesmo problema, uma não-convergência fica ambígua — não se sabe qual sensor a causou.
  Separado, a atribuição é imediata.
- **Consequências:** `T_zed_micro` não sai naturalmente (registrado como não-objetivo). Se depois
  interessar, é uma execução extra.

### D3 — Raw e retificado exigem camchain estruturalmente diferentes

- **Contexto:** o bag 1 gravou `gray/raw` e o bag 2 gravou `gray/rect`. O `camera_info` traz `K`+`D`
  (modelo da câmera crua) e `P` (projeção **retificada**).
- **Decisão:**
  - **bag raw:** `intrinsics` da matriz `K`, `distortion_coeffs` de `D`, `T_cn_cnm1` do extrínseco
    estéreo real.
  - **bag rect:** `intrinsics` da matriz `P` (`P[0]`, `P[5]`, `P[2]`, `P[6]`), `distortion_coeffs`
    **zerados**, e `T_cn_cnm1` com **rotação identidade** e translação `−P[3]/fx` em x.
- **Por quê:** numa imagem retificada a distorção já foi removida; aplicar `D` de novo corrige duas
  vezes e enviesa tudo silenciosamente. E a retificação alinha as linhas por construção, então a
  rotação entre as câmeras retificadas é identidade por definição.
- **Consequências:** comparar os dois bags vira um **teste extra de sanidade** — raw e rect deveriam
  convergir para o mesmo extrínseco câmera-IMU. Se divergirem, um dos dois camchains está errado.

### D4 — Intrínsecos como dimensão de variação, não escolha única

- **Contexto:** os intrínsecos disponíveis eram de calibração submersa (`fx = 1372`), inúteis no ar.
  Restam duas fontes: o `camera_info` de fábrica da ZED e uma estimação pelo próprio Kalibr.
- **Decisão:** rodar a matriz com as duas e guardar lado a lado (pedido do usuário).
- **Por quê:** a de fábrica é rápida e dá um resultado imediato; a do Kalibr ajusta o modelo às
  imagens reais. Comparar é a única forma de saber se recalibrar vale o custo — e o histórico mostra
  que truncar um modelo rico (o k3 dropado, D7 da spec de tanque) custou reprojeção.
- **Consequências:** dobra o número de execuções. Mitigado pela ordem: a fase de fábrica entrega
  resultado antes de a de Kalibr começar.
- **Ressalva:** o SDK usa `rational_polynomial` de **8** coeficientes e o `radtan` do Kalibr só tem
  4. A fonte "de fábrica" é, portanto, necessariamente **truncada** — e é exatamente o defeito que a
  fonte "Kalibr" existe para evitar. Esperar reprojeção pior na fábrica não é falha do teste, é o
  resultado esperado.

### D5 — `--timeoffset-padding 0.3` desde o início

- **Contexto:** o padrão 0.03 estourou na v1; 0.1 resolveu a v1 mas estourou na v2 (priors de 95 e
  130 ms). O sintoma é `Spline Coefficient Buffer Exceeded`, que **mata a otimização** no meio.
- **Decisão:** começar com 0.3 e só mexer se estourar de novo.
- **Por quê:** o custo de padding folgado é marginal; o de padding curto é perder uma execução inteira.
- **Consequências:** se 0.3 estourar num bag de ar, é sinal forte de problema de sincronização, não
  de configuração — e vira achado.

### D6 — Ruído da IMU da ZED: datasheet provisório, inflado

- **Contexto:** não há Allan variance para nenhuma das IMUs, e para a ZED não há sequer os valores
  que já usamos para a Microstrain.
- **Decisão:** partir de valores de datasheet **inflados** em relação ao nominal, e registrar como
  provisório.
- **Por quê:** subestimar o ruído faz o otimizador confiar demais na IMU e brigar com os termos de
  câmera. Inflar é o erro seguro: perde-se um pouco de precisão, não a convergência.
- **Consequências:** os valores absolutos de incerteza não são confiáveis; a **comparação** entre
  bags (que é o objetivo) permanece válida, pois todos usam os mesmos números.

### D7 — Medir excitação e continuidade **antes** de calibrar

- **Contexto:** no v2, só depois de horas de processamento é que se descobriu que a excitação
  rotacional estava abaixo do necessário e que o limite era nível de sinal, não qualidade de dado.
- **Decisão:** rodar `analyze_excitation.py` e a checagem de gaps nos dois bags **antes** da primeira
  calibração.
- **Por quê:** custa minutos e calibra a expectativa. Se a excitação estiver baixa de novo, sabe-se
  de antemão que a translação sairá imprecisa — e isso não deve ser confundido com falha do pipeline
  (que é o que a spec quer medir).
- **Consequências:** o resultado entra como contexto na interpretação, não como critério de parada.

### D8 — Reusar o layout `runs/<bag>__<variante>/` com symlink

- **Contexto:** o Kalibr grava as saídas ao lado do bag, usando a **string** do caminho
  (`kalibr_calibrate_imu_camera:218`), sem flag para mudar. Isso já causou um resultado sobrescrever
  outro nesta investigação.
- **Decisão:** cada execução num diretório próprio, com um symlink para o bag nomeado com o rótulo
  da variante.
- **Por quê:** o nome do symlink vira o prefixo de **todas** as saídas, então a variante se
  auto-rotula e aparece sozinha na tabela do comparador. Sem duplicar bags de 6 GB.
- **Consequências:** convenção de nome `v3_<bag>__<imu>-<intrinsecos>`, por exemplo
  `v3_raw__micro-fabrica`. Ver `data/output/README.md`.
