# Runbook — Allan variance da IMU da ZED 2i

> Atende **R1 / CA1** da `spec.md`. Documento **autocontido**: tudo que é preciso para executar está
> aqui, sem depender de contexto de conversa anterior.
>
> **Ambiente verificado em 2026-09-24.** Ver §2 antes de assumir que algo ainda vale.

---

## 1. Decisão: qual ferramenta, e por quê

**Escolhido: [`ori-drs/allan_variance_ros`](https://github.com/ori-drs/allan_variance_ros) (ROS 1),
instalado no container `kalibr_zed` que já existe.**

O fluxo é: gravar em ROS 2 (container do zed_wrapper, separado) → converter para ROS 1 no host →
rodar o pacote no `kalibr_zed`.

**Por que o pacote pronto e não implementação própria:** decisão do usuário — preferir código já
validado pela comunidade a reimplementar a variância de Allan. O pacote também gera o `imu.yaml` no
formato que o Kalibr consome, sem passo de tradução.

**Por que no container atual e não num novo:** todas as dependências já estão presentes (§2), então
o custo de instalação é clonar e compilar. Um container novo não traria isolamento suficiente para
justificar mantê-lo.

### Alternativas descartadas

| alternativa | por que não |
|---|---|
| [`Autoliv-Research/allan_variance_ros2`](https://github.com/Autoliv-Research/allan_variance_ros2) | Lê MCAP direto e dispensaria a conversão, mas é porte menos rodado que o original. Fica como **plano B** se o original der problema. |
| [`CruxDevStuff/allan_ros2`](https://github.com/CruxDevStuff/allan_ros2) | Idem, testado em Foxy/Humble. Segundo plano B. |
| Implementar em numpy no host | Descartado pelo usuário — preferência explícita por pacote validado. |

---

## 2. Ambiente verificado (2026-09-24)

Container `kalibr_zed`, Noetic / Ubuntu 20.04, workspace **`/catkin_ws`** (catkin_tools:
`build/ devel/ logs/ src/`), com `src/kalibr`.

**Dependências do `allan_variance_ros` — todas já presentes, nenhum `apt install` necessário:**

```
OK  rosbag        OK  tf2_ros
OK  sensor_msgs   OK  tf2_geometry_msgs
OK  geometry_msgs OK  yaml-cpp
OK  rospy
```

**Host:** sem ROS instalado (`/opt/ros` não existe). Tem `rosbags` **0.11.3** (Python), que é o que
faz a conversão. Disco: 136 GB livres.

⚠️ **O container tem scipy 1.3.3 e matplotlib 3.1.2** (antigos). Ver risco #3.

---

## 3. Conversão ROS 2 → ROS 1: usar `rosbags-convert`, **não** o script do repo

**Não use `scripts/ros2_to_ros1_kalibr.py` aqui.** Ele tem `--image-topics` como argumento
**obrigatório** e não roda num bag só de IMU. Além disso, tudo que ele customiza é específico de
imagem e irrelevante neste caso:

| customização do script | serve para o bag de IMU? |
|---|---|
| conversão para mono8 | não — não há imagem |
| filtro de frames vazios do ZED | não — idem |
| rebasing de timestamp (buffer float32 do spline do Kalibr) | não — o `allan_variance_ros` não tem esse problema |

Use o CLI genérico da mesma biblioteca.

> ⚠️ **Sintaxe:** a versão instalada (0.11.3) exige **`--src`**. A forma posicional
> (`rosbags-convert <pasta> --dst ...`) é de versões antigas e **falha**.

---

## 4. Passo a passo

### 4.0 — Snapshot do container (risco #1)

```bash
docker commit kalibr_zed kalibr_zed:pre-allan
```

Reverte em segundos se a compilação mexer em algo do Kalibr.

### 4.1 — Gravar (no container ROS 2 / zed_wrapper)

Bag **dedicado, só a IMU**. Nada de imagem — mantém o bag pequeno e elimina o passo de reordenação
(ver risco #6).

```bash
ros2 bag record /zed/zed_node/imu/data -o allan_zed --max-bag-duration 600
```

O `--max-bag-duration 600` divide em arquivos de 10 min: se a máquina cair na hora 18, o que já foi
gravado sobrevive (risco #5). A pasta inteira é aceita depois pelo `--src`.

**Condições da coleta:**
- câmera **parada, nivelada, em superfície estável**; ninguém circulando por perto;
- temperatura o mais constante possível — deriva térmica contamina o trecho longo do gráfico, que é
  justamente onde se lê o random walk;
- **20+ h** (OpenVINS). Menos que isso ainda dá a densidade de ruído, mas degrada o random walk;
- antes de começar, verificar **R1b da spec**: subir a IMU para ≥200 Hz se possível. 99 Hz é metade
  do mínimo recomendado.

### 4.2 — Instalar o pacote (container `kalibr_zed`)

```bash
docker start kalibr_zed && docker exec -it kalibr_zed bash

cd /catkin_ws/src
git clone https://github.com/ori-drs/allan_variance_ros
cd /catkin_ws
catkin build allan_variance_ros
. devel/setup.bash
```

### 4.3 — Medir a taxa **real** antes de converter (risco #2 — o mais importante)

```bash
python3 scripts/inspect_bag.py <pasta_do_bag_ros2>
```

Anote a taxa **efetiva medida**, não a nominal do config do wrapper. Ela vai para o `imu_rate`.

### 4.4 — Converter (host)

```bash
rosbags-convert \
  --src <pasta_do_bag_ros2> \
  --dst allan_zed.bag \
  --include-topic /zed/zed_node/imu/data
```

A extensão `.bag` já faz o destino sair como ROS 1. O `--include-topic` garante que só a IMU entre,
mesmo que algo a mais tenha sido gravado.

Tempo estimado: ~30 min a 400 Hz (Python puro, ~29 M mensagens).

### 4.5 — Rodar (container)

Config de entrada — **campos exatos**, conferidos no exemplo oficial do repo:

```yaml
# config/allan_zed.yaml
imu_topic: "/zed/zed_node/imu/data"
imu_rate: 200        # ⚠️ o valor MEDIDO em 4.3, não o assumido
measure_rate: 100    # subamostragem p/ o cálculo; precisa ser <= imu_rate
sequence_time: 72000 # 20 h em segundos
```

> ⚠️ Não confundir com o `imu.yaml` **de saída**, que usa outros nomes (`rostopic`, `update_rate`).

```bash
# o 1º argumento é a PASTA que contém o .bag, não o arquivo
rosrun allan_variance_ros allan_variance <pasta_com_o_bag> <caminho_do_config.yaml>
```

### 4.6 — Analisar (preferir o **host**, risco #3)

```bash
python3 analysis.py --data allan_variance.csv
```

Gera o `imu.yaml` com `accelerometer_noise_density`, `accelerometer_random_walk`,
`gyroscope_noise_density`, `gyroscope_random_walk`.

### 4.7 — Inflar antes de usar no Kalibr

Aplicar o fator de **10–20×** recomendado pelo OpenVINS, e **registrar o fator usado**.

> 🔄 **Corrige raciocínio anterior (v3):** naquele dataset o resíduo normalizado baixo (0.222) foi
> lido como "inflei demais" e o ruído foi **reduzido** — o resultado piorou. A direção correta é
> inflar **mais**, não menos.

---

## 5. Riscos

| # | risco | gravidade | mitigação |
|---|---|---|---|
| 1 | `catkin build` afetar o workspace do Kalibr | média | `docker commit` antes (4.0) |
| 2 | **`imu_rate` ≠ taxa real → ruído errado por fator de escala, sem aviso** | **alta** | medir no bag (4.3) |
| 3 | `analysis.py` quebrar com scipy 1.3.3 / mpl 3.1.2 do container | baixa | rodar no host — só lê o CSV |
| 4 | conversão lenta (~30 min a 400 Hz) | baixa | rodar junto com a gravação, de noite |
| 5 | perder 20 h se a gravação cair no fim | média | `--max-bag-duration 600` (4.1) |
| 6 | ordenação por header stamp (`cookbag.py`) | **nula** | bag só de IMU: ordem de recepção = ordem de header |

### Sobre o risco #2

O `allan_variance_ros` **não valida** o `imu_rate` contra o conteúdo do bag. Se divergir, os valores
de ruído saem errados por um fator fixo e **nada reclama**.

É a mesma classe de falha do `tagSpacing` (ver `spec.md` R3): converge normal, entrega número errado,
sintoma silencioso. E aqui o estrago é maior, porque esse `imu.yaml` pondera **todos** os termos
inerciais da calibração seguinte — inclusive o lever-arm, que é o que a spec quer medir.

---

## 6. Espaço em disco

`sensor_msgs/Imu` em ROS 1 dá ~450 B/mensagem com overhead de bag.

| taxa | 20 h | `.bag` | + mcap (ambos em disco) |
|---|---|---|---|
| 99 Hz | 7.1 M msgs | ~3.2 GB | ~6.4 GB |
| 200 Hz | 14.4 M msgs | ~6.5 GB | ~13 GB |
| 400 Hz | 28.8 M msgs | ~13 GB | ~26 GB |

Com 136 GB livres, cabe em qualquer cenário.

---

## 7. Pendências

- [ ] Escrever `scripts/config/allan_zed.yaml` e `scripts/run_allan.sh` (oferecido, ainda não feito).
- [ ] Confirmar se a IMU da ZED sobe para ≥200 Hz (R1b da spec).
- [ ] Registrar o fator de inflação efetivamente aplicado (4.7).
