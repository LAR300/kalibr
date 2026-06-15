# Kalibr — Guia Completo de Calibração Câmera + IMU

## Sumário

1. [Introdução](#introdução)
2. [Por que calibrar?](#por-que-calibrar)
3. [Pré-requisitos](#pré-requisitos)
4. [Estrutura de Pastas](#estrutura-de-pastas)
5. [Arquivos de Entrada](#arquivos-de-entrada)
   - [Dados brutos (imagens + IMU)](#1-dados-brutos-imagens--imu)
   - [Arquivo de configuração da câmera (camchain.yaml)](#2-arquivo-de-configuração-da-câmera-camchainyaml)
   - [Arquivo de configuração da IMU (imu.yaml)](#3-arquivo-de-configuração-da-imu-imuyaml)
   - [Arquivo do alvo de calibração (target.yaml)](#4-arquivo-do-alvo-de-calibração-targetyaml)
6. [Passo a Passo da Calibração](#passo-a-passo-da-calibração)
   - [Passo 1 — Preparar o ambiente Docker](#passo-1--preparar-o-ambiente-docker)
   - [Passo 2 — Preparar os dados brutos](#passo-2--preparar-os-dados-brutos)
   - [Passo 3 — Gerar o ROS bag](#passo-3--gerar-o-ros-bag)
   - [Passo 4 — Calibrar os intrínsecos da câmera (recomendado)](#passo-4--calibrar-os-intrínsecos-da-câmera-recomendado)
   - [Passo 5 — Calibrar câmera + IMU](#passo-5--calibrar-câmera--imu)
7. [Arquivos de Resultado](#arquivos-de-resultado)
8. [Como Interpretar os Resultados](#como-interpretar-os-resultados)
9. [Cuidados e Boas Práticas](#cuidados-e-boas-práticas)
10. [Troubleshooting](#troubleshooting)

---

## Introdução

O **Kalibr** é uma toolbox de calibração desenvolvida pelo Autonomous Systems Lab (ETH Zurich) para estimar os parâmetros espaciais e temporais entre câmeras e IMUs (Inertial Measurement Units). Ele utiliza otimização batch com B-splines em tempo contínuo para fusionar dados assíncronos de múltiplos sensores.

Este guia descreve o processo completo de calibração **câmera + IMU** usando o Kalibr em ambiente Docker.

---

## Por que calibrar?

A calibração câmera-IMU determina:

| Parâmetro | Descrição | Por que importa |
|---|---|---|
| **Transformação espacial (T_cam_imu)** | Posição e orientação relativa entre a câmera e a IMU | Essencial para fusão de dados em SLAM, VIO (Visual-Inertial Odometry), navegação autônoma |
| **Offset temporal (timeshift)** | Diferença de clock entre câmera e IMU | Sincronização incorreta causa drift e erros de estimação de estado |
| **Intrínsecos da câmera** | Focal length, principal point, distorção | Afeta diretamente a precisão da reprojeção e estimação de pose |

Sem calibração precisa, algoritmos de odometria visual-inercial (como VINS-Mono, MSCKF, ORB-SLAM3) não conseguem fusionar corretamente os dados dos sensores.

---

## Pré-requisitos

- **Docker** instalado no sistema host
- **Alvo de calibração** (AprilGrid recomendado)
  - Pode ser gerado em: https://github.com/ethz-asl/kalibr/wiki/calibration-targets
- **Dados de calibração** coletados (imagens + dados da IMU sincronizados)
- **ROS** rodando no host (para o `roscore`) ou rodar tudo dentro do container

---

## Estrutura de Pastas

Toda a estrutura de dados deve estar dentro do volume compartilhado com o Docker (`scripts/data/`), que é montado em `/data` dentro do container.

```
scripts/
├── Makefile                          # Comandos Docker
├── data/                             # Volume compartilhado (montado em /data no container)
│   ├── calibration/
│   │   └── cam_imu/
│   │       └── <SESSAO>/             # Ex: 2026-05-20_20-03-04_cam_imu_calib
│   │           ├── cam0/             # Imagens da câmera (timestamps em nanossegundos)
│   │           │   ├── 1779307384567800320.png
│   │           │   ├── 1779307384667899392.png
│   │           │   └── ...
│   │           ├── imu0.csv          # Dados da IMU
│   │           ├── camchain.yaml     # Configuração da câmera (formato camchain)
│   │           ├── imu0.yaml         # Configuração da IMU
│   │           └── target.yaml       # Configuração do alvo de calibração
│   └── output/                       # Resultados da calibração
│       ├── cam_imu.bag               # ROS bag gerado
│       ├── cam_imu-results-imucam.txt
│       ├── cam_imu-camchain-imucam.yaml
│       ├── cam_imu-imu.yaml
│       └── cam_imu-report-imucam.pdf
└── docs/
    └── kalibr.md                     # Este documento
```

---

## Arquivos de Entrada

### 1. Dados brutos (imagens + IMU)

#### Imagens — pasta `cam0/`

As imagens devem estar em uma pasta com prefixo `cam` (ex: `cam0`, `cam1` para estéreo). Cada imagem deve ter o **nome = timestamp em nanossegundos** com extensão `.png`, `.jpg` ou `.bmp`.

```
cam0/
├── 1779307384567800320.png
├── 1779307384667899392.png
├── 1779307384767911168.png
└── ...
```

> **Importante:** O nome do arquivo (sem extensão) é usado como timestamp da imagem. Deve ser um inteiro representando nanossegundos desde a epoch Unix.

#### Dados da IMU — arquivo `imu0.csv`

O nome do arquivo **deve começar com `imu`** e ter extensão `.csv`. As colunas devem seguir este formato:

```csv
timestamp [ns],omega_x [rad/s],omega_y [rad/s],omega_z [rad/s],alpha_x [m/s^2],alpha_y [m/s^2],alpha_z [m/s^2]
1779307384533935360,0.042207,-0.016299,-0.030140,0.197073,0.859901,9.608495
1779307384538990336,0.042227,-0.018468,-0.030672,0.185190,0.895563,9.620505
...
```

| Coluna | Unidade | Descrição |
|---|---|---|
| timestamp | nanossegundos | Timestamp absoluto |
| omega_x, omega_y, omega_z | rad/s | Velocidade angular (giroscópio) |
| alpha_x, alpha_y, alpha_z | m/s² | Aceleração linear (acelerômetro) |

> **Atenção:** O nome do arquivo define o tópico ROS no bag. `imu0.csv` → tópico `/imu0`.

---

### 2. Arquivo de configuração da câmera (`camchain.yaml`)

Este arquivo define os parâmetros intrínsecos da câmera no formato **camchain** (esperado pelo `kalibr_calibrate_imu_camera`).

```yaml
cam0:
  cam_overlaps: []              # Lista de câmeras com campo de visão sobreposto (vazio para câmera única)
  camera_model: pinhole         # Modelo: pinhole, omni, ds (double sphere)
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]   # Coeficientes de distorção [k1, k2, p1, p2]
  distortion_model: radtan      # Modelo: radtan, equidistant
  intrinsics: [528.8, 528.8, 640.0, 360.0]   # [fx, fy, cx, cy]
  resolution: [1280, 720]       # [largura, altura] em pixels
  rostopic: /cam0/image_raw     # Tópico ROS no bag (gerado pelo bagcreater: /<nome_pasta>/image_raw)
```

#### Modelos de câmera suportados

| Modelo | Parâmetros intrínsecos | Uso típico |
|---|---|---|
| `pinhole` | `[fx, fy, cx, cy]` | Câmeras convencionais |
| `omni` | `[xi, fx, fy, cx, cy]` | Câmeras omnidirecionais |
| `ds` | `[xi, alpha, fx, fy, cx, cy]` | Double sphere (fisheye) |

#### Modelos de distorção suportados

| Modelo | Coeficientes | Uso típico |
|---|---|---|
| `radtan` | `[k1, k2, p1, p2]` | Distorção radial-tangencial (padrão OpenCV) |
| `equidistant` | `[k1, k2, k3, k4]` | Câmeras fisheye / grande angular |

> **Importante:** O `rostopic` deve ser `/<nome_da_pasta>/image_raw`. Se suas imagens estão em `cam0/`, o tópico no bag será `/cam0/image_raw`.

> **Recomendação:** Para melhores resultados, calibre primeiro os intrínsecos com `kalibr_calibrate_cameras` em vez de usar valores estimados ou do fabricante.

---

### 3. Arquivo de configuração da IMU (`imu0.yaml`)

```yaml
accelerometer_noise_density: 0.01      # m/s²/√Hz  — Noise density do acelerômetro
accelerometer_random_walk: 0.0002      # m/s³/√Hz  — Random walk do acelerômetro
gyroscope_noise_density: 0.0004        # rad/s/√Hz — Noise density do giroscópio
gyroscope_random_walk: 0.00002         # rad/s²/√Hz — Random walk do giroscópio
model: calibrated                      # Modelo da IMU
rostopic: /imu0                        # Tópico ROS no bag (gerado pelo bagcreater: /<nome_arquivo_sem_extensao>)
update_rate: 198                       # Taxa de atualização da IMU em Hz
```

#### Como obter os parâmetros de ruído da IMU

Os parâmetros de ruído podem ser obtidos de 3 formas (da melhor para a pior):

1. **Allan Variance** — análise de dados estáticos da IMU (mais preciso)
2. **Datasheet do fabricante** — valores nominais do sensor (ex: BMI055 para ZED 2i)
3. **Valores típicos** — estimativas conservadoras baseadas no tipo de sensor

#### Valores típicos por tipo de sensor

| Sensor | accel_noise | accel_walk | gyro_noise | gyro_walk |
|---|---|---|---|---|
| MEMS consumer (BMI055) | 0.01 | 0.0002 | 0.0004 | 2e-05 |
| MEMS industrial (ADIS16448) | 0.006 | 0.0002 | 0.0004 | 4e-06 |
| MEMS tático | 0.002 | 0.0001 | 0.0001 | 1e-06 |

> **Atenção:** O `rostopic` deve corresponder ao nome do arquivo CSV sem extensão. `imu0.csv` → `/imu0`.

> **Modelo:** Use `calibrated` para IMUs já calibradas pelo fabricante. Opções avançadas: `scale-misalignment`, `scale-misalignment-size-effect`.

---

### 4. Arquivo do alvo de calibração (`target.yaml`)

#### AprilGrid (recomendado)

```yaml
target_type: 'aprilgrid'
tagCols: 6               # Número de colunas de tags
tagRows: 6               # Número de linhas de tags
tagSize: 0.088            # Tamanho do tag em metros (lado externo do quadrado preto)
tagSpacing: 0.3           # Razão entre espaçamento e tamanho do tag (espaço / tagSize)
```

#### Checkerboard (alternativa)

```yaml
target_type: 'checkerboard'
targetCols: 7             # Número de cantos internos (colunas)
targetRows: 6             # Número de cantos internos (linhas)
rowSpacingMeters: 0.03    # Espaçamento entre linhas em metros
colSpacingMeters: 0.03    # Espaçamento entre colunas em metros
```

> **Dica:** O AprilGrid é preferível ao checkerboard porque permite detecção parcial (não precisa ver o grid inteiro) e é mais robusto a oclusões.

---

## Passo a Passo da Calibração

### Passo 1 — Preparar o ambiente Docker

```bash
# Na pasta scripts/
cd scripts/

# Construir a imagem Docker
make build

# Iniciar o container (com suporte a GUI e volume compartilhado)
make run
```

O container monta a pasta `scripts/data/` do host em `/data` dentro do container.

Em outro terminal no **host**, inicie o ROS master:

```bash
roscore
```

---

### Passo 2 — Preparar os dados brutos

Coloque os dados de calibração na estrutura esperada dentro de `scripts/data/`:

```
data/calibration/cam_imu/<NOME_DA_SESSAO>/
├── cam0/                    # Imagens com nome = timestamp em nanossegundos
│   ├── 1779307384567800320.png
│   └── ...
├── imu0.csv                 # Dados da IMU (7 colunas)
├── camchain.yaml            # Configuração da câmera
├── imu0.yaml                # Configuração da IMU
└── target.yaml              # Configuração do alvo
```

---

### Passo 3 — Gerar o ROS bag

Dentro do container, execute:

```bash
rosrun kalibr kalibr_bagcreater \
  --folder /data/calibration/cam_imu/<SESSAO> \
  --output-bag /data/output/cam_imu.bag
```

**O que o comando faz:**
- Lê as imagens de cada pasta `cam*/` e cria mensagens `sensor_msgs/Image`
- Lê os arquivos `imu*.csv` e cria mensagens `sensor_msgs/Imu`
- Grava tudo em um arquivo ROS bag

**Tópicos criados no bag:**

| Origem | Tópico no bag |
|---|---|
| `cam0/` | `/cam0/image_raw` |
| `cam1/` | `/cam1/image_raw` |
| `imu0.csv` | `/imu0` |

> **Atenção:** O script não imprime progresso — ele apenas mostra "importing libraries" e trabalha silenciosamente. Aguarde o retorno ao prompt.

---

### Passo 4 — Calibrar os intrínsecos da câmera (recomendado)

Se você não possui valores intrínsecos precisos da câmera, calibre-os primeiro:

```bash
rosrun kalibr kalibr_calibrate_cameras \
  --bag /data/output/cam_imu.bag \
  --topics /cam0/image_raw \
  --models pinhole-radtan \
  --target /data/calibration/cam_imu/<SESSAO>/target.yaml \
  --dont-show-report
```

**Parâmetros:**

| Flag | Descrição |
|---|---|
| `--bag` | Caminho do ROS bag |
| `--topics` | Tópicos das câmeras (separados por espaço para múltiplas câmeras) |
| `--models` | Modelo câmera-distorção (ex: `pinhole-radtan`, `pinhole-equidist`, `omni-radtan`) |
| `--target` | Arquivo do alvo de calibração |
| `--dont-show-report` | Não abrir relatório gráfico ao final |

O resultado gera um arquivo `camchain-*.yaml` que deve ser usado no passo seguinte.

---

### Passo 5 — Calibrar câmera + IMU

```bash
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag /data/output/cam_imu.bag \
  --cams /data/calibration/cam_imu/<SESSAO>/camchain.yaml \
  --imu /data/calibration/cam_imu/<SESSAO>/imu0.yaml \
  --target /data/calibration/cam_imu/<SESSAO>/target.yaml \
  --show-extraction
```

**Parâmetros:**

| Flag | Descrição |
|---|---|
| `--bag` | Caminho do ROS bag |
| `--cams` | Camchain YAML (resultado do passo 4 ou configuração manual) |
| `--imu` | Configuração da IMU |
| `--target` | Configuração do alvo |
| `--show-extraction` | Mostrar janela com detecção dos cantos em tempo real |
| `--dont-show-report` | Não abrir relatório gráfico ao final |
| `--max-iter N` | Número máximo de iterações do otimizador (padrão: 30) |
| `--bag-freq Hz` | Frequência de extração de features (reduzir para datasets grandes) |

> **Tempo de execução:** Varia de 5 a 30+ minutos dependendo do tamanho do dataset e número de iterações.

---

## Arquivos de Resultado

A calibração gera os seguintes arquivos no diretório de onde o comando é executado:

| Arquivo | Descrição |
|---|---|
| `*-results-imucam.txt` | Resumo textual completo dos resultados |
| `*-camchain-imucam.yaml` | Camchain atualizado com a transformação T_cam_imu e timeshift |
| `*-imu.yaml` | Parâmetros da IMU (modelo calibrado) |
| `*-report-imucam.pdf` | Relatório visual com gráficos de residuais, trajetória e detecções |

### Exemplo de resultado — `camchain-imucam.yaml`

```yaml
cam0:
  T_cam_imu:                    # Matriz 4x4 de transformação (IMU → câmera)
  - [0.0238, -0.9997,  0.0092,  0.0432]
  - [-0.0122, -0.0095, -0.9999,  0.0133]
  - [0.9996,  0.0237, -0.0124, -0.0150]
  - [0.0,     0.0,     0.0,     1.0   ]
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
  distortion_model: radtan
  intrinsics: [528.8, 528.8, 640.0, 360.0]
  resolution: [1280, 720]
  rostopic: /cam0/image_raw
  timeshift_cam_imu: -0.001099  # Offset temporal em segundos (t_imu = t_cam + shift)
```

---

## Como Interpretar os Resultados

### Erro de reprojeção

| Valor (mean) | Qualidade |
|---|---|
| **< 0.5 px** | Excelente |
| **0.5 – 1.0 px** | Bom |
| **1.0 – 2.0 px** | Aceitável |
| **> 2.0 px** | Ruim — verificar dados e parâmetros |

### Translação (T_cam_imu)

A translação deve corresponder à **distância física** entre a câmera e a IMU no sensor. Para a ZED 2i, por exemplo, essa distância é de poucos centímetros. Valores muito grandes (> 10 cm) indicam problemas na calibração.

### Timeshift

| Valor | Interpretação |
|---|---|
| **< 10 ms** | Normal para sensores sincronizados por hardware |
| **10 – 50 ms** | Pode ocorrer com sincronização por software |
| **> 50 ms** | Possível problema de sincronização |

### Vetor gravidade

A magnitude deve ser ≈ **9.81 m/s²**. Desvios grandes indicam problemas com os dados da IMU ou escala incorreta.

---

## Cuidados e Boas Práticas

### Durante a captura dos dados

1. **Movimente o sensor em todos os 6 graus de liberdade** — translação (X, Y, Z) e rotação (roll, pitch, yaw)
2. **Evite movimentos bruscos** — movimentos rápidos geram motion blur nas imagens
3. **Mantenha o alvo visível** na maior parte do tempo
4. **Capture dados por 60-120 segundos** — tempo suficiente para excitar todos os modos
5. **Mantenha o alvo fixo e mova o sensor** (não o contrário)
6. **Cubra todo o campo de visão** — mova o sensor para que o alvo apareça em diferentes regiões da imagem

### Configuração dos arquivos

1. **Confira a resolução das imagens** — deve corresponder exatamente ao campo `resolution` no `camchain.yaml`
2. **Confira os tópicos ROS** — devem corresponder ao nome das pastas/arquivos:
   - Pasta `cam0/` → rostopic `/cam0/image_raw`
   - Arquivo `imu0.csv` → rostopic `/imu0`
3. **Use parâmetros de ruído da IMU realistas** — valores muito baixos fazem o otimizador confiar demais na IMU; valores muito altos reduzem a contribuição da IMU
4. **Meça o tamanho do tag com precisão** — erros no `tagSize` afetam diretamente a escala da calibração

### Otimização

1. **Calibre intrínsecos separadamente** sempre que possível (passo 4)
2. **Verifique o relatório PDF** — gráficos de residuais devem ter distribuição uniforme sem outliers
3. **Se o erro de reprojeção for alto**, tente:
   - Reduzir a frequência com `--bag-freq 4` (usar apenas 4 Hz)
   - Recapturar dados com movimentos mais lentos
   - Calibrar os intrínsecos da câmera antes

---

## Troubleshooting

### Erro: "Could not find topic X in bag"

O tópico ROS especificado no YAML não existe no bag. Verifique:
- O `rostopic` no `camchain.yaml` deve ser `/<nome_pasta>/image_raw` (ex: `/cam0/image_raw`)
- O `rostopic` no `imu0.yaml` deve ser `/<nome_arquivo_csv>` (ex: `/imu0`)

### Erro: "DLT algorithm needs at least 6 points"

O detector encontrou poucos pontos do alvo em uma imagem. Causas:
- Resolução no YAML diferente da resolução real das imagens
- Intrínsecos muito distantes dos reais (undistortion corrompe a imagem)
- Alvo parcialmente visível ou muito distante

### O script `kalibr_bagcreater` não gera output

Verifique se:
- As pastas de imagem começam com `cam` (ex: `cam0`)
- Os arquivos de IMU começam com `imu` e têm extensão `.csv`
- As imagens têm extensão `.png`, `.jpg` ou `.bmp`

### Translação muito grande nos resultados

- Intrínsecos da câmera incorretos (especialmente focal length)
- Dados insuficientes ou com pouca excitação
- Parâmetros de ruído da IMU mal configurados
