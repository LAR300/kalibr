---
description: Termos do domínio do Kalibr (calibração câmera/IMU, alvos, modelos). Abrir quando aparecer um termo cujo significado não é óbvio.
sources: [README.md, scripts/docs/kalibr.md, aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py]
---

# Glossário do domínio

- **Intrínsecos:** parâmetros internos da câmera — foco `fx, fy`, ponto principal `cx, cy`, e distorção.
- **Extrínsecos:** transformação rígida entre sensores. `T_cam_imu` (IMU→câmera, 4×4); `T_cn_cnm1` (câmera n em relação à n−1, no camchain).
- **T_cam_imu:** matriz 4×4 de saída da calibração CAM-IMU: pose da IMU no frame da câmera. Ver `data-model.md` para a convenção exata.
- **Timeshift (timeshift_cam_imu):** offset temporal entre os relógios da câmera e da IMU, em segundos (`t_imu = t_cam + shift`). Estimado como variável contínua graças à spline.
- **Camchain:** arquivo YAML descrevendo a cadeia de câmeras (intrínsecos, distorção, extrínsecos entre câmeras, tópicos). Entrada do `kalibr_calibrate_imu_camera`; a saída CAM-IMU é o `camchain-imucam.yaml`.
- **AprilGrid:** alvo de calibração feito de tags AprilTag numa grade. Permite detecção parcial (não precisa ver a grade inteira) e é robusto a oclusão — preferido ao checkerboard. Parâmetros: `tagCols`, `tagRows`, `tagSize` (m), `tagSpacing` (razão espaço/tag).
- **Checkerboard / circlegrid:** alvos alternativos. Checkerboard: `targetCols/Rows` (cantos internos) + espaçamento em metros.
- **Modelo de câmera:** `pinhole` `[fx,fy,cx,cy]`, `omni` `[xi,fx,fy,cx,cy]`, `ds` (double sphere, fisheye) `[xi,alpha,fx,fy,cx,cy]`, `eucm`.
- **Modelo de distorção:** `radtan` (radial-tangencial, `[k1,k2,p1,p2]`, padrão OpenCV), `equidistant`/`equi` (fisheye, `[k1,k2,k3,k4]`), `fov`.
- **Noise density:** densidade de ruído do sensor por √Hz. `accelerometer_noise_density` (m/s²/√Hz), `gyroscope_noise_density` (rad/s/√Hz).
- **Random walk:** deriva do bias. `accelerometer_random_walk` (m/s³/√Hz), `gyroscope_random_walk` (rad/s²/√Hz).
- **Allan Variance:** análise de dados estáticos da IMU para estimar noise density e random walk (melhor fonte desses valores; alternativas: datasheet, valores típicos).
- **B-spline (de pose):** curva contínua por partes usada para representar a trajetória do sensor no tempo; permite avaliar pose/velocidade/aceleração em qualquer instante. Base teórica da estimação batch do Kalibr.
- **Estimação batch / continuous-time:** resolver toda a trajetória + calibração de uma vez, minimizando resíduos de todos os sensores, com a trajetória modelada em tempo contínuo (spline).
- **Error term / resíduo:** termo que o otimizador minimiza — reprojeção (câmera), acelerômetro, giroscópio.
- **Erro de reprojeção:** distância (px) entre o ponto do alvo projetado e o detectado. Métrica de qualidade: <0.5px excelente, >2px ruim.
- **Observabilidade:** se os parâmetros são recuperáveis com os dados dados (análise QR / informação mútua em `incremental_calibration`). Movimento em 6-DOF melhora a observabilidade.
- **rosbag:** contêiner de mensagens ROS gravadas (imagens + IMU) usado como dataset de entrada. Criado por `kalibr_bagcreater`.
- **ZED 2i:** câmera estéreo com IMU (acel/giro BMI055) que é o alvo do fluxo customizado em `scripts/`.
