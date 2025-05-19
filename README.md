# 📐📷 distance_camera2aruco - Visão Computacional Aplicada

Este projeto foi desenvolvido como parte da disciplina **Visão Computacional Aplicada**, e tem como objetivo detectar marcadores **ArUco**, estimar sua pose em relação à câmera, e calcular a **distância e orientação** em tempo real.

Além disso, a versão estendida do código permite:
- 📸 **Salvar imagens** pressionando `s`;
- 🎥 **Gravar vídeos** pressionando `r` para iniciar/parar.

---

## 🎯 Objetivos

- Detectar marcadores ArUco em imagens de uma câmera.
- Estimar a pose (posição e orientação) do marcador em relação à câmera.
- Calcular ângulos de Euler a partir de quaternions.
- Exibir visualmente a posição do marcador com eixos.
- Adicionar recursos de captura e gravação de vídeo para experimentos visuais.

---

## 🧰 Requisitos

Instale as dependências com:

```bash
pip install opencv-python numpy scipy is-wire
```

---

## 📂 Estrutura

O projeto possui duas versões principais:

### ✅ `distance_camera2aruco.py`
- Versão base do projeto.
- Detecta ArUco, calcula distância e ângulos, exibe os eixos na imagem.

### ✅ `distance_camera2aruco_photo_video.py`
- Versão estendida.
- Inclui as **funções**:
  - `save_frame_image()`: salva imagem com tecla `s`.
  - `record_video_loop()`: grava vídeo com tecla `r` até pressionar `r` novamente.
- Os vídeos e imagens são salvos em `./output_media`.

---

## 🚀 Como usar

1. Certifique-se de que o arquivo de calibração da câmera (`calib_rt1.npz`) está presente no caminho definido:
   ```python
   dados1 = np.load('/pasta/calib_rt1.npz')
   ```
2. Execute o script:
   ```bash
   python3 distance_camera2aruco_photo_video.py
   ```

3. Durante a execução:
   - Pressione `s` para salvar uma imagem (`frame_1.jpg`, ...);
   - Pressione `r` para iniciar a gravação e `r` novamente para parar (`output_video.avi`);
   - Pressione `q` para encerrar o programa.

---

## 🧪 O que o código faz

- Lê imagens transmitidas via **mensageria AMQP** (IS framework).
- Corrige distorção da lente usando parâmetros de calibração.
- Detecta marcadores ArUco com ID 9.
- Estima pose com `cv2.aruco.estimatePoseSingleMarkers`.
- Converte rotação para **quaternion** e depois para **ângulos de Euler** (roll, pitch, yaw).
- Desenha eixos 3D sobre o marcador detectado com `cv2.drawFrameAxes`.

---

## 📁 Saída

Os arquivos gerados são salvos automaticamente em:

```
output_media/
├── frame_1.jpg
├── frame_2.jpg
└── output_video.avi
```

---

## 🧠 Conceitos aplicados

- **ArUco Markers**: padrões visuais quadrados utilizados para rastreamento e estimação de pose.
- **Pose Estimation**: cálculo da posição e orientação de um objeto 3D a partir de uma imagem 2D.
- **Calibração de câmera**: remoção da distorção da lente usando matriz intrínseca e coeficientes.
- **Quaternions e ângulos de Euler**: representação e interpretação de rotação no espaço 3D.

---

## 🖼️ Exemplo visual

Na visualização da câmera:

- O marcador ArUco com ID `9` é detectado.
- Eixos 3D (X em vermelho, Y em verde, Z em azul) são desenhados no centro do marcador.
- A imagem é exibida em tempo real com opção de salvar e gravar.

---

## 📚 Referências

- OpenCV ArUco Module: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
- Calibração de câmeras: https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html
- IS-Wire (Intelligent Space middleware): https://github.com/labvis-ufpa/is-wire

---

## 👨‍🏫 Créditos

Desenvolvido como parte das atividades da disciplina **Visão Computacional Aplicada** no **LabSEA**.

---

## 📬 Contato

Para dúvidas ou sugestões, entre em contato com o time do **LabSEA**.
