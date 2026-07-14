# Teleoperação 2D do Braço Unitree D1-T via Visão Computacional (Método RPC)

Repositório do laboratório contendo o script de controle por visão computacional e os modelos pré-treinados do Google MediaPipe para a teleoperação 3D do braço robótico antropomórfico **Unitree D1-T** a partir de imagens bidimensionais (webcam RGB monocular).

## 📌 Sobre o Projeto
Este projeto introduz o método original **Related Pairs Control (RPC)**, que associa pares específicos de pontos-chave do corpo humano a atuadores robóticos individuais, permitindo controle espacial em tempo real sem câmeras de profundidade. A arquitetura de visão é dividida em:
* **Pose Landmarker (Heavy):** Detecção robusta dos pontos do ombro e cotovelo para movimentação espacial do braço.
* **Hand Landmarker:** Detecção de alta precisão dos pontos do pulso e da mão para controle dedicado da garra (abertura, fechamento e rotação).

## 📂 Estrutura do Repositório
* `control_d1t.py`: Nó principal em Python integrando OpenCV, inferência do MediaPipe e comunicação com o ecossistema ROS 2.
* `landmarks/`: Pasta contendo os modelos `.task` do Google MediaPipe (`hand_landmarker.task` e `pose_landmarker_heavy.task`).

## 🚀 Pré-requisitos e Execução
* Ubuntu (22.04 / 24.04) com **ROS 2** (Humble / Jazzy)
* Python 3, OpenCV e Google MediaPipe

Para iniciar o nó de controle de visão no seu workspace:
    python3 control_d1t.py

## 📄 Referência e Citação
Trabalho aceito para apresentação e publicação nos anais do **XXVI Congresso Brasileiro de Automática (CBA 2026)**, São Paulo, SP.  
*Título:* "Related Pairs Control (RPC): A 3D Position Controller for an Anthropomorphic Arm Using Two-Dimensional Images"
