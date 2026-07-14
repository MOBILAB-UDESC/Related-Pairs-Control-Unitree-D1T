#! /usr/bin/env python3
import math
# Text To Speach
import pyttsx3
import threading

# ROS 2
import rclpy
from rclpy.node import Node                                             
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration  
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient

# Visão Computacional
import cv2
import mediapipe as mp
import time

# Cinematica
#import pinocchio as pin
#import numpy as np

# Webcam
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)                        # configuração de vídeo para minha webcam logitech
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # para utilizar a padrão do notebook: cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 820)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# MediaPipe
BaseOptions = mp.tasks.BaseOptions
VisionRunningMode = mp.tasks.vision.RunningMode
HandLandmarker = mp.tasks.vision.HandLandmarker                # mão
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions  # mão
PoseLandmarker = mp.tasks.vision.PoseLandmarker                # braço
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions  # braço

options_pose = PoseLandmarkerOptions(
    base_options=BaseOptions(
        model_asset_path="./landmarks/pose_landmarker_heavy.task"),
    running_mode=VisionRunningMode.VIDEO,
    num_poses=2
)
options_hand = HandLandmarkerOptions(
    base_options=BaseOptions(
        model_asset_path="./landmarks/hand_landmarker.task"),  
    running_mode=VisionRunningMode.VIDEO,
    num_hands =2
)

# Limites das Juntas
J1_MAX, J1_MIN = 2.35, -2.35
J2_MAX, J2_MIN = 1.57, -1.57
J3_MAX, J3_MIN = 1.57, -1.57
J4_MAX, J4_MIN = 2.35, -2.35
J5_MAX, J5_MIN = 1.57, -1.57
J6_MAX, J6_MIN = 2.30, -2.30


class MinimalPublisher(Node):

    def __init__(self):
        """
          Create a custom node class for publishing messages.
        """
        
        super().__init__('publisher_dedo')  # Inicializa o node com o nome 'publisher_dedo'

        # Cria um publisher no topico /unitree_d1_arm_controller/joint_trajectory
        # útil para o controle do ARM-POSE
        self.publisher_rotacao_punho = self.create_publisher(JointTrajectory, '/unitree_d1_arm_controller/joint_trajectory', 10)
        
        # Cria um client do Action /d1_2f_gripper_controller/gripper_cmd
        # útil para o controle do GRIPPER-HAND
        self._action_client = ActionClient(
            self,
            GripperCommand,
            '/d1_2f_gripper_controller/gripper_cmd'
        )
        
        # Cria uma função que vai chamar o callbck a cada x segundos, salvo no self.timer por segurança.
        # se eu não salvar, o garbage collector do python pode apagar.
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.detecta_dedos_callback)
        
        # landmarks hand e pose
        self.landmarker_hand = HandLandmarker.create_from_options(options_hand)
        self.landmarker_pose = PoseLandmarker.create_from_options(options_pose)

        # biblioteca para tts
        self.tts = pyttsx3.init()
        self.falando = False

        # controle via tts
        self.calibra = True
        self.calibraCont = 0
        self.enviaRos2 = False

        # braço corpo 
        self.corpo = {
            'peito': {'atual': 0, 'max': 0, 'min': 0},
            'ombro': {'atual': 0, 'max': 0, 'min': 0},
            'cotov': {'atual': 0, 'max': 0, 'min': 0},
            'pulso': {'atual': 0, 'max': 0, 'min': 0}
        }
        
        self.xy = {
            'palma': {'x': None, 'y': None},
            'pulso': {'x': None, 'y': None},
            'ombro': {'x': None, 'y': None},
            'cotov': {'x': None, 'y': None},
            'dedao': {'x': None, 'y': None},
            'indic': {'x': None, 'y': None},
            'centr': {'x': None, 'y': None}
        }
        
        # braço robótico
        self.joints = {
            'J1': 0.0,
            'J2': -1.56,
            'J3': 1.56,
            'J4': 0.0,
            'J5': 0.0,
            'J6': 0.0,
        }
        
        # configuração pro Gripper
        self.maior_drt = 1
        self.threshold_drt = 2
        self.distancia_estavel_drt = 0
        position_max_gripper = 0.028
        self.position_min_gripper = -0.008
        self.gap = position_max_gripper - self.position_min_gripper
        self.gripper = 0.0

        # configuração pro Pulso (joint_6)
        self.temp = False
        self.theta_inicial = 0
        self.theta_atual = 0
        self.theta_enviado = 0
        self.percent_punho = 0   # transformação de radianos para o intervalo

        self.indice = 0          # usado para controlar a primeira passagem
        self.timestamp = 0       # controla o tempo
        self.timestamp_anterior = 0
        self.modo = True         # True == Garra, False == Braço
        self.comp_d1 = 50        # o braço robótico tem x cm


    def fala(self, texto):
        if self.falando:
            return  # ignora se já está falando
        self.falando = True
        thread = threading.Thread(target=self.fala_thread, args=(texto,))
        thread.daemon = True
        thread.start()
    
    
    def fala_thread(self, texto):
        tts = pyttsx3.init()
        tts.say(texto)
        tts.runAndWait()
        self.falando = False


    def normalizaValor(self, max_original, min_original, max_custom, min_custom, valor):
        if max_custom == min_custom:
            return min_original
        return min_original + ((valor - min_custom)*(max_original - min_original)) / (max_custom - min_custom)
        
    
    def verificaVazio(self):
        if(
            self.xy['ombro']['x'] is not None and
            self.xy['cotov']['x'] is not None and
            self.xy['pulso']['y'] is not None and
            self.xy['palma']['y'] is not None
        ):
            return True
        else:
            return False


    def giraAtuador(self, duracao=2):
        
        msg = JointTrajectory()
        point = JointTrajectoryPoint()
        
        msg.joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
        point.positions = list(self.joints.values())
        
        point.time_from_start = Duration(sec=duracao)
        msg.points = [point]
        
        self.publisher_rotacao_punho.publish(msg)


    def sendGripperGoal(self, position, max_effort=10.0):
        if not self._action_client.server_is_ready():
            return
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort
    
        self._action_client.send_goal_async(goal_msg)

                
    def desenhaHud(self, img):
        
            cv2.putText(
                img,
                "CALIBRATION: ",
                (10, 20),                   # coordenada
                cv2.FONT_HERSHEY_SIMPLEX,   # fonte
                0.5,                          # tamanho da fonte
                (0, 255, 0),                # verde (BGR!)
                2                           # espessura
            )
            cv2.putText(
                img,
                f"MODE: {'GRIPPER' if self.modo else 'ARM'}",
                (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2                           
            )
            cv2.putText(
                img,
                "ROS2: ",
                (10, 60),                   
                cv2.FONT_HERSHEY_SIMPLEX,   
                0.5,                          
                (0, 255, 0),
                2
            )           
            if(self.calibra):
                cv2.putText(img,"ON", (115, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)        
            else:
                cv2.putText(img,"OFF", (115, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            if(self.enviaRos2):
                cv2.putText(img,"ON", (65, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)        
            else:
                cv2.putText(img,"OFF", (65, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)


    def processaTeclado(self):
        
        key = cv2.waitKey(1) & 0xFF  
        
        if key == 101: # e: troca o modo (gripper/arm)
            if(self.modo == False):    
                self.fala("mode gripper")
                self.modo = True
            else:
                self.fala("mode arm")
                self.modo = False
                                   
        if key == 116: # t: entra no modo calibração
            self.fala("calibration active")
            self.calibra = True
            self.enviaRos2 = False # já desconectada do ros2
            self.temp = False
            self.calibraCont = 0
                
        if key == 117: # u: sai do modo calibração
            self.fala("calibration deactive")
            self.calibra = False
            self.calibraCont = 0
                
        if key == 111: # o: fala/muta com o ros2
            if(self.enviaRos2 == False):
                self.fala("connected with ros2")
                self.enviaRos2 = True    
                self.calibra = False # já desconecta da calibragem
            else:
                self.fala("desconnected with ros2")
                self.enviaRos2 = False


    def detecta_dedos_callback(self):
        """
          Callback function for the timer.Call back function executed periodically by the timer
        """
        
        ret, img = cap.read()
        self.indice += 1
        self.timestamp = int(time.monotonic() * 1000)
        
        if ret:
        
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)   
            altura, largura, _ = img.shape
            
            self.xy['ombro']['x'] = None
            self.xy['ombro']['y'] = None
            self.xy['cotov']['x'] = None
            self.xy['cotov']['y'] = None
            self.xy['pulso']['x'] = None
            self.xy['pulso']['y'] = None
            self.xy['palma']['x'] = None
            self.xy['palma']['y'] = None
            
            result_hand = self.landmarker_hand.detect_for_video(mp_image, self.timestamp)
            
            for i,hand in enumerate(result_hand.hand_landmarks):
            
                if result_hand.handedness[i][0].index == 0: # DIREITA
                    
                    if self.modo == True: # MODO GRIPPER
                    
                        dedao_ = hand[4]
                        indicador_ = hand[8]
                        # controle punho 
                        ponto_1 = hand[0]
                        ponto_2 = hand[5]
                        ponto_3 = hand[17]
                        
                        self.xy['dedao']['x'] = int(dedao_.x * largura)
                        self.xy['dedao']['y'] = int(dedao_.y * altura)
                        
                        self.xy['indic']['x'] = int(indicador_.x * largura)
                        self.xy['indic']['y'] = int(indicador_.y * altura)
                        
                        x1 = int(ponto_1.x * largura)
                        y1 = int(ponto_1.y * altura)
                  
                        x2 = int(ponto_2.x * largura)
                        y2 = int(ponto_2.y * altura)
                        
                        x3 = int(ponto_3.x * largura)
                        y3 = int(ponto_3.y * altura)
                    
                        self.xy['centr']['x'] = int((x1 + x2 + x3)/3)
                        self.xy['centr']['y'] = int((y1 + y2 + y3)/3)
                        
                        # vetor
                        xv = self.xy['dedao']['x'] - self.xy['centr']['x']
                        yv = self.xy['dedao']['y'] - self.xy['centr']['y']
                
                        cv2.circle(img, (self.xy['dedao']['x'], self.xy['dedao']['y']), 3, (0,255,0), -1)
                        cv2.circle(img, (self.xy['indic']['x'], self.xy['indic']['y']), 3, (0,0,255), -1)
                        cv2.circle(img, (x1,y1), 3, (255,0,0), -1)
                        cv2.circle(img, (x2,y2), 3, (255,0,0), -1)
                        cv2.circle(img, (x3,y3), 3, (255,0,0), -1)
                        cv2.circle(img, (self.xy['centr']['x'], self.xy['centr']['y']), 3, (255,255,255), -1)
                        
                        # angulo do punho
                        self.theta_atual = math.atan2(yv, xv)
                        
                        # abertura da garra
                        distancia = ((self.xy['indic']['x'] - self.xy['dedao']['x'])**2 + (self.xy['indic']['y'] - self.xy['dedao']['y'])**2) ** 0.5
                        
                        if self.temp == False or self.indice == 1: 
                            self.maior_drt = distancia
                            self.distancia_estavel_drt = distancia           

                            self.theta_inicial = self.theta_atual  # rotacao punho  
                            self.theta_anterior = self.theta_atual 
                            
                            self.temp = True
                        
                        
                        if abs(distancia - self.distancia_estavel_drt) > self.threshold_drt: # threshold evitando ocilações
                            self.distancia_estavel_drt = distancia
                            
                        total = math.trunc(self.distancia_estavel_drt / self.maior_drt * 100) # valor em %
                        
                        if(total > 100):
                            total = 100;
                            
                        garra = self.position_min_gripper + (self.gap * (total/100)) # valor tratado para o gripper
        
                        
                        delta = self.theta_atual - self.theta_anterior
                        
                        if delta > math.pi:
                            delta -= 2 * math.pi
                        elif delta < -math.pi:
                            delta += 2 * math.pi
                        
                        self.theta_anterior += delta
                        self.theta_enviado = self.theta_anterior - self.theta_inicial
                             
                        if self.theta_enviado > J6_MAX:
                            self.theta_enviado = J6_MAX
                        elif self.theta_enviado < -J6_MAX:
                            self.theta_enviado = -J6_MAX
                        
                        self.joints['J6'] = self.theta_enviado
                        self.percent_punho = (self.theta_enviado + J6_MAX) / (2*J6_MAX)
                        
                        if(self.calibra == False):
                            if(self.enviaRos2 == True):
                                self.sendGripperGoal(garra)  # ENVIA GARRA                
                                self.giraAtuador()            # ENVIA PUNHO
                        
                        print(f'[GARRA][gripper]:: {self.distancia_estavel_drt:.2f} - {total}% - GARRA: {garra}')
                        print(f'[GARRA][punho]:: rotação={self.percent_punho:.2f} - radianos={self.theta_enviado}')
                    
                    else: # MODO ARM
                    
                        self.timestamp += 1
                        result_arm = self.landmarker_pose.detect_for_video(mp_image, self.timestamp)
                
                        pulso = hand[0]             
                        palma = hand[10]
                    
                        self.xy['palma']['x'] = int(palma.x * largura)
                        self.xy['palma']['y'] = int(palma.y * altura)
                        
                        self.xy['pulso']['x'] = int(pulso.x * largura)
                        self.xy['pulso']['y'] = int(pulso.y * altura)
                        
                        self.corpo['pulso']['atual'] = self.xy['pulso']['y'] - self.xy['palma']['y']
                    
                        cv2.circle(img, (self.xy['palma']['x'], self.xy['palma']['y']), 3, (0,255,255), -1)
                        cv2.circle(img, (self.xy['pulso']['x'], self.xy['pulso']['y']), 3, (255,255,0), -1)
               
                        if result_arm.pose_landmarks:
                            
                            braco = result_arm.pose_landmarks[0]
                            
                            ombro = braco[12]
                            cotovelo = braco[14]
                            
                            self.xy['ombro']['x'] = int(ombro.x * largura)
                            self.xy['ombro']['y'] = int(ombro.y * altura)
                            
                            self.xy['cotov']['x'] = int(cotovelo.x * largura)
                            self.xy['cotov']['y'] = int(cotovelo.y * altura)
                            
                            if (self.verificaVazio()):
                                self.corpo['peito']['atual'] = self.xy['ombro']['x'] - self.xy['cotov']['x']
                                self.corpo['ombro']['atual'] = self.xy['ombro']['y'] - self.xy['cotov']['y']
                                self.corpo['cotov']['atual'] = self.xy['cotov']['y'] - self.xy['pulso']['y']
                            
                            cv2.circle(img, (self.xy['ombro']['x'], self.xy['ombro']['y']), 3, (0, 255, 0), -1)
                            cv2.circle(img, (self.xy['cotov']['x'], self.xy['cotov']['y']), 3, (0, 255, 0), -1)
                        
                            
                            if(self.calibra == False):
                                
                                self.joints['J1'] = -self.normalizaValor(J1_MAX, J1_MIN, self.corpo['peito']['max'], 
                                                                    self.corpo['peito']['min'], self.corpo['peito']['atual'])
                                
                                self.joints['J2'] = self.normalizaValor(J2_MAX, J2_MIN, self.corpo['ombro']['max'], 
                                                                    self.corpo['ombro']['min'], self.corpo['ombro']['atual'])
                                
                                self.joints['J3'] = -self.normalizaValor(J3_MAX, J3_MIN, self.corpo['cotov']['max'], 
                                                                    self.corpo['cotov']['min'], self.corpo['cotov']['atual'])
                                
                                self.joints['J5'] = -self.normalizaValor(J5_MAX, J5_MIN, self.corpo['pulso']['max'], 
                                                                    self.corpo['pulso']['min'], self.corpo['pulso']['atual'])
                                
                                if(self.enviaRos2 == True):
                                    self.giraAtuador()
                                    
                            if(self.calibra == True): 
                                if (self.verificaVazio()):
                                    self.corpo['peito']['atual'] = self.xy['ombro']['x'] - self.xy['cotov']['x']
                                    self.corpo['ombro']['atual'] = self.xy['ombro']['y'] - self.xy['cotov']['y']
                                    self.corpo['cotov']['atual'] = self.xy['cotov']['y'] - self.xy['pulso']['y']
                                    self.corpo['pulso']['atual'] = self.xy['pulso']['y'] - self.xy['palma']['y']

                            
                                if(self.calibraCont == 0):
                                    self.calibraCont = 1
                                    if (self.verificaVazio()):
                                        self.corpo['peito']['max'] = self.xy['ombro']['x'] - self.xy['cotov']['x']
                                        self.corpo['ombro']['max'] = self.xy['ombro']['y'] - self.xy['cotov']['y']
                                        self.corpo['cotov']['max'] = self.xy['cotov']['y'] - self.xy['pulso']['y']
                                        self.corpo['pulso']['max'] = self.xy['pulso']['y'] - self.xy['palma']['y']
                                        
                                        self.corpo['peito']['min'] = self.xy['ombro']['x'] - self.xy['cotov']['x']
                                        self.corpo['ombro']['min'] = self.xy['ombro']['y'] - self.xy['cotov']['y']
                                        self.corpo['cotov']['min'] = self.xy['cotov']['y'] - self.xy['pulso']['y']
                                        self.corpo['pulso']['min'] = self.xy['pulso']['y'] - self.xy['palma']['y']
                                else:
                  
                                    if (self.verificaVazio()):                      
                                        self.corpo['peito']['max'] = max(self.corpo['peito']['max'], self.xy['ombro']['x'] - self.xy['cotov']['x'])
                                        self.corpo['ombro']['max'] = max(self.corpo['ombro']['max'], self.xy['ombro']['y'] - self.xy['cotov']['y'])
                                        self.corpo['cotov']['max'] = max(self.corpo['cotov']['max'], self.xy['cotov']['y'] - self.xy['pulso']['y'])
                                        self.corpo['pulso']['max'] = max(self.corpo['pulso']['max'], self.xy['pulso']['y'] - self.xy['palma']['y'])
                                    
                                        self.corpo['peito']['min'] = min(self.corpo['peito']['min'], self.xy['ombro']['x'] - self.xy['cotov']['x'])
                                        self.corpo['ombro']['min'] = min(self.corpo['ombro']['min'], self.xy['ombro']['y'] - self.xy['cotov']['y'])
                                        self.corpo['cotov']['min'] = min(self.corpo['cotov']['min'], self.xy['cotov']['y'] - self.xy['pulso']['y'])
                                        self.corpo['pulso']['min'] = min(self.corpo['pulso']['min'], self.xy['pulso']['y'] - self.xy['palma']['y'])
                                
                            print("\n\n\n")
                            print(f"[BRAÇO] ROTAÇÃO BASE: max[{self.corpo['peito']['max']}] min[{self.corpo['peito']['min']}] atual: {self.corpo['peito']['atual']}")
                            print(f"[BRAÇO] ROTAÇÃO JT-2: max[{self.corpo['ombro']['max']}] min[{self.corpo['ombro']['min']}] atual: {self.corpo['ombro']['atual']}")
                            print(f"[BRAÇO] ROTAÇÃO JT-3: max[{self.corpo['cotov']['max']}] min[{self.corpo['cotov']['min']}] atual: {self.corpo['cotov']['atual']}")
                            print(f"[BRAÇO] ROTAÇÃO JT-4: max[{self.corpo['pulso']['max']}] min[{self.corpo['pulso']['min']}] atual: {self.corpo['pulso']['atual']}")
                       
                
            self.desenhaHud(img)
            
            cv2.imshow("webcam", img)
            
            self.processaTeclado()
                    
            
        else:
            print("IMAGEM NÃO ESTÁ SENDO PROCESSADA")


def main(args=None):

    rclpy.init(args=args)
    
    publisher_dedo = MinimalPublisher()# Create an instance of the Minimal Publisher node
    rclpy.spin(publisher_dedo)

    publisher_dedo.destroy_node()      # Destroy the node explicitly
    rclpy.shutdown()                   # Shutdown RO 2 communication
    
    cap.release()                      # Desliga as câmeras
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
