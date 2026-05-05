import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import torch
import torch.nn as nn
from torchvision import transforms
from PIL import Image as PILImage


class ModeloPiloto(nn.Module):
    '''
    Aqui solo se declara la misma estructura de la red para poder reservarla en memoria
    y cargar el modelo ya entrenado 
    '''
    def __init__(self):
        super().__init__()

        self.convoluciones = nn.Sequential(
            nn.Conv2d(3, 16, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2, 2),
            nn.Conv2d(16, 32, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2, 2),
            nn.Conv2d(32, 64, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2, 2)
        )
        self.decisiones = nn.Sequential(
            nn.Flatten(),
            nn.Linear(64 * 8 * 8, 128), nn.ReLU(),
            nn.Linear(128, 1) 
        )

    def forward(self, x):
        x = self.convoluciones(x)
        return self.decisiones(x)

##NODO
class PilotoAutonomo(Node):
    '''
    Funcinamiento principal del nodo:
    cargar CNN -> suscribir a la camara -> ver cada foto y predecir una velocidad angular -> aplicarla
    por ahora siempre avanza a 0.15 rd/s asique el modelo solo debe decir para donde girar segun lo que este viendo
    '''
    def __init__(self):
        super().__init__('nodo_piloto_ia')
        
        self.bridge = CvBridge()
        self.get_logger().info("Iniciando carga del modelo")
        self.dispositivo = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.modelo = ModeloPiloto()
        nombre_modelo = 'modelo_piloto.pth'
        self.declare_parameter('archivo_modelo', nombre_modelo)
        self.archivo_modelo =self.get_parameter('archivo_modelo').get_parameter_value().string_value
        self.ruta_modelo = f'/home/ros2_ws/src/race_navigation/race_navigation/{self.archivo_modelo}'
        #ros2 run race_navigation piloto --ros-args -p archivo_modelo:="nombre.pth"
        
        try:
            self.modelo.load_state_dict(torch.load(self.ruta_modelo, map_location=self.dispositivo))
            self.modelo.to(self.dispositivo)
            self.modelo.eval() # MODO EVALUACION 
            self.get_logger().info(f"Modelo cargado en {self.dispositivo}")
        except Exception as e:
            self.get_logger().error(f"Error cargando el modelo: {e}")
            raise e
        ##Hasta aqui se ha cargado el modelo en memoria y se han aplicado los pesos del .pth
        
        self.transformaciones = transforms.Compose([
            transforms.Resize((64, 64)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
        ])
        
       
        self.sub_camara = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.conducir, 
            10
        )
        
        self.pub_cmd_vel = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )
        
        ################VELOCIDAD (POR AHORA CONSTANTE)
        self.velocidad_lineal = 0.25 

    def conducir(self, msg):
        ### CADA VEZ QUE EL ROBOT PUBLICA UN FRAME DE LA CAMARA ###
        try:
            ##Preprocesar la imagen (lo que se hace con las de entrenamiento)
            ##Recortar a la mitad, normalizar colores y convertir a tensor
            imagen_cv = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            alto = imagen_cv.shape[0]
            imagen_recortada = imagen_cv[alto//2:, :]
            imagen_rgb = cv2.cvtColor(imagen_recortada, cv2.COLOR_BGR2RGB)
            imagen_pil = PILImage.fromarray(imagen_rgb)
            tensor_entrada = self.transformaciones(imagen_pil).unsqueeze(0).to(self.dispositivo)
            
            with torch.no_grad():
                salida_logits = self.modelo(tensor_entrada) #LLAMAR AL MODELO Y PREDECIR LA VELOCIDAD ANGULAR
                giro_estimado = salida_logits.item()
                
            #Enviar el mensaje al robot
            msg_cmd = Twist()
            msg_cmd.linear.x = self.velocidad_lineal
            msg_cmd.angular.z = float(giro_estimado)
            self.pub_cmd_vel.publish(msg_cmd)
            self.get_logger().info(f'Giro enviado: {giro_estimado:.2f} rad/s')
            
        except Exception as e:
            self.get_logger().error(f"Error en el bucle de conducción: {e}")

def main(args=None):
    rclpy.init(args=args)
    nodo = PilotoAutonomo()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.get_logger().info("Apagando Piloto IA...")
    finally:
        # Freno de emergencia al apagar
        msg_stop = Twist()
        nodo.pub_cmd_vel.publish(msg_stop)
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()