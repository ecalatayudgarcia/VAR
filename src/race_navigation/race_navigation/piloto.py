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

# ==========================================
# 1. LA TOPOLOGÍA DE LA RED (Debe ser idéntica al entrenamiento)
# ==========================================
class CerebroPiloto(nn.Module):
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

# ==========================================
# 2. EL NODO DE ROS2 (Intermediario entre Sensor y Motor)
# ==========================================
class PilotoAutonomo(Node):
    def __init__(self):
        super().__init__('nodo_piloto_ia')
        
        self.bridge = CvBridge()
        self.get_logger().info("Iniciando carga del modelo de IA...")
        
        # --- CARGA DEL CEREBRO ---
        self.dispositivo = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.modelo = CerebroPiloto()
        
        # OJO: Cambia esto por la ruta absoluta a tu archivo .pth
        self.ruta_modelo = '/home/ros2_ws/race_navigation/race_navigation/modelo_piloto.pth' 
        
        try:
            self.modelo.load_state_dict(torch.load(self.ruta_modelo, map_location=self.dispositivo))
            self.modelo.to(self.dispositivo)
            self.modelo.eval() # Fundamental: Modo lectura, no escritura
            self.get_logger().info(f"Modelo cargado correctamente en {self.dispositivo}!")
        except Exception as e:
            self.get_logger().error(f"Error cargando el modelo: {e}")
            raise e
        
        # --- REGLAS DE VISIÓN (Sin el recorte, lo haremos manual en cv2) ---
        self.transformaciones = transforms.Compose([
            transforms.Resize((64, 64)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
        ])
        
        # --- CONEXIONES ROS2 ---
        self.sub_camara = self.create_subscription(Image, '/camera/image_raw', self.conducir, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.velocidad_lineal = 0.15 # Velocidad constante de avance
        self.get_logger().info("Piloto IA listo. Esperando imágenes de la cámara...")

    def conducir(self, msg):
        try:
            # 1. Extraer foto de ROS2
            imagen_cv = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 2. Recortar físicamente el techo (la mitad superior)
            alto = imagen_cv.shape[0]
            imagen_recortada = imagen_cv[alto//2:, :]
            
            # 3. Pasar de BGR (OpenCV) a RGB (PIL) para PyTorch
            imagen_rgb = cv2.cvtColor(imagen_recortada, cv2.COLOR_BGR2RGB)
            imagen_pil = PILImage.fromarray(imagen_rgb)
            
            # 4. Aplicar tensores y añadir dimensión de Batch (1, 3, 64, 64)
            tensor_entrada = self.transformaciones(imagen_pil).unsqueeze(0).to(self.dispositivo)
            
            # 5. INFERENCIA (Pensar)
            with torch.no_grad():
                salida_logits = self.modelo(tensor_entrada)
                # Extraemos directamente el número continuo del tensor
                giro_estimado = salida_logits.item()
                
            # 6. TRADUCCIÓN A FÍSICA (Inyección directa)
            msg_cmd = Twist()
            msg_cmd.linear.x = self.velocidad_lineal
            msg_cmd.angular.z = float(giro_estimado)
                
            # 7. PUBLICAR COMANDO Y LOGUEAR
            self.pub_cmd_vel.publish(msg_cmd)
            self.get_logger().info(f'Acción IA (Volante analógico): {giro_estimado:.2f} rad/s')
            
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