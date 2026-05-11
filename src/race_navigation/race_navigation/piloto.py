import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan 
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import torch
import torch.nn as nn
from torchvision import transforms
from PIL import Image as PILImage
from ultralytics import YOLO # Librería de detección de objetos
import numpy as np # Útil para procesar los datos del láser
from rclpy.qos import QoSProfile, ReliabilityPolicy

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
        self.get_logger().info("Iniciando carga del modelo IA y YOLO")
        self.dispositivo = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        # --- CARGA DEL MODELO CNN ORIGINAL ---
        self.modelo = ModeloPiloto()
        nombre_modelo = 'modelo_bomba.pth' 
        self.declare_parameter('archivo_modelo', nombre_modelo)
        self.archivo_modelo = self.get_parameter('archivo_modelo').get_parameter_value().string_value
        self.ruta_modelo = f'/home/ros2_ws/src/race_navigation/race_navigation/{self.archivo_modelo}'
        
        try:
            self.modelo.load_state_dict(torch.load(self.ruta_modelo, map_location=self.dispositivo))
            self.modelo.to(self.dispositivo)
            self.modelo.eval() # MODO EVALUACION 
            self.get_logger().info(f"Modelo cargado en {self.dispositivo}")
        except Exception as e:
            self.get_logger().error(f"Error cargando el modelo: {e}")
            raise e
        ##Hasta aqui se ha cargado el modelo en memoria y se han aplicado los pesos del .pth

        # Cargamos el modelo de YOLO
        self.yolo = YOLO('yolov8n.pt') # Se descargará la primera vez
        
        self.transformaciones = transforms.Compose([
            transforms.Resize((64, 64)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
        ])
        
        self.sub_camara = self.create_subscription(Image, '/camera/image_raw', self.conducir, 10)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Nos suscribimos al tópico REAL (/scan) usando el perfil QoS
        self.sub_laser = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.leer_laser, 
            qos_profile
        )
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        ################VELOCIDAD (POR AHORA CONSTANTE)
        self.velocidad_lineal = 0.55 
        
        # Control de maniobras de adelantamiento
        self.maniobra_activa = False
        self.frames_maniobra = 0
        self.giro_esquiva = 0.0
        
        # Almacén para los datos del láser
        self.hueco_izq = 0.0
        self.hueco_der = 0.0

    def leer_laser(self, msg):
        '''
        Callback para procesar los datos del LIDAR.
        Divide el escaneo en dos mitades para medir dónde hay más espacio.
        '''
        # Limpiamos los datos de infinitos (lecturas fuera de rango)
        lecturas = np.array(msg.ranges)
        lecturas = np.where(np.isinf(lecturas) | np.isnan(lecturas), msg.range_max, lecturas)

        total_rayos = len(lecturas)
        cuarto = total_rayos // 4 # Cogemos solo un 25% de los rayos (90 grados)

        # El rayo [0] es el frente. 
        # Del [0] al [cuarto] es el cono delantero IZQUIERDO.
        # Los últimos rayos del array ([-cuarto:]) es el cono delantero DERECHO.
        self.hueco_izq = np.mean(lecturas[0:cuarto])
        self.hueco_der = np.mean(lecturas[-cuarto:])

    def conducir(self, msg):
        ### CADA VEZ QUE EL ROBOT PUBLICA UN FRAME DE LA CAMARA ###
        try:
            ##Preprocesar la imagen (lo que se hace con las de entrenamiento)
            ##Recortar a la mitad, normalizar colores y convertir a tensor
            imagen_cv = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            ancho_imagen = imagen_cv.shape[1]
            alto = imagen_cv.shape[0]
            
            # 1. VISION YOLO
            resultados_yolo = self.yolo.predict(imagen_cv, classes=[0], verbose=False) # Devuelve array con el resultado para cada imagen, como es solo un frame pues solo devuelve un elemento
            persona_detectada = False
            limite_izq = ancho_imagen * 0.30
            limite_der = ancho_imagen * 0.70
            cv2.line(imagen_cv, (int(limite_izq), 0), (int(limite_izq), alto), (255, 0, 0), 2)
            cv2.line(imagen_cv, (int(limite_der), 0), (int(limite_der), alto), (255, 0, 0), 2)
            
            if len(resultados_yolo[0].boxes) > 0:
                """ Cogemos el resultado de la primera imagen (y única) [0], la primera persona detectada [0] (una foto puede tener 10 personas)
                , cogemos el formato de coordenadas [x_ini, y_ini, x_fin, y_fin] para la primera persona, xyxy es una matriz 
                [[persona1_coordX, persona1_coordY, persona1_coordX2, persona1_coordY2],  [persona2_coordX, persona2_coordY, persona2_coordX2, persona2_coordY2]]
                , el .cpu() es porque YOLO calcula todo en la tarjeta gráfica (VRAM), y esto mueve el dato a la memoria RAM, y luego por último 
                con .numpy() convertimos el dato interno de PyTorch en un array estándar de Python"""
    
                caja = resultados_yolo[0].boxes[0].xyxy[0].cpu().numpy()
                ancho_caja = caja[2] - caja[0]  # x_fin - x_ini
                y_abajo_caja = caja[3]
                centro_persona_x = (caja[0] + caja[2]) / 2.0

                cv2.circle(imagen_cv, (int(centro_persona_x), int(y_abajo_caja)), 8, (0, 0, 255), -1)
        
                # Si la persona está cerca (>15% de la pantalla)
                if ancho_caja > (ancho_imagen * 0.15) and y_abajo_caja > (alto * 0.5):
                    if centro_persona_x > limite_izq and centro_persona_x < limite_der:
                        persona_detectada = True
                        cv2.putText(imagen_cv, "¡PELIGRO!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    else:
                        if not self.maniobra_activa:
                            cv2.putText(imagen_cv, "IGNORANDO (Arcen)", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # MOSTRAMOS LA PANTALLA EN TIEMPO REAL
            cv2.imshow("Ojos del Robot", imagen_cv)
            cv2.waitKey(1) # Necesario para que la ventana de cv2 se actualice
            
            # 2. INICIO DE LA MANIOBRA DE ADELANTAMIENTO
            # Si vemos persona y NO estábamos ya esquivando, disparamos la macro
            if persona_detectada and not self.maniobra_activa:
                self.maniobra_activa = True
                self.frames_maniobra = 0
                
                """El láser escanea el entorno y calcula la distancia media a los obstáculos. 
                Si la media de espacio libre a la izquierda es mayor que a la derecha, giramos a la izquierda."""
                if self.hueco_izq > self.hueco_der:
                    self.giro_esquiva = 0.7  # Izquierda libre -> Volantazo Izquierda
                else:
                    self.giro_esquiva = -0.7 # Derecha libre -> Volantazo Derecha
                    
                self.get_logger().warn('¡OBSTÁCULO! Iniciando macro de adelantamiento ciego con láser...')

            # 3. EJECUCIÓN DE LA MANIOBRA CIEGA (Sobreescribe a la CNN)
            msg_cmd = Twist()
            
            if self.maniobra_activa:
                self.frames_maniobra += 1
                msg_cmd.linear.x = 0.45 # Vamos más despacio al esquivar
                
                if self.frames_maniobra < 15:
                    # FASE 1: Apartarse del obstáculo
                    msg_cmd.angular.z = self.giro_esquiva
                elif self.frames_maniobra < 90:
                    # FASE 2: Ir recto en paralelo (El robot pasa por el lado a ciegas)
                    msg_cmd.angular.z = 0.0
                elif self.frames_maniobra < 105:
                    # FASE 3: Volantazo de regreso al carril
                    msg_cmd.angular.z = -self.giro_esquiva
                else:
                    # FASE 4: Fin. Devolvemos el control
                    self.maniobra_activa = False
                    self.get_logger().info('Adelantamiento completado. Devolviendo control a CNN.')
                
                # Publicamos y abortamos para que la CNN no haga nada
                self.pub_cmd_vel.publish(msg_cmd)
                return 

            # ESTADO NORMAL: CONDUCCIÓN POR RED NEURONAL (Si no hay maniobra activa)
            if not self.maniobra_activa:
                imagen_recortada = imagen_cv[alto//2:, :]
                imagen_rgb = cv2.cvtColor(imagen_recortada, cv2.COLOR_BGR2RGB)
                imagen_pil = PILImage.fromarray(imagen_rgb)
                tensor_entrada = self.transformaciones(imagen_pil).unsqueeze(0).to(self.dispositivo)
                
                with torch.no_grad():
                    salida_logits = self.modelo(tensor_entrada) #LLAMAR AL MODELO Y PREDECIR LA VELOCIDAD ANGULAR
                    giro_estimado = salida_logits.item()
                    
                #Enviar el mensaje al robot
                msg_cmd.linear.x = self.velocidad_lineal
                msg_cmd.angular.z = float(giro_estimado)
                self.pub_cmd_vel.publish(msg_cmd)
            
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