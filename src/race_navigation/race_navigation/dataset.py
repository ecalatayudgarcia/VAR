import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import os
import csv

APPEND = True

class NodoRecolector(Node):
    def __init__(self):
        super().__init__('nodo_recolector_datos')
        
        self.giro_actual = 0.0
        self.velocidad_actual = 0.0
        
        self.contador_frames = 0 
        
        self.frecuencia_guardado = 3 # Se guardan 1 de cada 3 frames 
        self.directorio_datos = './src/dataset_carrera'
        self.archivo_csv = f'{self.directorio_datos}/registro_conduccion.csv'
    
        if not os.path.exists(self.directorio_datos):
            os.makedirs(self.directorio_datos)

        
        if os.path.exists(self.archivo_csv) and self.contador_frames > 0:
            self.get_logger().info(f"CSV detectado. Añadiendo datos a partir del frame {self.contador_frames}")
        else:
            self.get_logger().info("Creando dataset NUEVO")
            self.contador_frames = 0 
            with open(self.archivo_csv, 'w', newline='') as f:
                escritor = csv.writer(f)
                escritor.writerow(['nombre_imagen', 'velocidad_angular_z'])
            
     
        self.bridge = CvBridge() #necesario para transformar la imagenes
        
        self.sub_camara = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.procesar_imagen,
            10)
            
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.procesar_comando,
            10)
            
        self.get_logger().info('Recolector iniciado. Conduce el robot para grabar datos.')

    def procesar_comando(self, msg): #cuando el joystick envia una velocidad angular se guarda aqui
        self.giro_actual = msg.angular.z
        self.velocidad_actual = msg.linear.x

    def procesar_imagen(self, msg):
        self.contador_frames += 1
        
        if self.contador_frames % self.frecuencia_guardado != 0:
            return
            
        if self.velocidad_actual == 0.0:
            return
            
        try:
            #Traducir de mensaje de ros2 a matriz de openCV (imagen) y guardar
            imagen_cv = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            nombre_archivo = f'frame_{self.contador_frames}.jpg'
            ruta_absoluta = os.path.join(self.directorio_datos, nombre_archivo)
            cv2.imwrite(ruta_absoluta, imagen_cv)
            
            #GUARDAR en el dataset la velocidad angular asociada a este frame
            with open(self.archivo_csv, 'a', newline='') as f:
                escritor = csv.writer(f)
                escritor.writerow([nombre_archivo, self.giro_actual])
                
            self.get_logger().info(f'Guardado: {nombre_archivo} | Giro: {self.giro_actual}')
            
        except Exception as e:
            self.get_logger().error(f'Error procesando imagen: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    nodo = NodoRecolector()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.get_logger().info('Recolección finalizada por el usuario.')
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()