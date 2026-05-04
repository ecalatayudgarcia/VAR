import torch
import torch.nn as nn
from torch.utils.data import Dataset
from torchvision import transforms
from PIL import Image
from torch.utils.data import DataLoader
import torch.optim as optim
import pandas as pd
import os


class CircuitoDataset(Dataset): ##Esta clase se instancia y prepara los tensores (datos, imagenes y csv)
    #basicamente recorta las imagenes y las pasa a tensor (valores 0.0 a 1.0)
    
    def __init__(self, archivo_csv, directorio_raiz):
        self.datos = pd.read_csv(archivo_csv)
        self.directorio_raiz = directorio_raiz
        
        # Pipeline de transformaciones matemáticas
        self.transformaciones = transforms.Compose([
            # 1. Recortar la mitad superior (el techo/cielo no aporta información para conducir)
            transforms.Lambda(lambda img: img.crop((0, img.height//2, img.width, img.height))),
            # 2. Redimensionar a una matriz cuadrada de 64x64 píxeles para aligerar la carga
            transforms.Resize((64, 64)),
            # 3. Convertir la matriz de píxeles (0-255) a un Tensor de PyTorch (0.0 - 1.0)
            transforms.ToTensor(),
            # 4. Normalizar los canales RGB
            transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
        ])

    def __len__(self):
        return len(self.datos)

    def __getitem__(self, indice):
        # 1. Cargar la imagen
        nombre_img = os.path.join(self.directorio_raiz, self.datos.iloc[indice, 0])
        imagen = Image.open(nombre_img).convert('RGB')
        
        # 2. Aplicar las transformaciones matemáticas
        imagen_tensor = self.transformaciones(imagen)
        
        # 3. Mapeo Lógico (Regresión a Clasificación)
        # Convertimos los valores del teclado (-1.0, 0.0, 1.0) en índices de clase enteros
        giro_bruto = float(self.datos.iloc[indice, 1])
        etiqueta = torch.tensor([giro_bruto], dtype=torch.float32)
            
        return imagen_tensor, etiqueta


class CerebroPiloto(nn.Module):
    def __init__(self):
        super().__init__()
        
        # FASE A: Extracción de Características Visuales
        # Entrada: Tensor [3 canales, 64 alto, 64 ancho]
        self.convoluciones = nn.Sequential(
            # Capa 1: Detecta bordes simples
            nn.Conv2d(in_channels=3, out_channels=16, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2), # Reduce la imagen a 32x32
            
            # Capa 2: Detecta formas (curvas de la pared)
            nn.Conv2d(in_channels=16, out_channels=32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2), # Reduce a 16x16
            
            # Capa 3: Detecta patrones complejos abstractos
            nn.Conv2d(in_channels=32, out_channels=64, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2)  # Reduce a 8x8
        )
        
        # FASE B: Clasificador Lógico
        # Al llegar aquí, tenemos 64 mapas de características de 8x8 píxeles
        dimensiones_aplanadas = 64 * 8 * 8  # = 4096 variables
        
        self.decisiones = nn.Sequential(
            nn.Flatten(),
            nn.Linear(dimensiones_aplanadas, 128),
            nn.ReLU(),
            # Salida de 1 neurona (valor continuo de velocidad angular)
            nn.Linear(128, 1) 
        )

    def forward(self, x):
        # Flujo de datos hacia adelante (Forward Pass)
        x = self.convoluciones(x)
        x = self.decisiones(x)
        return x

if __name__ == '__main__':
    # 1. Configuración de hardware
    dispositivo = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Entrenando en: {dispositivo}")
    csv_path = '../src/dataset_carrera/registro_conduccion.csv'
    img_dir = '../src/dataset_carrera/'
    
    dataset = CircuitoDataset(archivo_csv=csv_path, directorio_raiz=img_dir)
    #El DataLoader gestiona el flujo de datos por lotes (batches)
    train_loader = DataLoader(dataset, batch_size=32, shuffle=True)

    modelo = CerebroPiloto().to(dispositivo)
    criterio = nn.MSELoss() # Error Cuadrático Medio para regresión continua
    optimizador = optim.Adam(modelo.parameters(), lr=0.001)


    epochs = 20
    print("Iniciando entrenamiento...")

    for epoch in range(epochs):
        error_total = 0.0
        for imagenes, etiquetas in train_loader: ##como en la clase del dataset se ha sobreescrito el __getitem__ devuelve imagen,label por cada for (yield)
            #enviar imagenes y labels a la gpu
            imagenes, etiquetas = imagenes.to(dispositivo), etiquetas.to(dispositivo)

            # Paso 1: Limpiar gradientes anteriores
            optimizador.zero_grad()
            
            # Paso 2: Forward pass (Predicción)
            predicciones = modelo(imagenes)
            
            # Paso 3: Calcular el error
            loss = criterio(predicciones, etiquetas)
            
            # Paso 4: Backpropagation (Calcular derivadas)
            loss.backward()
            
            # Paso 5: Actualizar pesos de las neuronas
            optimizador.step()
            
            error_total += loss.item()

        print(f"Época [{epoch+1}/{epochs}] - Error medio: {error_total/len(train_loader):.4f}")

    # 5. Guardar el "cerebro" resultante
    torch.save(modelo.state_dict(), 'modelo_piloto.pth')
    print("Entrenamiento completado. Archivo 'modelo_piloto.pth' generado.")