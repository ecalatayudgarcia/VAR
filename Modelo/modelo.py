import torch
import torch.nn as nn
from torch.utils.data import Dataset
from torchvision import transforms
from PIL import Image
from torch.utils.data import DataLoader
import torch.optim as optim
import pandas as pd
import os
import sys


class ProcesadorDataset(Dataset): ##Esta clase se instancia y prepara los tensores (datos, imagenes y csv)
    #basicamente recorta las imagenes y las pasa a tensor (valores 0.0 a 1.0)
    
    def __init__(self, archivo_csv, directorio_raiz):
        self.datos = pd.read_csv(archivo_csv)
        self.directorio_raiz = directorio_raiz
        
        ##Procesamiento del dataset, recortar imagenes, resize y pasar a 0.0-1.0
        self.transformaciones = transforms.Compose([
            transforms.Lambda(lambda img: img.crop((0, img.height//2, img.width, img.height))),
            transforms.Resize((64, 64)),
            # --- DATA AUGMENTATION SUTIL ---
            # Variamos brillo y contraste un 20% para aguantar sombras de personas
            transforms.ColorJitter(brightness=0.2, contrast=0.2),
            # Rotación de solo 5º y Zoom de solo el 5% (0.95 a 1.05) para no perder las líneas
            transforms.RandomAffine(degrees=5, scale=(0.95, 1.05)),
            # -------------------------------
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5]),
            # Simula oclusiones (como una persona tapando parte del carril)
            # p=0.3 significa que solo se aplica al 30% de las imágenes cada vez
            transforms.RandomErasing(p=0.3, scale=(0.02, 0.1), value=0)
        ])

    def __len__(self):
        return len(self.datos)

    def __getitem__(self, indice):
        ##sobreeescribir esto para poder hacer for imagen,velocidad in dataset
        nombre_img = os.path.join(self.directorio_raiz, self.datos.iloc[indice, 0])
        imagen = Image.open(nombre_img).convert('RGB')
        imagen_tensor = self.transformaciones(imagen)#cargar una imagen y transformarla
        
        giro_bruto = float(self.datos.iloc[indice, 1])#cargar el label para esa foto 
        etiqueta = torch.tensor([giro_bruto], dtype=torch.float32) #convertir el escalar a tensor
        #la cnn trabaja con tensores, aunque solo vaya a generar 1 escalar, tiene que hacerlo como vector de 1 dimension
            
        return imagen_tensor, etiqueta


class ModeloPiloto(nn.Module):
    def __init__(self):
        super().__init__()
        
        #Una imagen es un vector de 64x64x3, esa es la entrada a la CNN
        #Arquitectura de 3 capas con 3 poolings
        self.convoluciones = nn.Sequential(
           
            nn.Conv2d(in_channels=3, out_channels=16, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2), #Devuelve 16 imagenes de 32x32
            
            nn.Conv2d(in_channels=16, out_channels=32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2), #32 imagenes de 16x16
            
            nn.Conv2d(in_channels=32, out_channels=64, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2)  #64 imagenes de 8x8
        )
        
        #el clasificador final requiere aplanar la imagen
        self.decisiones = nn.Sequential(
            nn.Flatten(),
            nn.Linear(4096, 128), #tras los 3 pooling se queda en 8x8x64
            nn.ReLU(),
            # Salida de 1 neurona 
            nn.Linear(128, 1) 
        )

    def forward(self, x):#internamente llama a esta funcion
        #es como el main, solo aplica el cnn+mlp
        x = self.convoluciones(x)
        x = self.decisiones(x)
        return x

if __name__ == '__main__':
    
   


    
    dispositivo = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Entrenando en: {dispositivo}")
    csv_path = '../src/dataset_carrera/registro_conduccion.csv'
    img_dir = '../src/dataset_carrera/'
    
    dataset = ProcesadorDataset(archivo_csv=csv_path, directorio_raiz=img_dir)
    #El DataLoader gestiona el flujo de datos por lotes (batches)
    train_loader = DataLoader(dataset, batch_size=32, shuffle=True)

    ##El .to reserva la memoria de toda la clase (todas las matrices de la red)
    ##En la GPU
    modelo = ModeloPiloto().to(dispositivo)
        
    criterio = nn.MSELoss() # Error Cuadrático Medio para regresión continua
    optimizador = optim.Adam(modelo.parameters(), lr=0.001)

    epochs = 20

    if len(sys.argv) > 1 and sys.argv[1] == '--fine-tuning':
        #En este caso hay que bajar el learning rate
        #ajustar menos los pesos
        #el modelo ya sabe conducir asique no se puede cambiar demasiado si no volvera a fallar
        
        pesos_guardados = torch.load('modelo_base.pth',weights_only=True)
        modelo.load_state_dict(pesos_guardados)
        optimizador = optim.Adam(modelo.parameters(), lr=1e-4)
        epochs = 5
        print("Modelo cargado. Se va a entrenar con fine tuning")

    print("Iniciando entrenamiento...")

    for epoch in range(epochs):
        error_total = 0.0
        for imagenes, etiquetas in train_loader: ##como en la clase del dataset se ha sobreescrito el __getitem__ devuelve imagen,label por cada for (yield)
            #enviar imagenes y labels a la gpu
            imagenes, etiquetas = imagenes.to(dispositivo), etiquetas.to(dispositivo)

            #Limpiar el objeto optimizador
            optimizador.zero_grad()  
            #ejecutar la red
            predicciones = modelo(imagenes)
            #Calcular el error
            loss = criterio(predicciones, etiquetas)
            #Backpropagation
            loss.backward() 
            #Actualizar pesos
            optimizador.step()
            
            error_total += loss.item()

        print(f"Época [{epoch+1}/{epochs}] - Error medio: {error_total/len(train_loader):.4f}")

   
    torch.save(modelo.state_dict(), 'modelo_piloto.pth')
    print("Entrenamiento completado. Archivo 'modelo_piloto.pth' generado.")