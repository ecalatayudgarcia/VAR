//libreria diagramas
#import "@preview/fletcher:0.5.8" as fletcher: diagram, node, edge
#import fletcher.shapes: house, hexagon, pill, octagon

#let blob(pos, label, tint: white, ..args) = node(
	pos, align(center, label),
	width: 28mm,
	fill: tint.lighten(60%),
	stroke: 1pt + tint.darken(20%),
	corner-radius: 5pt,
  ..args,
)

#set page(
  paper: "us-letter",
  margin: (x: 4cm, y: 3cm),
  header: context {
    // No mostrar encabezado en la página 1 (portada)
    if counter(page).at(here()).first() > 1 [
      #set text(9pt)
      #grid(
        columns: (1fr, 1fr),
        [VAR - Práctica 2],
        align(right, emph[Enrique Calatayud García, Daniel Mesas Navas])
      )
    ]
  },
  footer: context {
    if counter(page).at(here()).first() > 1 [
      #align(center, counter(page).display())
    ]
  }
)

#set text(font: "Linux Libertine", size: 11pt, lang: "es")
#set par(justify: true, spacing: 1em)

#show heading: set block(above: 1.4em, below: 1em)
#show figure: set block(above: 2em, below: 2em)

// --- ESTILO DE CÓDIGO Y TERMINAL ---
#let terminal(title: "bash -- 80x24", body) = {
  block(
    width: 100%,
    fill: gray.lighten(80%),
    stroke: 0.5pt + gray,
    radius: 6pt,
    clip: true,
  )[
    // Barra superior estilo terminal
    #block(
      width: 100%,
      fill: gray.lighten(60%),
      inset: (x: 8pt, y: 4pt),
    )[
      #stack(
        dir: ltr,
        spacing: 4pt,
        circle(radius: 2.5pt, fill: rgb("#ff5f56")),
        circle(radius: 2.5pt, fill: rgb("#ffbd2e")),
        circle(radius: 2.5pt, fill: rgb("#27c93f")),
        h(1fr),
        text(fill: gray.darken(20%), size: 8pt, weight: "bold", title),
        h(1fr),
      )
    ]
    // Cuerpo del código
    //#padding(x: 10pt, y: 8pt, raw(body, lang: "bash", block: true))
  ]
}

// Configuración para fragmentos de C++
#show raw.where(lang: "cpp"): it => {
  block(
    width: 100%,
    fill: rgb("#f8f8f8"),
    inset: 10pt,
    stroke: (left: 2pt + blue),
    it
  )
}

// --- PORTADA ---
#align(center)[
  #v(3cm)
  #text(20pt, weight: "bold")[Visión Artificial y Robótica] \
  #text(18pt)[2025-2026]
  
  #v(1cm)
  #text(16pt, weight: "bold")[Práctica 2. Carrera de Robots]
  
  #v(2cm)
  #text(12pt)[
    Enrique Calatayud García \
    Daniel Mesas Navas
  ]
  
  #v(1cm)
  #datetime.today().display("[day] de [month repr:long] de [year]")
  
  #v(1.5cm)
  // Reemplaza con tu ruta de imagen
  #image("logo_ua.jpg", width: 75%) 
]

#pagebreak()

// --- ÍNDICE ---

#outline(indent: auto)
#pagebreak()

// --- CONTENIDO ---
#set heading(numbering: "1.")

= Introducción

Para esta práctica se propone desarrollar un sistema de navegación autónoma que permita a un robot turtlebot3 completar una vuelta a un circuito en el menor tiempo posible.

Como segundo objetivo se propone implementar un sistema de detección de objetos mediante el cual el robot sea capaz de distinguir diferentes obstáculos en el circuito, además de ser capaz de esquivarlos en tiempo real.

La arquitectura de la solución se ha construido usando ROS @quigley2009ros, el cual permite gestionar la comunicación de los nodos. El entorno del robot y la toma de datos se han simulado utilizando Gazebo @koenig2004design.  

Se puede consultar el vídeo que contiene la demostración de la solución en el siguiente enlace: #link("https://www.youtube.com/watch?v=DNDud_Wnras&list=LL&index=1")[#underline([Vídeo demostración])]

= Propuesta

Se propone desarrollar una red neuronal convolucional (CNN) la cual reciba la información de la cámara montada por el robot y produzca como salida la velocidad angular que debería tomar el robot en un fotograma determinado.

Para el reconocimiento de obstáculos, se propone integrar YOLO @redmon2016you mediante la librería de Ultralytics @Jocher_Ultralytics_YOLO_2023. Esta red neuronal será la encargada de identificar los diferentes objetos. 

Se han implementado dos soluciones finales usando estas dos tecnologías de distinta forma: Una en la cual todo el control del robot pertenece a la CNN a excepción de cuando se detecta una persona, donde se reduce la velocidad para poder esquivar los obstáculos con mayor tasa de éxito; y otra donde se adopta un enfoque híbrido, quitándole el control a la CNN cuando se detecta una persona y controlando el robot mediante lógica. Se detallarán más adelante.

Cabe destacar que, debido a limitaciones temporales y de hardware, la CNN ha sido entrenada para esquivar en tiempo real únicamente a personas (@modelo_visitor). 

#figure(
  image("malevisitoronphone.png",width:75%),
  caption: [Modelo de persona utilizado.]
)<modelo_visitor>

== Arquitectura del Sistema
Se han desarrollado tres nodos principales para la implementación de la solución, representados en la @diagrama_solucion:

#figure(
  diagram(
    spacing: 8pt,
    cell-size: (8mm, 10mm),
    edge-stroke: 1pt,
    edge-corner-radius: 5pt,
    mark-scale: 70%,

    blob((0.5,0.5), [*recolector*], tint: yellow, shape: hexagon),
    edge((0.5,0.5),(0.5,1.5)),
    edge((0,1.5),(1,1.5)),
    edge((1,1.5),(1,2.5),"-|>"),
    edge((0,1.5),(0,2.5),"-|>"),
    blob((0,2.5), [Frame_i], tint: orange),
    blob((1,2.5), [Velocidad angular_i], tint: orange),
    
    edge((0,3.5),(1,3.5)),
    edge((0.5,4.5),(0.5,3.5)),
    edge((0,2.5),(0,3.5),"-|>"),
    edge((1,2.5),(1,3.5),"-|>"),
    blob((0.5,4.5), [*modelo*], tint: red, shape: hexagon),

    
    edge((0.5,4.5),(3.5,4.5), "-|>"),
    //edge((0,2.8), (0,4)),

    //edge((0,3), "l,uu,r", "--|>"),
    //edge((0,1), (0, 0.35), "r", (1,3), "r,u", "-|>"),
    //edge((1,2), "d,rr,uu,l", "--|>"),

    blob((3.5,2.5), [*piloto*], tint: green, shape: hexagon),
    blob((4.5,4.5), [/camera/image_raw], tint: orange),
    blob((3,4.5), [modelo_piloto], tint: blue, ),
    edge((3.5,2.5),(3.5,3.5)),
    edge((3,3.5),(4.5,3.5)),
    edge((3,3.5),(3,4.5),"<|-"),
    edge((4.5,3.5),(4.5,4.5),"<|-"),

    blob((3.5,1), [Velocidad angular], tint: gray, ),
    edge((3.5,1),(3.5,2.5),"<|-"),

  ),
  caption: [Arquitectura de la solución. Los nodos son las figuras hexagonales.]
)<diagrama_solucion>

El nodo dataset es el encargado de grabar el recorrido del robot: guarda la imagen de cada frame y la velocidad angular que tomó el robot en ese momento y genera un archivo CSV que relaciona ambos. El recorrido fue grabado utilizando un joystick, puesto que la velocidad angular que envía es más precisa que la del teclado.

El nodo modelo se sitúa fuera de ROS y su función es entrenar a la CNN con la arquitectura de la @tabla_cnn leyendo los datos del archivo csv. Este nodo guardará los pesos finales de la red neuronal en un archivo llamado `modelo_piloto.pth`. Para desarrollar la CNN se ha usado PyTorch @Ansel_PyTorch_2_Faster_2024.


#figure(
  diagram(
    spacing: 8pt,
    cell-size: (8mm, 10mm),
    edge-stroke: 1pt,
    edge-corner-radius: 5pt,
    mark-scale: 70%,

    blob((0,0), [Input \ (64x64x3)], tint: yellow, shape: octagon),
    edge((0,0),(2,0),"-|>"),
    blob((1.5,0), [Conv2D \ (64x64x16)], tint: blue),
    edge("-|>"),
    blob((3,0), [MaxPool2D \ (32x32x16)], tint: gray),
    edge("-|>"),
    blob((4.5,0), [Conv2D \ (32x32x32)], tint: blue),
    edge("-|>"),
    blob((4.5,1.5), [MaxPool2D \ (16x16x32)], tint: gray),
    edge("-|>"),
    blob((3,1.5), [Conv2D \ (16x16x64)], tint: blue),
    edge("-|>"),
    blob((1.5,1.5), [MaxPool2D \ (8x8x64)], tint: gray),
    edge("-|>"),
    blob((0,1.5), [Linear \ (4096,128)], tint: red),
    edge("-|>"),
    blob((0,3), [Linear \ (128,1)], tint: red),
    edge("-|>"),
    blob((1.5,3), [Output], tint: yellow, shape:octagon),

  ),
  caption: [Arquitectura de la CNN.]
)<tabla_cnn>

== Versiones del piloto

La primera versión consiste en un enfoque que prioriza la simplicidad de la solución y deja el control total a la CNN durante todo el recorrido. Cuando el robot se encuentra con personas, la CNN las reconoce y comienza a girar. Como la red no infiere velocidad lineal, mientras YOLO esté detectando a una persona se ha optado por reducir la velocidad del robot drásticamente.

El principal problema de este enfoque es que si los obstáculos están en posiciones complicadas, como curvas (ver @curva), la CNN eventualmente dejará de ver el obstáculo y tratará de reincorporarse al centro del carril, causando ocasionalmente el choque del robot. 

#figure(
    grid(
        columns: (auto, auto),
        rows:    (auto, auto),
        gutter: .2em,
        [ #image("curva.png",   width: 100%) ],
        [ #image("curva2.png", width: 100%) ],
    ),
    caption: [Curva con obstáculo.]
) <curva>

La segunda versión implementa una arquitectura de control híbrido que interrumpe temporalmente las inferencias de la CNN al detectar una persona, delegando el control lateral a un algoritmo basado en estados.  Cuando la red YOLO detecta una persona, el sistema analiza la proporción que ocupa su bounding box respecto al área total de la imagen, así como su posición vertical. Este análisis actúa como un filtro espacial que garantiza que la maniobra de evasión únicamente se active cuando la presencia de la persona suponga un riesgo de colisión inminente, descartando así detecciones lejanas que no requieran intervención.

Una vez superado este filtro espacial, el sistema procede a determinar la trayectoria de evasión óptima evaluando el centroide del obstáculo en el eje horizontal. Si la persona se encuentra ubicada en la mitad derecha del fotograma, la lógica infiere que la vía libre transcurre por la izquierda, fijando en consecuencia una velocidad angular de evasión positiva y viceversa. 

Finalmente, con la dirección de evasión validada, se activa una bandera de estado (flag) que desencadena la ejecución de la maniobra, controlada por un contador de fotogramas. Esta secuencia cinemática se estructura en tres etapas: un primer giro de evasión para apartar al robot de la trayectoria de colisión, seguido de un avance recto en diagonal que permite rebasar el obstáculo manteniendo el margen de seguridad, y un giro final de corrección destinado a reincorporar el vehículo al centro de su carril original. Una vez agotados los fotogramas estipulados para esta secuencia temporal, el control del robot retorna a la CNN.

= Experimentación

== Construcción del dataset

Durante el desarrollo, se ha experimentado con diferentes datasets. La primera vuelta completa al circuito (sin obstáculos) fue lograda únicamente con un total de 8800 frames. Sin embargo, al ser todos los frames sin obstáculos, la CNN nunca había visto un píxel que no fuera gris, por tanto, era incapaz de esquivar ningún obstáculo. Además, se observó que frente a perturbaciones menores que desviaban al robot del centro del carril, el modelo carecía de la capacidad de generalización necesaria para corregir su trayectoria, resultando inevitablemente en una colisión.

Para solucionar este último efecto, se reforzó el dataset para evitar situaciones de choque, añadiendo 3000 frames en los que el robot era puesto cerca de una pared y dirigido hasta volver al centro del circuito. Esta versión aún no era capaz de esquivar personas, pero sí aumentó notablemente la consistencia del robot. Estos frames permitirán que cuando se esquive a personas, la CNN sepa volver al recorrido por el centro del circuito, el cual compone el 75% del dataset.

Por último, a la versión del dataset se le añadieron *más de 20.000 frames* del circuito con personas en diversas posiciones. El objetivo era proporcionar a la CNN  suficientes ejemplos de esquives para que supiese inferir el giro correcto en cada caso. Aun así, el problema de la @curva sigue presente.

La versión final del dataset se compone de un total de 32.026 frames, siendo (aproximadamente) un 30% circuito vacío por el centro, 1% corrección de choques contra paredes/curvas y un 69% situaciones de esquive de obstáculos.

== Velocidad máxima

Cuando no hay obstáculos, la velocidad lineal es constante, lo que permite experimentar con diferentes valores. El principal desafío es que la solución sea determinista y siempre complete una vuelta al circuito. El problema es que esto depende de los frames por segundo a los que se ejecute el simulador. Por tanto, se han hecho las siguientes comparaciones para determinar cuál es la mejor versión en cuanto a fiabilidad/velocidad. Se ha ejecutado con cada velocidad 5 veces.


#figure(
  table(
    columns: (1fr, 1fr, 1fr),
    align: center,
    stroke: (x, y) => (
      left: if x > 0 { 0.5pt + black } else { none },
      top: if y == 1 {1pt} else if y > 0 { 0.5pt + black } else { none },
    ),
    [*Velocidad*], [*Tiempo (MM:SS)*], [*Vueltas completadas*], 
    
    [0.90 ], [-], [0%],
    [0.85 ], [01:50], [75%],
    [0.80], [01:55], [75%],
    [0.75 ], [02:00], [100%], 
    [0.70], [02:09], [100%]

  ),
  caption: [Velocidad lineal y tasa de éxito.]
) <velocidades>

A velocidades superiores de 0.90 el robot no es capaz de completar ninguna vuelta. A velocidades inferiores a 0.75, el robot completa siempre el circuito, por lo que es la mejor velocidad estable con esta arquitectura.

== Comparación de pilotos

Con el objetivo de evaluar qué modelo de piloto esquiva mejor los obstáculos se han propuesto dos versiones del circuito. El primer circuito que se va a evaluar es el de la @facil. Este es teóricamente más fácil ya que la mayoría de los obstáculos se sitúan en rectas, donde hipotéticamente la CNN tiene más tiempo para reaccionar.

#figure(
  image("obstaculos_facil.png", width: 75%),
  caption: [Circuito "fácil".]
)<facil>

La versión de la @dificil supone un desafío mayor, puesto que los obstáculos se sitúan en curvas, donde el robot tiene menos tiempo para reaccionar y, además, es más probable que se produzca el efecto anteriormente mencionado en la @curva.

#figure(
  image("obstaculos_dificil.png", width: 75%),
  caption: [Circuito "difícil".]
)<dificil>

#figure(
  table(
    columns: (1fr, 1fr, 1fr),
    align: center,
    stroke: (x, y) => (
      left: if x > 0 { 0.5pt + black } else { none },
      top: if y == 1 {1pt} else if y > 0 { 0.5pt + black } else { none },
    ),
    [*Métrica*], [*Piloto CNN*], [*Piloto Híbrido*],
    
    // Fila separadora visual
    table.cell(colspan: 3, fill: gray.lighten(80%), [*Circuito Fácil (11 obstáculos)*]),
    
    [Colisiones], [3], [2],
    [Tasa de éxito], [72.7%], [81.8%],
    
    // Fila separadora visual
    table.cell(colspan: 3, fill: gray.lighten(80%), [*Circuito Difícil (17 obstáculos)*]),
    
    [Colisiones], [4], [4],
    [Tasa de éxito], [76.4%], [76.4%]
  ),
  caption: [Comparativa de rendimiento en la evasión de obstáculos.]
) <tabla_comparativa>

A primera vista, los resultados parecen sugerir que las dos versiones son similares en rendimiento, siendo ligeramente mejor la solución híbrida en la versión fácil. Esto es debido a que, en ciertas posiciones en las rectas, la CNN parece ignorar al obstáculo, probablemente debido a falta de dataset.

Sin embargo, en la versión difícil, aunque ambas versiones tengan la misma tasa de éxito, la versión híbrida condujo al robot por el callejón sin salida del centro de la figura, debido a que la lógica de programación no puede saber la trayectoria correcta del circuito.

== Data augmentation

Se experimentó utilizando data augmentation, concretamente modificando los valores de brillo, contraste, rotación y zoom. La versión no mostró mejora evidente, por tanto, queda descartada por complicar la solución y no aportar nada.

= Conclusiones

El desarrollo de este sistema de navegación autónoma ha permitido validar la 
viabilidad de utilizar redes neuronales y visión artificial para el control de un 
vehículo en un entorno simulado. La experimentación demostró que la clonación de comportamiento (mediante la CNN) es altamente efectiva para el seguimiento continuo de carril, siempre y cuando el *dataset* incluya suficientes fotogramas de recuperación para mitigar la desviación de estado. 

Respecto a la evasión de obstáculos, se ha evidenciado el compromiso existente entre un sistema puramente reactivo y uno basado en reglas. Mientras que el piloto híbrido mejora la tasa de éxito en situaciones predecibles (rectas), su rigidez lógica le impide tener contexto espacial a largo plazo, llegando a guiar al robot por rutas sin salida en zonas complejas. En base a los resultados, la solución más consistente fue limitar la velocidad lineal a 0.75 m/s, lo que proporciona el equilibrio óptimo entre el cumplimiento del objetivo temporal y la estabilidad de las inferencias.

== Posibles mejoras
Como líneas de trabajo futuro y posibles mejoras al sistema actual, se proponen las siguientes aproximaciones:

- *Refinamiento del Dataset:* Ampliar el volumen de datos y experimentar con diferentes distribuciones y porcentajes.

- *Reducir tamaño de las personas:* Actualmente, su altura permite que la red YOLO las detecte por encima de los muros.

- *Otras arquitecturas CNN:* Explorar arquitecturas alternativas que integren información temporal (por ejemplo, recibiendo una ventana de los últimos $N$ fotogramas) o información multimodal (fusionando la imagen de la cámara con los datos del sensor LiDAR).

- *Inferir la Velocidad Lineal:* Modificar la capa de salida de la CNN para que no solo infiera la velocidad angular, sino que también prediga la velocidad lineal óptima (End-to-End).

#pagebreak()

= Referencias
#bibliography("referencias.bib", style: "apa", title: "Bibliografía")