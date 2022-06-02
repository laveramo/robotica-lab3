# Laboratorio 3 Robótica: Cinemática inversa robot Phantom X - ROS
## MATLAB + Toolbox
### Procedimiento
En el script *cinematica_inversa_px1.m* se encuentra una función de matlab que recibe una matriz de transformación homogénea (que representa una postura) y devuelve una matriz q de 2 filas y 4 columnas, las columnas representan las posiciones de los motores en grados y cada fila representa una solución diferente: codo arriba o codo abajo.

La cinemática inversa se realizó a través de un análisis geométrico:

1. En primer lugar, se halló el vector de la posición de la muñeca, restándole a la matriz de la pose del gripper el vector de la longitud del último eslabón. A esta posición se le llama en el código "Posw", donde $Pos_w = [x y z] -l_4[ax ay az] = [x_w y_w z_w]
2. A partir de las proyecciones de esta posición ya se puede hallar $q_1$ (posición de la primera junta)
3. Posteriormente se soluciona geométricamente los 2 eslabones centrales del robot, como un mecanismo 2R-planar, usando la ley del cosenoy teniendo en cuenta las siguientes expresiones:
  $r = sqrt{x_w^2+y_w^2}$
5. 
### Análisis

## Aplicación de Pick and Place

### Procedimiento
Se tenía como objetivo implementar una aplicación de "pick and place" con el robot que consistía en tomar la pieza tipo 1 (ubicada a su derecha) y posicionarla al frente para posteriormente tomar la pieza tipo 2 (ubicada a su izquierda) e insertarla en la pieza anterior.

[![image.png](https://i.postimg.cc/7h5RP2Vh/image.png)](https://postimg.cc/QKrm0VvZ)

Esta aplicación se puede encontrar en el script *pick_and_place.m*. Para lograrlo se siguieron los siguientes pasos:

1. Se crearon 7 matrices de transformación homogéneas que indicaban las posiciones para realizar las rutas del robot: home, punto derecho superior e inferior, punto izquierdo superior e inferior y punto central superior e inferior. Adicionalmente se creó una MTH de la posición de home.

2. Con ayuda de la función `ctraj` de matlab se creó un vector de matrices de transformación homogénea que contienen la trayectoria desde los puntos mencionados anteriormente, es decir, se tiene una trayectoria para ir de home al punto superior derecho, luego al punto inferior derecho, y así sucesivamente.

3. Posteriormente se creó un ciclo que recorre este vector que representa el trayectoria, de forma que en cada "punto" de la trayectoria (que está representado por una MTH) se realiza otro ciclo para mover cada motor del robot según la configuración de los mismos. Para saber esto, se usa la función de la cinemática inversa creada anteriormente `cinematica_inversa_px1(pose)`.

4. Finalmente, en ciertos momentos del ciclo se agrega código que permite mover el gripper cuando sea necesario, además de agregar ciertas pausas entre movimientos.

### Resultados
El código proporcionado en el script *pick_and_place.m* proporciona la visualización del robot a través del toolbox de Peter Corke, de forma que se pueda comprobar por simulación los movimientos del robot.

Adicionalmente, en el siguiente enlace se puede ver el funcionamiento final del robot:

Podemos observar que el robot tiene movimientos bruscos en su trayectoria, esto puede deberse a que no se configuró de manera adecuada el torque para el movimiento de los motores.

Por otro lado, existe una demora entre ciertos puntos de la trayectoria, la cual pudo haberse solucionado modificando los tiempos de pausa en ciclos (haciéndolos más cortos).

## Conclusiones


