Resumen!!!
SPLCoachMessage representa un mensaje enviado por el entrenador.
- Define un formato de 260 bytes con un encabezado (SPLC), versión, número de equipo, secuencia y mensaje.
- Permite serializar y deserializar mensajes para enviarlos y recibirlos a través de la red.
- Implementa validaciones para asegurarse de que el mensaje recibido es válido.

Según el código en Java, el mensaje tiene el siguiente formato binario:
Campo	Tamaño (bytes)
header	4
version	1
team	1
sequence	1
message	253
Total	260