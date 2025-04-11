# Importamos librerías necesarias
import struct
import socket

# Defino la estructura del mensaje
COACH_HEADER = b"SPLC"  # Encabezado esperado
COACH_VERSION = 4       # Versión esperada
COACH_SIZE = 253        # Tamaño del mensaje del coach
COACH_T_SIZE = 260  # Tamaño total del paquete

def decodeCoachMessage(mess):#defino una función para decodificar los datos binarios recibidos del árbitro
    #print(len(mess))
    if len(mess) != 688:#COACH_T_SIZE
        print("Tamaño del mensaje inválido")
        return None

    #Mensaje formato little Endian (bit menos significativo primero)
    unpacked_data = struct.unpack("<4s B B B 681s", mess)   # header (4s)
                                                            # version (B)
                                                            # teamNumber (B)
                                                            # sequenceNumber (B)
                                                            # couchMessage (253s)

    header = unpacked_data[0].decode("utf-8")  # Convertimos los bits de header a string
    version = unpacked_data[1] # version
    teamNumber = unpacked_data[2] # numero de equipo
    sequenceNumber = unpacked_data[3] # numero de sequencia
    message = unpacked_data[4].decode("utf-8", errors="ignore").strip()  # Obtenemos el mensaje del couch como string 
                                                                                 #   eliminando bytes inválidos
    '''print(len(unpacked_data))
        for item in unpacked_data:
            if isinstance(item, bytes) and COACH_HEADER in item:
                print("sip")
            else:
                print("no")'''


    # Ahora Validamos
    if header != COACH_HEADER.decode("utf-8"):
        print("Error: Encabezado inválido")
        return None

    if version != COACH_VERSION:
        print("Versión no soportada")
        return None

    # Retornamos la info del mensaje
    return {
        "header": header,
        "version": version,
        "team": teamNumber,
        "sequence": sequenceNumber,
        "message": message
    }

# Comunicación con el árbitro
class UDPCommunication:
    def __init__(self, coachIp, refereeIp, listeningPort, sendingPort):
        self.coachIp = coachIp  # ip del coach 
        self.refereeIp = refereeIp  # ip del referee
        self.listeningPort = listeningPort  # Puerto del referee
        self.sendingPort = sendingPort  # Puerto del couch

        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # Creamos un socket UDP usando IPv4 para escuchar
        self.returnSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #Creamos otro socket para retornar
        self.sock.bind((self.coachIp,self.listeningPort))# Asociamos el socket de escucha a la ip correspondiente

    def listenReferee(self):
        #Recibe un mensaje del árbitro
        data, addr = self.sock.recvfrom(688)#COACH_T_SIZE
        print(f"Mensaje recibido desde {addr}")
        return data

    def talk2Referee(self, msg):
        #Envía un mensaje al árbitro
        print(f"Enviando mensaje: {msg}")
        self.sock.sendto(msg, (self.refereeIp, self.sendingPort))

#Revisar el contenido del emnsaje del game controller y ver si se encuentra el mensaje del couch
def barridoMensaje(msg):
    print("Primeros 100 bytes")
    print(' '.join(f'{b:02X}' for b in msg[:100])) #Imprimir en formato hexadecimal de 2 digitos los primeros 100 bytes de msg 
    print("\nTexto (primeros 50 bytes como utf-8):")
    print(msg[:50].decode('utf-8', errors='replace'))#Decodeo los primeros 50 a formato string

def buscar_splc_en_mensaje(msg):
    index = msg.find(b'SPLC')#Con la función find, busco si los bytes del header se encuentran dentro del game controler
    if index != -1:#Si lo encuentra...
        print(f"\n'SPLC' encontrado en el índice {index}")#Imprimo la posición del game controller
        if index + COACH_T_SIZE <= len(msg):#Si está el mensaje completo dentro del mensaje del game controller...
            posible_msg = msg[index:index + COACH_T_SIZE]#Recuperamos el mensaje del couch del mensaje enviado por el arbitro con un slice
            try:
                unpacked = struct.unpack('<4s B B B 253s', posible_msg)#Desempaco el mensaje según el formato esperado 
                return {#Devuelvo el json con la información del couch message
                    "header": unpacked[0].decode(),
                    "version": unpacked[1],
                    "team": unpacked[2],
                    "sequence": unpacked[3],
                    "message": unpacked[4].rstrip(b'\x00').decode("utf-8", errors="ignore")
                }
            except Exception as e:
                print("Error desempaquetando:", e)
        else:
            print("Mensaje SPLC truncado o incompleto.")
    else:
        print("No se encontró 'SPLC' en el mensaje.")
    return None

def main():
    coachIp = "192.168.0.241"  # ip de mi pc
    refereeIp = "192.168.0.162"  # ip del árbitro (GameController)

    udpHandler = UDPCommunication(coachIp, refereeIp, 3838, 3939)

    message_bytes = struct.pack('<4s B B B 253s', b'SPLC', 4, 11, 1, b"Vamos Liquadoras!".ljust(253, b'\x00')) # mensaje de prueba

    while True:
        rawData = udpHandler.listenReferee()  # Esperamos el mensaje del árbitro
        udpHandler.talk2Referee(message_bytes)  # Envíamos el mensaje
        decodedMessage = decodeCoachMessage(rawData)  # decodificamos el mensaje
        barridoMensaje(rawData)
        buscar_splc_en_mensaje(rawData)
        if decodedMessage: # Validamos la codificación
            print("Mensaje decodificado correctamente:", decodedMessage)

if __name__ == '__main__':
    main()
