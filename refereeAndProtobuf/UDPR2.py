#!/usr/bin/env python3
# coding=utf-8

import rclpy
from rclpy.node import Node

import threading

from blenders_msgs.msg import Referee
import socket
import struct


class GameStateDecoder:

    def __init__(self):

        self._competitionType = {
            0: "SMALL",
            1: "MIDDLE",
            2: "LARGE"
        }

        self._gamePhase = {
            0: "NORMAL",
            1: "PENALTY_SHOOT_OUT",
            2: "EXTRA_TIME",
            3: "TIMEOUT"
        }

        self._state = {
            0: "INITIAL",
            1: "READY",
            2: "SET",
            3: "PLAYING",
            4: "FINISHED"
        }

        self._setPlay = {
            0: "NONE",
            1: "DIRECT_FREE_KICK",
            2: "INDIRECT_FREE_KICK",
            3: "PENALTY_KICK",
            4: "THROW_IN",
            5: "GOAL_KICK",
            6: "CORNER_KICK"
        }

        self._penalty = {
            0: "NONE",
            1: "ILLEGAL_POSITIONING",
            2: "MOTION_IN_SET",
            3: "MOTION_IN_STOP",
            4: "LOCAL_GAME_STUCK",
            5: "INCAPABLE_ROBOT",
            6: "PICK_UP",
            7: "BALL_HOLDING",
            8: "LEAVING_THE_FIELD",
            9: "PLAYING_WITH_ARMS_HANDS",
            10: "PUSHING",
            11: "CAUTIONED",
            12: "SENT_OFF",
            13: "SUBSTITUTE"
        }


    def decode(self, rawData, robotID):

        refereeMsg = Referee()

        # =============================
        # Header
        # =============================

        refereeMsg.header = rawData[0:4].decode()

        refereeMsg.protocol_version = rawData[4]
        refereeMsg.packet_number = rawData[5]
        refereeMsg.players_per_team = rawData[6]


        # =============================
        # Game information
        # =============================

        competitionType = rawData[7]
        stopped = rawData[8]
        gamePhase = rawData[9]
        state = rawData[10]
        setPlay = rawData[11]

        refereeMsg.first_half = bool(rawData[12])
        refereeMsg.kicking_team = rawData[13]


        refereeMsg.secs_remaining = int.from_bytes(
            rawData[14:16],
            byteorder="little",
            signed=True
        )

        refereeMsg.secondary_time = int.from_bytes(
            rawData[16:18],
            byteorder="little",
            signed=True
        )


        refereeMsg.competition_type = self._competitionType.get(
            competitionType,
            "UNKNOWN"
        )

        refereeMsg.game_phase = self._gamePhase.get(
            gamePhase,
            "UNKNOWN"
        )

        refereeMsg.state = self._state.get(
            state,
            "UNKNOWN"
        )

        refereeMsg.set_play = self._setPlay.get(
            setPlay,
            "UNKNOWN"
        )


        # =============================
        # Select team
        # =============================

        # robotID comienza en 1
        # equipo 0 o equipo 1 según configuración

        if robotID == 1:
            teamOffset = 18
        else:
            teamOffset = 88


        # =============================
        # Team information
        # =============================

        refereeMsg.team_number = rawData[teamOffset]

        refereeMsg.field_player_color = rawData[teamOffset + 1]

        refereeMsg.goalkeeper_color = rawData[teamOffset + 2]

        refereeMsg.goalkeeper = rawData[teamOffset + 3]

        refereeMsg.score = rawData[teamOffset + 4]

        refereeMsg.penalty_shot = rawData[teamOffset + 5]


        refereeMsg.single_shots = int.from_bytes(
            rawData[teamOffset + 6:teamOffset + 8],
            "little"
        )

        refereeMsg.message_budget = int.from_bytes(
            rawData[teamOffset + 8:teamOffset + 10],
            "little"
        )


        # =============================
        # Player information
        # =============================

        playerNumber = robotID - 1

        playerOffset = (
            teamOffset +
            10 +
            playerNumber * 3
        )


        penalty = rawData[playerOffset]

        refereeMsg.penalty = self._penalty.get(
            penalty,
            "UNKNOWN"
        )

        refereeMsg.secs_till_unpenalised = rawData[playerOffset + 1]

        refereeMsg.cautions = rawData[playerOffset + 2]


        # =============================
        # Robot state
        # =============================

        if refereeMsg.penalty != "NONE":
            refereeMsg.robot_play_state = "quieto"
            refereeMsg.robot_play_state_int = 0

        elif refereeMsg.state == "READY":
            refereeMsg.robot_play_state = "acomodate"
            refereeMsg.robot_play_state_int = 1

        elif refereeMsg.state == "PLAYING":
            refereeMsg.robot_play_state = "playing"
            refereeMsg.robot_play_state_int = 2

        else:
            refereeMsg.robot_play_state = "quieto"
            refereeMsg.robot_play_state_int = 0


        return refereeMsg
    

class RefereePublisher(Node):

    #Información de mesaje Alive
    HEADER = b'RGrt'    # Header RGrt
    VERSION = 20        # Versión de la estructura de datos
    TEAM = 16           # Número de equipo
    STDMSG = 2

    def __init__(self,nodeName, publisherIp, refereeIp, listeningPort, sendingPort):
        #Definition of publisher
        
              
        super().__init__(nodeName) #we use init method of node class
        
        #robot id starts from 0, but GameController starts from 1
        #self.robotID = self.get_parameter('robot_id').value()
        self.declare_parameter('robot_id', 0)
        self.robotID =1
        self._pub = self.create_publisher(Referee,f'robotis_{self.robotID}/refereeData', 1) #we create the publisher

        self._ip = publisherIp
        self._refereeIp = refereeIp
        self._listeningPort = listeningPort
        self._sendingPort = sendingPort
        #we create the socket for listen to the referee
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) #datagram socket IPv4
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1) #allows to use the server after being restarted

        self.sock.bind((self._ip , self._listeningPort))
        
        #we create the socket to talk to the referee

        self.returnSock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.aliveMsg = struct.pack(
            "<4s4B",
            self.HEADER,
            self.VERSION,
            self.TEAM,
            self.robotID,
            self.STDMSG
        )

        #we instance a decoder to read the messages sent by the referee
        self.decoder = GameStateDecoder()

        self.running = True
        #we create a thread to review the messages listened by our node
        self.udp_thread = threading.Thread(
            target=self.udp_loop,
            daemon=True
        )

        self.udp_thread.start()

        

    def udp_loop(self):

        while self.running:

            try:

                rawData, addr = self.sock.recvfrom(1024)
                #print(len(rawData))
                #print(rawData)

                msg = self.decoder.decode(
                    rawData,
                    self.robotID
                )

                self.returnSock.sendto(
                    self.aliveMsg,
                    (self._refereeIp, self._sendingPort)
                )

                self._pub.publish(msg)
                

            except Exception as e:

                self.get_logger().error(str(e))


    def destroy_node(self):
        self.running = False
        self.get_logger().info(
            "Cerrando sockets..."
        )

        if self.udp_thread.is_alive():
            self.udp_thread.join(timeout=1.0)

        try:
            self.sock.close()
        except:
            pass

        try:
            self.returnSock.close()
        except:
            pass

        super().destroy_node()

    

def main(args=None):
    #initialize the node
    rclpy.init(args=args)
    refPublisher=RefereePublisher("referee_node","0.0.0.0","10.43.103.218",3838,3939)
    rclpy.spin(refPublisher)

    #shoutdown the node
    node.destroy_node()
    rclpy.shutdown()
   


if __name__ == '__main__': #entrypoint
    main()