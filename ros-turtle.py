# Importa as bibliotecas e módulos necessários para a execução do código.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from turtlesim.srv import SetPen, TeleportAbsolute, Kill, Spawn

# Define a classe TurtleController que é um tipo de nó de ros.
class TurtleController(Node):
    def __init__(self, turtleName):
        super().__init__(turtleName)
        self.turtleName = turtleName

    # Método para criar e configurar a tartaruga no ambiente de simulação.
    def spawn_turtle(self, x=1.0, y=1.0):

        # Cria um cliente para a chamada do serviço de remoção de tartaruga.
        self.killClient = self.create_client(Kill, '/kill')
        killRequest = Kill.Request()
        killRequest.name = self.turtleName

        # Chamada assíncrona para a remoção da tartaruga.
        self.killClient.call_async(killRequest)
        time.sleep(0.5)

        # Cria um cliente para a chamada do serviço de criação de tartaruga.
        self.spawnClient = self.create_client(Spawn, '/spawn')
        spawnRequest = Spawn.Request()
        spawnRequest.name = self.turtleName
        spawnRequest.x = x
        spawnRequest.y = y

        # Chamada assíncrona para a criação da tartaruga.
        self.spawnClient.call_async(spawnRequest)

        # Cria um publicador para controlar o movimento da tartaruga.
        self.publisher = self.create_publisher(Twist, f'/{self.turtleName}/cmd_vel', 10)
       
        # Cria um cliente para a chamada do serviço que define as configurações da caneta da tartaruga.
        self.penClient = self.create_client(SetPen, f'/{self.turtleName}/set_pen')
        time.sleep(0.4)

    # Método para mover a tartaruga no ambiente de simulação.
    def move_turtle(self, x=0.0, y=0.0, z=0.0, ax=0.0, ay=0.0, az=0.0, times=1):
        for i in range(times):
            moveMessage = Twist()
            moveMessage.linear.x = x
            moveMessage.linear.y = y
            moveMessage.linear.z = z
            moveMessage.angular.x = ax
            moveMessage.angular.y = ay
            moveMessage.angular.z = az

            # Publica a mensagem de movimento da tartaruga.
            self.publisher.publish(moveMessage)
            time.sleep(1)

    # Método para definir a cor da caneta da tartaruga.
    def pen_color(self, rgb, width=5):
        pen = SetPen.Request()
        pen.off = False
        pen.r = rgb[0]
        pen.g = rgb[1]
        pen.b = rgb[2]     
        pen.width = width

        # Chamada assíncrona para definir as configurações da caneta da tartaruga.
        self.penClient.call_async(pen)

    # Método para remover a tartaruga do ambiente de simulação.
    def delete_turtle(self):
        killRequest = Kill.Request()
        killRequest.name = self.turtleName

        # Chamada assíncrona para remover a tartaruga.
        self.killClient.call_async(killRequest)

# Função principal do código.
def main ():
    
    # Inicia o sistema ROS
    rclpy.init()

    # Cria as tartarugas e faz um desenho
    turtle = TurtleController("Verde")
    turtle.spawn_turtle()
    turtle.pen_color(rgb=[0, 128, 0], width=255)
    turtle.move_turtle(x=10.0)
    turtle.delete_turtle()

    turtle2 = TurtleController("Arvore1")
    turtle2.spawn_turtle(x=2.0, y=4.0)
    turtle2.pen_color(rgb=[124, 71, 0], width=50)
    turtle2.move_turtle(y=3.0)
    turtle2.delete_turtle()

    turtle3 = TurtleController("Arvore2")
    turtle3.spawn_turtle(x=6.0, y=2.0)
    turtle3.pen_color(rgb=[124, 71, 0], width=50)
    turtle3.move_turtle(y=3.0)
    turtle3.delete_turtle()

    turtle4 = TurtleController("Arvore3")
    turtle4.spawn_turtle(x=9.0, y=3.0)
    turtle4.pen_color(rgb=[124, 71, 0], width=50)
    turtle4.move_turtle(y=3.0)
    turtle4.delete_turtle()

    turtle5 = TurtleController("Folhas1")
    turtle5.spawn_turtle(x=0.0, y=7.0)
    turtle5.pen_color(rgb=[0, 100, 0], width=50)
    turtle5.move_turtle(x=3.0)
    turtle5.move_turtle(y=1.0)
    turtle5.move_turtle(x=-3.0)
    turtle5.move_turtle(y=1.0)
    turtle5.move_turtle(x=3.0)
    turtle5.delete_turtle()

    turtle6 = TurtleController("Folhas2")
    turtle6.spawn_turtle(x=4.0, y=5.0)
    turtle6.pen_color(rgb=[0, 100, 0], width=50)
    turtle6.move_turtle(x=3.0)
    turtle6.move_turtle(y=1.0)
    turtle6.move_turtle(x=-2.0)
    turtle6.move_turtle(y=1.0)
    turtle6.move_turtle(x=2.0)
    turtle6.delete_turtle()

    turtle7 = TurtleController("Folhas3")
    turtle7.spawn_turtle(x=8.0, y=6.0)
    turtle7.pen_color(rgb=[0, 100, 0], width=50)
    turtle7.move_turtle(x=2.0)
    turtle7.move_turtle(y=1.0)
    turtle7.move_turtle(x=-3.0)
    turtle7.delete_turtle()

    turtle8 = TurtleController("Sun")
    turtle8.spawn_turtle(x=9.0, y=9.0)
    turtle8.pen_color(width=80, rgb=[255,180, 40])
    turtle8.move_turtle(x=0.5, az=1.5, times=4)
    turtle8.delete_turtle()

    # Termina o sistema ROS
    rclpy.shutdown()

# Chama a função main
if __name__ == '__main__':
    main()