import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import gopigo3


class CmdVelNode(Node):
    
    def __init__(self):
        # Initialisation de la classe mère dans le cas où elle possède un constructeur 
        super().__init__('cmd_vel_node')

        # Paramètres configurables
        # self.declare_parameter c'est une fonction pour déclarer une variable globale manipulable (lecture, modification, réinitialisation) 
        # au cours de l'exécution du fichier (.py) du noeud par ROS2 depuis le terminal. Sans y toucher au code dans le fichier lui-même
        # 1er argument : 'max_speed' -> nom de la variable globale (le paramètre configurable de ROS2)
        # 2eme argument : 300 -> variable par défaut du variable globale (le paramètre configurable de ROS2)
        self.declare_parameter('max_speed', 300)   # vitesse max GoPiGo3 (0-1000)
        self.declare_parameter('wheel_base', 0.117) # distance entre roues en mètres

        self.max_speed  = self.get_parameter('max_speed').value
        self.wheel_base = self.get_parameter('wheel_base').value

        # Initialise le GoPiGo3
        try:
            # Initialisation de la classe GoPiGo3 fournir par la bibliothèque gopigo3
            self.gpg = gopigo3.GoPiGo3()
            
            # Par le principe de l'héritage nous allons directement accéder aux fonctions de la classe mère, au niveau de la classe fille via le mot-clé "self". Sans avoir besoin d'instancier la classe mère dans la classe fille. Le mot-clé "self" permet d'y accéder aux éléments de la classe courante (attributs et méthode)
            # self.get_logger().info() -> Pour écrire un message de log de niveau "Info" dans le système de ROS2
            # Le message en question apparaîssent dans le terminal où l'on a lancé le noeud
            self.get_logger().info('GoPiGo3 initialise !')
            self.get_logger().info(
                f'Batterie : {self.gpg.get_voltage_battery():.2f}V'
            )
        except Exception as e:
            self.get_logger().error(f'Erreur GoPiGo3 : {e}')
            return

        # Subscriber — écoute les commandes de mouvement
        self.subscription = self.create_subscription(
            Twist,      # Type de message attendu (geometry_msgs/Twist)
            '/cmd_vel', # Nom du topic auquel on s'abonne
            self.cmd_vel_callback,  # Fonction callback appelé à chaque message reçu
            10          # Taille de la file d'attente
        )

        # Sécurité — arrête le robot si plus de commande reçue depuis 0.5s
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info('cmd_vel_node pret - en attente de commandes...')

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        linear  = msg.linear.x   # m/s
        angular = msg.angular.z  # rad/s

        # Convertit en vitesse pour chaque moteur
        # Différentielle : gauche = linear - angular, droite = linear + angular
        left_speed  = linear - (angular * self.wheel_base / 2.0)
        right_speed = linear + (angular * self.wheel_base / 2.0)

        # Normalise entre -max_speed et +max_speed
        left_dps  = int(left_speed  * self.max_speed)
        right_dps = int(right_speed * self.max_speed)

        # Clamp — évite de dépasser les limites
        left_dps  = max(-self.max_speed, min(self.max_speed, left_dps))
        right_dps = max(-self.max_speed, min(self.max_speed, right_dps))

        # Envoie aux moteurs
        self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT,  left_dps)
        self.gpg.set_motor_dps(self.gpg.MOTOR_RIGHT, right_dps)

        self.get_logger().debug(
            f'cmd_vel -> L:{left_dps} R:{right_dps} dps'
        )

    def safety_check(self):
        # Si aucune commande depuis 0.5s → arrêt d'urgence
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 0.5:
            self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT,  0)
            self.gpg.set_motor_dps(self.gpg.MOTOR_RIGHT, 0)

    def destroy_node(self):
        # Arrêt propre à la fermeture
        try:
            self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT,  0)
            self.gpg.set_motor_dps(self.gpg.MOTOR_RIGHT, 0)
            self.gpg.reset_all()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
