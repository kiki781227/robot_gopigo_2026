import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math 

class RetourLogique(Node):
    def __init__(self):
        super().__init__('retour_logique')

        self.declare_parameter('distance_cible', 1.0)

        # On récupère la valeur initiale du paramètre
        self.distance_actuelle = self.get_parameter('distance_cible').get_parameter_value().double_value
        
        # 1. Abonnement à l'odométrie (pour RECUPERER les valeurs)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)
        
        # 2. Publication des commandes (pour Pilotage)
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        # 3. Variables de stockage
        self.historique_x = [] # Le tableau de stockage des valeurs d'Odométrie
        self.phase = "ALLER"   # Etat actuel du robot pour spécifier quelle action doit-il faire

        # --- ENREGISTREMENT DU CALLBACK ---
        # Cette ligne lie la modification du paramètre à la fonction on_parameter_change
        self.add_on_set_parameters_callback(self.on_parameter_change)

        self.get_logger().info(f"Nœud prêt. Distance cible initiale : {self.distance_actuelle}m")

    def on_parameter_change(self, params):
        """Fonction appelée automatiquement lors d'un 'ros2 param set'"""
        for param in params:
            if param.name == 'distance_cible' and param.type_ == param.Type.DOUBLE:
                nouvelle_valeur = param.value
                
                # CONDITION CRITIQUE : On ne réinitialise que si la valeur a VRAIMENT changé
                if nouvelle_valeur != self.distance_actuelle:
                    self.get_logger().info(f"CHANGEMENT DÉTECTÉ : {self.distance_actuelle}m -> {nouvelle_valeur}m")
                    
                    # Mise à jour de la mémoire du nœud
                    self.distance_actuelle = nouvelle_valeur
                    
                    # RÉINITIALISATION DE LA MISSION
                    self.phase = "ALLER"
                    self.historique_x = [] 
                    self.get_logger().info("Le robot est réinitialisé et repart en phase ALLER.")
                else:
                    self.get_logger().info("Le paramètre reçu est identique à l'actuel. Aucune action requise.")
                    
        return rclpy.node.SetParametersResult(successful=True)
    
    def listener_callback(self, msg):
        distance_max = self.get_parameter('distance_cible').get_parameter_value().double_value

        # Extraction de la valeur X du message Odometry
        x_actuel = msg.pose.pose.position.x
        
        cmd = Twist()
        
        if self.phase == "ALLER":
            # Tant que le robot n'aurais pas atteint la ligne d'arrivé (la ligne cible) il continuera d'avancer et de stocker les valeurs X du message Odometry dans le tableau
            if abs(x_actuel) < distance_max:
                # Etape : Stockage des valeurs
                self.historique_x.append(x_actuel)
                cmd.linear.x = -1.0 # On avance
            
            else:
                self.get_logger().info("Cible atteinte. Début du retour !")
                self.phase = "RETOUR"
                
        elif self.phase == "RETOUR":
            if len(self.historique_x) > 0:
                # Etape : on récupère la dernière valeur stockée (LIFO)
                derniere_valeur = self.historique_x.pop() # On enlève et renvoie le dernier élément
            
                # Logique simplifiée : si x_actuel est supérieur au point visé, on recule
                if x_actuel < derniere_valeur:
                    cmd.linear.x = 1.0
                
                else:
                    # On est déjà arrivé à ce point, la prochaine itération prendra le suivant
                    self.get_logger().info("PROCESSUS DE DEPLACEMENT DU ROBOT TERMINER !")
                    cmd.linear.x = 0.0
                    self.phase = "STOP"
        
        self.publisher.publish(cmd)
    
def main(args=None):
    rclpy.init(args=args)
    node = RetourLogique()

    try:
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
