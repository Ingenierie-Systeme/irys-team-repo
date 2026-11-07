import math
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float32MultiArray

from robot_control.servo_command import (
    FIN_MAX_ANGLE_DEGREES,
    MASS_MAX_ANGLE_DEGREES,
    convert_direction_to_servo_positions,
)

PUBLISH_RATE_HZ = 50.0


class ServoCommandPublisher(Node):
    """Publie les consignes servo autonomes ou celles de la télécommande."""

    def __init__(self) -> None:
        super().__init__('servo_command_publisher')
        self.declare_parameter('direction_topic', '/commande/direction')
        self.declare_parameter('direction_timeout_seconds', 1.0)
        self.declare_parameter('rc_topic', '/input/rc')
        self.declare_parameter('servos_topic', '/output/servos')
        self.declare_parameter('rc_timeout_seconds', 2.0)
        self.declare_parameter('throttle_channel_index', 2) # RC1 : u
        self.declare_parameter('aileron_channel_index', 0)  # RC3 : v
        self.declare_parameter('takeover_channel_index', 4) # RC5 : override
        self.declare_parameter('takeover_threshold', 0.5)
        self.declare_parameter('takeover_when_high', True)
        self.declare_parameter(
            'emergency_stop_topic',
            '/safety/emergency_stop',
        )
        self.declare_parameter('emergency_stop_timeout_seconds', 0.5)

        direction_topic = self.get_parameter('direction_topic').value
        rc_topic = self.get_parameter('rc_topic').value
        emergency_stop_topic = self.get_parameter(
            'emergency_stop_topic'
        ).value
        self.direction_timeout_seconds = float(
            self.get_parameter('direction_timeout_seconds').value
        )
        self.rc_timeout_seconds = float(
            self.get_parameter('rc_timeout_seconds').value
        )
        self.emergency_stop_timeout_seconds = float(
            self.get_parameter('emergency_stop_timeout_seconds').value
        )
        self.throttle_channel_index = int(
            self.get_parameter('throttle_channel_index').value
        )
        self.aileron_channel_index = int(
            self.get_parameter('aileron_channel_index').value
        )
        self.takeover_channel_index = int(
            self.get_parameter('takeover_channel_index').value
        )
        self.takeover_threshold = float(
            self.get_parameter('takeover_threshold').value
        )
        self.takeover_when_high = bool(
            self.get_parameter('takeover_when_high').value
        )

        if (
            not math.isfinite(self.direction_timeout_seconds)
            or self.direction_timeout_seconds <= 0.0
        ):
            raise ValueError(
                'direction_timeout_seconds doit être supérieur à zéro.'
            )
        if (
            not math.isfinite(self.rc_timeout_seconds)
            or self.rc_timeout_seconds <= 0.0
        ):
            raise ValueError(
                'rc_timeout_seconds doit être supérieur à zéro.'
            )
        if (
            not math.isfinite(self.emergency_stop_timeout_seconds)
            or self.emergency_stop_timeout_seconds <= 0.0
        ):
            raise ValueError(
                'emergency_stop_timeout_seconds doit être supérieur '
                'à zéro.'
            )
        if min(
            self.throttle_channel_index,
            self.aileron_channel_index,
            self.takeover_channel_index,
        ) < 0:
            raise ValueError('Les index de canaux RC doivent être positifs.')
        if (
            not math.isfinite(self.takeover_threshold)
            or not -1.0 <= self.takeover_threshold <= 1.0
        ):
            raise ValueError(
                'takeover_threshold doit être compris entre -1 et 1.'
            )

        direction_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.servo_publisher = self.create_publisher(
            Float32MultiArray, self.get_parameter('servos_topic').value, 10
        )
        self.direction_subscription = self.create_subscription(
            Vector3Stamped,
            direction_topic,
            self.on_direction,
            direction_qos,
        )
        self.rc_subscription = self.create_subscription(
            Float32MultiArray,
            rc_topic,
            self.on_rc,
            10,
        )
        self.emergency_stop_subscription = self.create_subscription(
            Bool,
            emergency_stop_topic,
            self.on_emergency_stop,
            direction_qos,
        )
        self.direction = (0.0, 0.0)
        self.last_direction_time: float | None = None
        self.rc_channels = (-1.0, 0.0, 0.0, 0.0, -1.0, -1.0, -1.0, -1.0)

        # État de la commande hydro.
        self.u_filtered = 0.0
        self.mean_filtered = 0.0
        self.fin_phase = 0.0
        self.fin_position = 0.0
        self.fin_speed = 0.0

        # Position estimée de la masse, sans capteur de position.
        self.mass_position = 0.0

        # Réglages provisoires à adapter au mécanisme.
        self.mass_speed_limit = 20.0  # degrés/seconde
        self.mass_angle_limit = 50.0  # degrés
        self.mass_deadband = 0.05

        self.last_update_time = time.monotonic()

        self.last_rc_time: float | None = None
        self.takeover_active = False
        self.takeover_start_time: float | None = None
        self.emergency_stop: bool | None = None
        self.last_emergency_stop_time: float | None = None
        self.last_safety_block_reason: str | None = None
        self.warned_invalid_rc = False
        self.start_time = time.monotonic()
        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE_HZ,
            self.publish_commands,
        )
        self.get_logger().info(
            f'Conversion de {direction_topic} vers /output/servos ; '
            f'canaux RC sur {rc_topic}.'
        )

    def on_direction(self, message: Vector3Stamped) -> None:
        direction = (message.vector.x, message.vector.y)
        if not all(math.isfinite(value) for value in direction):
            self.direction = (0.0, 0.0)
            self.last_direction_time = time.monotonic()
            self.get_logger().error(
                'Direction invalide reçue ; consigne servo mise à zéro.'
            )
            return

        self.direction = direction
        self.last_direction_time = time.monotonic()

    def on_rc(self, message: Float32MultiArray) -> None:
        required_channel = max(
            self.throttle_channel_index,
            self.aileron_channel_index,
            self.takeover_channel_index,
            7,  # RC8
        )
        if len(message.data) <= required_channel:
            self.invalidate_rc(
                f'{required_channel + 1} canaux requis, '
                f'{len(message.data)} reçus'
            )
            return

        throttle = float(message.data[self.throttle_channel_index])
        aileron = float(message.data[self.aileron_channel_index])
        switch_value = float(message.data[self.takeover_channel_index])
        if not all(
            math.isfinite(value)
            for value in message.data[:8]
        ):
            self.invalidate_rc('valeur de canal non finie')
            return

        switch_is_high = switch_value > self.takeover_threshold
        takeover_active = (
            switch_is_high
            if self.takeover_when_high
            else not switch_is_high
        )
        now = time.monotonic()
        if takeover_active and not self.takeover_active:
            self.takeover_start_time = now
            self.get_logger().warning(
                'La télécommande a pris le contrôle des servos.'
            )
        elif not takeover_active and self.takeover_active:
            self.get_logger().info(
                'La télécommande a rendu le contrôle aux autres nœuds.'
            )

        self.rc_channels = tuple(float(value) for value in message.data[:8])

        self.takeover_active = takeover_active
        self.last_rc_time = now
        self.warned_invalid_rc = False

    def invalidate_rc(self, reason: str) -> None:
        self.last_rc_time = None
        self.takeover_active = False
        self.takeover_start_time = None
        if not self.warned_invalid_rc:
            self.get_logger().warning(
                f'Entrée RC invalide ({reason}) ; '
                'commande manuelle désactivée.'
            )
            self.warned_invalid_rc = True

    def on_emergency_stop(self, message: Bool) -> None:
        self.emergency_stop = message.data
        self.last_emergency_stop_time = time.monotonic()

    def publish_commands(self) -> None:
        now = time.monotonic()
        dt = min(max(now - self.last_update_time, 0.001), 0.1)
        self.last_update_time = now
        elapsed = now - self.start_time
        rc_is_fresh = (
            self.last_rc_time is not None
            and now - self.last_rc_time <= self.rc_timeout_seconds
        )
        if not rc_is_fresh and self.takeover_active:
            self.takeover_active = False
            self.takeover_start_time = None
            self.get_logger().warning(
                'Signal de télécommande perdu ; retour au contrôle '
                'autonome.'
            )

        emergency_stop_is_fresh = (
            self.last_emergency_stop_time is not None
            and (
                now - self.last_emergency_stop_time
                <= self.emergency_stop_timeout_seconds
            )
        )
        if not emergency_stop_is_fresh:
            safety_block_reason = (
                'état d’arrêt d’urgence absent ou périmé'
            )
        elif self.emergency_stop:
            safety_block_reason = 'arrêt d’urgence actif'
        else:
            safety_block_reason = None

        if safety_block_reason is not None and not (rc_is_fresh and self.takeover_active):
            fin_angle, mass_angle = 0.0, 0.0
            if safety_block_reason != self.last_safety_block_reason:
                self.get_logger().warning(
                    f'Commandes servo neutralisées : {safety_block_reason}.'
                )
        elif rc_is_fresh and self.takeover_active:
            # Calcul de la commande avec loie définie.
            fin_angle, mass_angle = self.calculate_rc_commands(dt)
        else:
            direction = self.direction
            if (
                self.last_direction_time is None
                or now - self.last_direction_time
                > self.direction_timeout_seconds
            ):
                direction = (0.0, 0.0)
            fin_angle, mass_angle = convert_direction_to_servo_positions(
                *direction,
                elapsed,
            )
        if not (rc_is_fresh and self.takeover_active):
            self.fin_position = fin_angle
            self.fin_speed = 0.0
            self.mass_position = mass_angle
        self.last_safety_block_reason = safety_block_reason

        servo_message = Float32MultiArray()
        servo_message.data = [
            fin_angle / FIN_MAX_ANGLE_DEGREES,
            mass_angle / MASS_MAX_ANGLE_DEGREES,
        ]
        self.servo_publisher.publish(servo_message)

    def calculate_rc_commands(self, dt):
        """Calcule les consignes de nageoire et de masse."""

        def clamp(value, minimum, maximum):
            return max(minimum, min(maximum, value))

        rc1, rc2, rc3, rc4, _, rc6, rc7, rc8 = self.rc_channels

        # Le nœud SBUS publie de -1 à 1.
        # RC3 minimum -> u=0 ; RC3 maximum -> u=1.
        u = clamp((rc3 + 1.0) / 2.0, 0.0, 1.0)
        v = clamp(rc1, -1.0, 1.0)

        # Constantes de SimulationHydro.
        max_amplitude = 90.0 * clamp((rc8 + 1.0) / 2.0, 0.0, 1.0)  # RC8 : 0 à 90°
        max_frequency = 5.0 * clamp((rc7 + 1.0) / 2.0, 0.0, 1.0)  # RC7 : 0 à 5 Hz
        max_mean = 60.0            # degrés
        fin_limit = 80.0           # degrés
        max_speed = 720.0          # degrés/seconde
        max_acceleration = 4320.0  # degrés/seconde²

        # Filtres de HydroCommand.cs.
        self.u_filtered += (
            (u - self.u_filtered)
            * (1.0 - math.exp(-dt / 0.8))
        )

        self.mean_filtered += (
            (v * max_mean - self.mean_filtered)
            * (1.0 - math.exp(-dt / 0.65))
        )

        # Relation entre u, fréquence et amplitude.
        theta = self.u_filtered * math.pi / 2.0

        frequency = max_frequency * (1.0 - math.cos(theta))
        amplitude = max_amplitude * math.sin(theta)

        mean_speed_budget = 2.0 * max_mean / 0.65

        if frequency > 0.0:
            amplitude = min(
                amplitude,
                (max_speed - mean_speed_budget)
                / (2.0 * math.pi * frequency),
            )

        amplitude = max(
            0.0,
            min(amplitude, fin_limit - abs(self.mean_filtered)),
        )

        if rc6 > 0.5:
            # RC6 haut : RC4 donne la position de nageoire.
            desired_fin = clamp(rc4, -1.0, 1.0) * fin_limit
        else:
            # RC6 bas : oscillation autour de la moyenne donnée par v.
            self.fin_phase = (
                self.fin_phase + 2.0 * math.pi * frequency * dt
            ) % (2.0 * math.pi)

            desired_fin = (
                self.mean_filtered
                + amplitude * math.sin(self.fin_phase)
            )

        # Limites de vitesse et d'accélération de la consigne.
        desired_speed = clamp(
            30.0 * (desired_fin - self.fin_position),
            -max_speed,
            max_speed,
        )

        speed_change = clamp(
            desired_speed - self.fin_speed,
            -max_acceleration * dt,
            max_acceleration * dt,
        )

        self.fin_speed += speed_change

        self.fin_position = clamp(
            self.fin_position + self.fin_speed * dt,
            -fin_limit,
            fin_limit,
        )

        # RC2 commande la vitesse de déplacement de la masse.
        mass_input = clamp(rc2, -1.0, 1.0)

        if abs(mass_input) <= self.mass_deadband:
            mass_input = 0.0

        self.mass_position = clamp(
            self.mass_position
            + mass_input * self.mass_speed_limit * dt,
            -self.mass_angle_limit,
            self.mass_angle_limit,
        )

        return self.fin_position, self.mass_position


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ServoCommandPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
