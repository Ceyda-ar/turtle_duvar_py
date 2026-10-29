#!/usr/bin/env python3
import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from turtlesim.msg import Pose

ARENA_MERKEZI = 5.544445
KONTROL_FREKANSI_HZ = 20.0
LOG_PERIYODU_SN = 1.0
YON_TOLERANSI_RAD = 0.15
DONUS_KAZANCI = 3.0


def normalize_aci(aci: float) -> float:
    return math.atan2(math.sin(aci), math.cos(aci))


def _aralikli_aciklama(aciklama: str, alt: float, ust: float) -> ParameterDescriptor:
    return ParameterDescriptor(
        description=aciklama,
        floating_point_range=[FloatingPointRange(from_value=alt, to_value=ust, step=0.0)],
    )


class KacanKaplumbaga(Node):

    def __init__(self) -> None:
        super().__init__('kacan_kaplumbaga')

        self._declare_parameters()
        self._parametreleri_oku()
        self.add_on_set_parameters_callback(self._parametre_dogrula)

        self._pose: Optional[Pose] = None

        self._cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self._pose_callback, 10)
        self.create_timer(1.0 / KONTROL_FREKANSI_HZ, self._kontrol_callback)
        self.create_timer(LOG_PERIYODU_SN, self._log_callback)

        self.get_logger().info(
            f'Başlatıldı | hız: {self._lineer_hiz:.2f} m/s, '
            f'açısal: {self._aci_hiz:.2f} rad/s, '
            f'sınırlar: [{self._alt_sinir:.1f}, {self._ust_sinir:.1f}]'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            'lineer_hiz', 1.0, _aralikli_aciklama('İleri hız (m/s)', 0.0, 5.0))
        self.declare_parameter(
            'aci_hiz', 1.5, _aralikli_aciklama('Maksimum dönüş hızı (rad/s)', 0.1, 6.0))
        self.declare_parameter(
            'alt_sinir', 1.0, _aralikli_aciklama('Alt güvenli sınır', 0.0, 5.0))
        self.declare_parameter(
            'ust_sinir', 10.0, _aralikli_aciklama('Üst güvenli sınır', 6.0, 11.0))

    def _parametreleri_oku(self) -> None:
        self._lineer_hiz = self.get_parameter('lineer_hiz').value
        self._aci_hiz = self.get_parameter('aci_hiz').value
        self._alt_sinir = self.get_parameter('alt_sinir').value
        self._ust_sinir = self.get_parameter('ust_sinir').value

    def _parametre_dogrula(self, params: List[Parameter]) -> SetParametersResult:
        yeni = {
            'lineer_hiz': self._lineer_hiz,
            'aci_hiz': self._aci_hiz,
            'alt_sinir': self._alt_sinir,
            'ust_sinir': self._ust_sinir,
        }
        for p in params:
            if p.name in yeni:
                yeni[p.name] = float(p.value)

        if yeni['alt_sinir'] >= yeni['ust_sinir']:
            return SetParametersResult(
                successful=False, reason='alt_sinir, ust_sinir değerinden küçük olmalı.')

        self._lineer_hiz = yeni['lineer_hiz']
        self._aci_hiz = yeni['aci_hiz']
        self._alt_sinir = yeni['alt_sinir']
        self._ust_sinir = yeni['ust_sinir']
        return SetParametersResult(successful=True)

    def _pose_callback(self, msg: Pose) -> None:
        self._pose = msg

    def _sinir_disinda_mi(self, pose: Pose) -> bool:
        return not (self._alt_sinir <= pose.x <= self._ust_sinir
                    and self._alt_sinir <= pose.y <= self._ust_sinir)

    def _kontrol_callback(self) -> None:
        if self._pose is None:
            return

        twist = Twist()

        if self._sinir_disinda_mi(self._pose):
            hedef_aci = math.atan2(ARENA_MERKEZI - self._pose.y,
                                   ARENA_MERKEZI - self._pose.x)
            hata = normalize_aci(hedef_aci - self._pose.theta)

            if abs(hata) > YON_TOLERANSI_RAD:
                twist.angular.z = max(-self._aci_hiz,
                                      min(self._aci_hiz, DONUS_KAZANCI * hata))
            else:
                twist.linear.x = self._lineer_hiz
        else:
            twist.linear.x = self._lineer_hiz

        self._cmd_pub.publish(twist)

    def _log_callback(self) -> None:
        if self._pose is None:
            self.get_logger().warn('Henüz /turtle1/pose verisi alınmadı (turtlesim açık mı?).')
            return
        self.get_logger().info(
            f'Konum -> X: {self._pose.x:.2f}, Y: {self._pose.y:.2f}, '
            f'Yön: {math.degrees(self._pose.theta):.0f}°'
        )

    def durdur(self) -> None:
        self._cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KacanKaplumbaga()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.durdur()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

