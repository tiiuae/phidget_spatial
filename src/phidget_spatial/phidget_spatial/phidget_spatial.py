import Phidget22.PhidgetException
import rclpy
import rclpy.qos
from rclpy.node import Node

from Phidget22.Phidget import *
from Phidget22.Devices.Spatial import *

from std_msgs.msg import String

from os import environ as env

SERIALS = [373314]# , 373225, 373257, 373306]
G = 9.81

SENSOR_MESSAGES_PRESENT = False
try:
    from sensor_msgs.msg import Imu, MagneticField
    SENSOR_MESSAGES_PRESENT = True
except ImportError:
    print("SensorMsg not found, publishing Strings")


class PhidgetSpatialNode(Node):
    def __init__(self):
        super().__init__("phidget_spatial_node")#, namespace=(env.get("DRONE_DEVICE_ID", env.get("USER"))))
        self.declare_parameters(namespace="",
                                parameters=[("publish_interval_ms", 50),
                                            ("motor_serials", [])
                                            ]
                                )

        self._publish_interval_ms = self.get_parameter("publish_interval_ms").value

        self.get_logger().info(str(self.get_parameter("publish_interval_ms").value))
        self.get_logger().info(str(self.get_parameter("motor_serials").value))

        self._imu_publisher = self.create_publisher(Imu if SENSOR_MESSAGES_PRESENT else String,
                                                    "PhidgetSpatialIMU_PubSubTopic",
                                                    rclpy.qos.qos_profile_sensor_data)
        self._magnetometer_publisher = self.create_publisher(MagneticField if SENSOR_MESSAGES_PRESENT else String,
                                                             "PhidgetSpatialMag_PubSubTopic",
                                                             rclpy.qos.qos_profile_sensor_data)

        self._devices = []
        self._device_index = {}
        for i, number in enumerate(self.get_parameter("motor_serials").value):
            try:
                device = Spatial()
                device.setDeviceSerialNumber(number)
                device.openWaitForAttachment(1000)
                device.setDataInterval(self._publish_interval_ms)
                device.setOnSpatialDataHandler(self.spatial_data_callback)
                self._devices.append(device)
                self._device_index[number] = i
            except Phidget22.PhidgetException.PhidgetException as e:
                self.get_logger().fatal("Device with serial number {} was not found.".format(number))
                raise ModuleNotFoundError

    def spatial_data_callback(self, child, acceleration, angular_rate, magnetic_field, timestamp):
        if SENSOR_MESSAGES_PRESENT:
            imu_msg = Imu()
            mag_msg = MagneticField()

            imu_msg.header.frame_id = mag_msg.header.frame_id = \
                "/phidget_{}".format(self._device_index[child.getDeviceSerialNumber()] + 1)
            imu_msg.header.stamp = mag_msg.header.stamp = self.get_clock().now().to_msg()

            imu_msg.linear_acceleration.x = float(acceleration[0] * G)
            imu_msg.linear_acceleration.y = float(acceleration[1] * G)
            imu_msg.linear_acceleration.z = float(acceleration[2] * G)
            imu_msg.angular_velocity.x = float(angular_rate[0])
            imu_msg.angular_velocity.y = float(angular_rate[1])
            imu_msg.angular_velocity.z = float(angular_rate[2])

            mag_msg.magnetic_field.x = float(magnetic_field[0] / 10000)
            mag_msg.magnetic_field.y = float(magnetic_field[1] / 10000)
            mag_msg.magnetic_field.z = float(magnetic_field[2] / 10000)
        else:
            imu_msg = String()
            mag_msg = String()
            now = "{:.4f}".format(self.get_clock().now().nanoseconds / 1000000000)
            imu_msg.data = "Dev: {}, Time: {}, Acc: {}, Ang: {}".format(child.getDeviceSerialNumber(), now,
                                                                        str(list(map(lambda v: v * G, acceleration))),
                                                                        str(angular_rate))
            mag_msg.data = "Dev: {}, Time: {}, Mag: {}".format(child.getDeviceSerialNumber(), now,
                                                               str(magnetic_field))
            print("{},{},{}".format(*magnetic_field))

        self._imu_publisher.publish(imu_msg)
        self._magnetometer_publisher.publish(mag_msg)
        # Check if publish interval has changed
        if self._publish_interval_ms != (new_value := self.get_parameter("publish_interval_ms").value):
            self._publish_interval_ms = new_value
            child.setDataInterval(new_value)


def main(args=None):
    rclpy.init(args=args)

    phidget_spatial = PhidgetSpatialNode()

    try:
        rclpy.spin(phidget_spatial)
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    phidget_spatial.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
