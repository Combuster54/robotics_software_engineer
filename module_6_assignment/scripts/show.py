import rclpy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import pandas as pd
import matplotlib.pyplot as plt
import Odometry_pb2  # Archivo generado por Protobuf

def main():

    # Inicializa ROS 2
    rclpy.init()

    # Ruta al rosbag
    bag_file = "/home/combuster/robotisim_ws/src/robotics_software_engineer/module_6_assignment/bag_files/first_set/"

    # Configuración del lector de rosbag
    storage_options = StorageOptions(
        uri=bag_file,
        storage_id='sqlite3'
    )
    
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Configuración para leer datos
    odom_data = {'time': [], 'x': [], 'y': []}
    imu_data = {'time': [], 'angular_velocity_z': []}
    start_time = None

    # Leer mensajes del rosbag
    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if start_time is None:
            start_time = timestamp

        # Procesar /odometry/filtered
        if topic == '/odometry/filtered':
            # Decodificar datos binarios usando Protobuf
            odometry_msg = Odometry_pb2.Odometry()
            odometry_msg.ParseFromString(data)
            odom_data['time'].append((timestamp - start_time) / 1e9)
            odom_data['x'].append(odometry_msg.pose.pose.position.x)
            odom_data['y'].append(odometry_msg.pose.pose.position.y)

    # Crear DataFrames
    odom_df = pd.DataFrame(odom_data)
    imu_df = pd.DataFrame(imu_data)

if __name__ == '__main__':
    main()
