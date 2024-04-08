import dataclasses
import threading
import time
import math

COLLISION_DISTANCE = 1.0

from piosdk.piosdk import Pioneer
from edubot_sdk.edubot_sdk import EdubotGCS

# Классы для хранения настроек подключения
@dataclasses.dataclass
class IpPort:
    ip: str
    port: int

class DroneConnectingData:
    drone0: IpPort = IpPort(ip="127.0.0.1", port=8000)
    drone1: IpPort = IpPort(ip="127.0.0.1", port=8001)

class RobotConnectingData:
    robot0: IpPort = IpPort(ip="127.0.0.1", port=8004)
    robot1: IpPort = IpPort(ip="127.0.0.1", port=8005)

# Функция для обнаружения пожара и выполнения мероприятий БПЛА
def detect_fire(drone):
    while True:
        temperature = drone.get_piro_sensor_data()
        if temperature is not None and temperature >= 100:
            print("Fire detected!")
            drone.fire_detection()  
            time.sleep(5) 
            send_fire_location_to_RTS(drone)
            time.sleep(5)

# Функция для передачи координат места пожара РТС
def send_fire_location_to_RTS(drone):
    fire_location = drone.get_current_location()  
    fire_x, fire_y, fire_z = fire_location
    robot.move_to(x=fire_x, y=fire_y, z=fire_z)


# Функция для выполнения миссии поиска пожаров
def search_mission(drone, waypoints):
    for waypoint in waypoints:
        drone.go_to_local_point(x=waypoint[0], y=waypoint[1], z=waypoint[2])
        while not drone.point_reached():
            time.sleep(1)

# Функция для проверки столкновений БПЛА
def check_drone_collisions(drones):
    while True:
        for i in range(len(drones)):
            for j in range(i+1, len(drones)):
                distance = calculate_distance(drones[i], drones[j])
                if distance < COLLISION_DISTANCE:
                    print(f"Collision detected between drones {i} and {j}! Evading...")
                    evade_collision(drones[i], drones[j])
        time.sleep(1)

# Функция для уклонения от столкновения БПЛА
def evade_collision(drone1, drone2):
    new_x1 = drone1.x + 1.0  # Сдвигаем один БПЛА вправо
    new_x2 = drone2.x - 1.0  # Сдвигаем другой БПЛА влево
    drone1.go_to_local_point(x=new_x1, y=drone1.y, z=drone1.z)
    drone2.go_to_local_point(x=new_x2, y=drone2.y, z=drone2.z)


# Функция для проверки столкновений РТС
def check_robot_collisions(robots):
    while True:
        for i in range(len(robots)):
            for j in range(i+1, len(robots)):
                distance = calculate_distance(robots[i], robots[j])
                if distance < COLLISION_DISTANCE:
                    print(f"Collision detected between robots {i} and {j}! Evading...")
                    evade_collision(robots[i], robots[j])
        time.sleep(1)

# Функция для уклонения от столкновения
def evade_collision(robot1, robot2):
    robot1.move_to(x=robot1.x + 0.5, y=robot1.y + 0.5, z=robot1.z)
    robot2.move_to(x=robot2.x - 0.5, y=robot2.y - 0.5, z=robot2.z)

# Функция для выполнения мероприятий РТС
def perform_fire_extinguishing(robot):
    while True:
        robot.stop()
        time.sleep(5)
        robot.set_led_color(255, 0, 0)
        print("Fire extinguished!")

# Функция для расчета расстояния между объектами
def calculate_distance(obj1, obj2):
    return math.sqrt((obj1.x - obj2.x)**2 + (obj1.y - obj2.y)**2 + (obj1.z - obj2.z)**2)

# Создание класса для хранения данных о объектах
class ObjectData:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

# Создание БПЛА и РТС
drones = [ObjectData(0, 0, 0), ObjectData(0, 0, 0)]
robots = [ObjectData(0, 0, 0), ObjectData(0, 0, 0)]

# Запуск потоков для выполнения мероприятий РТС
for robot in robots:
    fire_extinguishing_thread = threading.Thread(target=perform_fire_extinguishing, args=(robot,))
    fire_extinguishing_thread.start()

# Создание дронов
drones = [
    Pioneer(ip=DroneConnectingData.drone0.ip, mavlink_port=DroneConnectingData.drone0.port),
    Pioneer(ip=DroneConnectingData.drone1.ip, mavlink_port=DroneConnectingData.drone1.port)
]

# Запуск потоков для выполнения миссии поиска пожаров
for i, drone in enumerate(drones):
    search_thread = threading.Thread(target=search_mission, args=(drone, waypoints))
    detect_fire_thread = threading.Thread(target=detect_fire, args=(drone,))
    search_thread.start()
    detect_fire_thread.start()

# Запуск потоков для проверки столкновений
drone_collision_thread = threading.Thread(target=check_drone_collisions, args=(drones,))
robot_collision_thread = threading.Thread(target=check_robot_collisions, args=(robots,))
drone_collision_thread.start()
robot_collision_thread.start()

# Основной цикл программы (можно реализовать дополнительную логику)
while True:
    time.sleep(1)