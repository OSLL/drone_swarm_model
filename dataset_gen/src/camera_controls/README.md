# Controller.py
## Запуск
Запуск контроллера дронов
```sh
rosrun camera_controls controller.py
```
## Работа с контроллером
Движение дрона *drone_name* в глобальные координаты *x y z*
```sh
move drone_name x y z
```
Движение дрона *drone_name* в локальные координаты *x y z*
```sh
move_direct drone_name x y z
```
Поворот дрона *drone_name* по осям roll, pitch, yaw на значения *roll pitch yaw* глобально
```sh
rotate drone_name roll pitch yaw
```
Поворот дрона *drone_name* по осям roll, pitch, yaw на значения *roll pitch yaw* локально
```sh
rotate_direct drone_name roll pitch yaw
```
Движение дрона *drone_name* в глобальные координаты *x y z* и поворот по осям roll, pitch, yaw на значения *roll pitch yaw* глобально
```sh
translate drone_name x y z roll pitch yaw
```
Движение дрона *drone_name* в локальные координаты *x y z* и поворот по осям roll, pitch, yaw на значения *roll pitch yaw* локально
```sh
translate_direct drone_name x y z roll pitch yaw
```
## Описание контроллера
При запуске контроллера пользователю выводится справка команд для управления контроллером. Контроллер считывает верно введенные команды, глобальные координаты переводит в локальные и передает обработанные параметры *x y z roll pitch yaw* в топик *drone_name/cmd_move* сообщением типа *camera_controls/msg_transposition*. В случае неверных команд или отсутствия данных о дроне выводится сообщение об ошибке.
Созданный тип сообщения имеет структуру:
```sh
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
```
# Odometry.py
## Запуск
Запуск узла одометрии дронов
```sh
rosrun camera_controls odometry.py
```
## Описание одометрии
Считываются данные об ориентации всех дронов и публикуются в инвидуальные топики *drone_name/odom* под каждый дрон раз в секунду сообщением типа *camera_controls/msg_odometry*. Когда считывание данных невозможно, раз в секунду выводится сообщение об ошибке.
Созданный тип сообщения имеет структуру:
```sh
float64 x
float64 y
float64 z
float64 w
```
# Rosbag_recorder.py
## Запуск
Запуск узла rosbag записи для дрона для одного топика
```sh
rosrun camera_controls rosbag_recorder.py /topic_name
```
Запуск узла rosbag записи для дрона для двух топиков
```sh
rosrun camera_controls rosbag_recorder.py /topic1_name /topic2_name
```
## Работа с узлом rosbag записи
Запуск записи
```sh
start record
```
Запуск записи через топик для команд
```sh
rostopic pub /rosbag_command std_msgs/String "start record"
```
Остановка записи
```sh
stop record
```
Остановка записи через топик для команд
```sh
rostopic pub /rosbag_command std_msgs/String "stop record"
```
Запуск записи на *n* секунд
```sh
record during n
```
Запуск записи на *n* секунд через топик для команд
```sh
rostopic pub /rosbag_command std_msgs/String "record during n"
```
## Описание узла rosbag записи
После верно введенного запуска узла rosbag записи проводится проверка возможности записи заданных топиков. В случае невозможности узел ждет 10 секунд, проверяя в это время доступность топиков, если через 10 секунд доступа ко всем заданным топикам нет, то программа завершает работу. Пользователя в такой ситуации уведомляет следующее сообщение *Waiting for /topics_name publishers*,в котором в случае недоступности всех заданных топиков каждую секунду добавляется точка и после 10 секунд *There is no publishers for topics: /topics_name*. В случае успешности выводится справка команд для управления записью. Узел rosbag записи считывает верно введенные команды и регулирует rosbag запись. Работает защита записи, которая не запускает новые записи, пока идет другая. Каждая запись сохраняется в свой файл *rosbag<number_of_record>.bag*. Перед каждой записью производится такая же проверка доступности всех заданных топиков, как и в начале, но в случае недоступности программа не завершает работу, вместо этого не запускается запись. В случае записи одного топика запись производится напрямую, в случае записи двух топиков запись производится в синхронизированном режиме со специальных топиков */rosbag_sync_topic* и */rosbag_sync_topic2*. Программа универсальна для любых типов сообщений.

# Parse_rosbag.py
## Запуск
Запуск парсера rosbag файла *rosbag_file_name.bag*. Опционально можно выбрать с помощью параметра *-o* имя csv файла *name_of_out_file*.
```sh
python3 parse_rosbag.py rosbag_file_name.bag -o name_of_out_file
```
## Описание парсера rosbag файла
Программа переводит bag файл в csv файл. Файл будет иметь имя либо то же, что и bag файл, либо то, что указал пользователь. Файл сохраняется в папке с именем bag файла. Программа работает с bag файлами, которые содержат один и два топика.


## driver.py
### Запуск
Запуск драйвера, который будет управлять дроном с 
именем drone_name
```bash
rosrun camera_controls driver.py __name:=drone_name
```

### Описание работы
Драйвер получает сообщения из топика drone_name/cmd_move и осуществляет
телепортацию дрона в соответствии с переданными параметрами

### Описание получаемого сообщения
Драйвер получает сообщения типа camera_controls/msg_transposition

Структура сообщения следующая
```
float64 x - смещение по оси x
float64 y - смещение по оси y
float64 z - смещение по оси z
float64 roll - угол поворота вокруг оси x 
float64 pitсh - угол поворота вокруг оси y
float64 yaw - угол поворота вокруг оси z
```
