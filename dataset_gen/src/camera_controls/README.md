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
Контроллер считывает верно введенные команды, глобальные координаты переводит в локальные и передает обработанные параметры *x y z roll pitch yaw* в топик *drone_name/cmd_move* сообщением типа *camera_controls/msg_transposition*.
Созданный тип сообщения имеет структуру:
```sh
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
```
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
