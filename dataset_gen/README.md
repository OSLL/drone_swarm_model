# Dataset generator

## Запуск Docker-контейнера
Сборка образа и запуск контейнера
```bash
cd dataset_gen
./scripts/build_and_start_docker.sh
```

## Использование

### Создание launch-файла для N дронов (камер)
Создание launch-файла (ROS) для эмуляции N камер
```bash
rosrun cottage configure_launch.py --num N > new_file.launch
```

### Запуск тестовой симуляции
```bash
roslaunch cottage cottage_blender.launch
# or
roslaunch cottage <launch_for_N_drones>.launch
```
Для N=4 в пакете `cottage` есть два launch-файла: `test_4.launch` и `test_4_headless.launch` (запуск без GUI)
```bash
roslaunch cottage test_4.launch
# or
roslaunch cottage test_4_headless.launch
```
_Note: если вовремя запуска возникли ошибки - повторите запуск (`Ctrl+C` для отмены текущего процесса)_

### Запуск генерации
#### Подключение второго терминала к контейнеру
На случай, если tmux для разделения терминала не работает:
```bash
docker exec -it dataset_gen bash
```
#### Скрипт для генерации
```bash
cd src/cottage
mkdir images
rosrun camera_controls trajectory.sh <trajectory_file>
```
Для базовой модели в симулятора и N=4 добавлен файл `camera_controls/test/test_trajectory_4.txt` с нужными координатами
```bash
cd cottage
mkdir images
rosrun camera_controls trajectory.sh /catkin_ws/src/camera_controls/test/test_trajectory_4.txt
```
Фотографии будут сохраняться в текущую папку (`cottage/images/`) - благодаря маппингу `src/` на хостовую машину, они доступы на хосте в соответствующей папке `dataset_gen/src/cottage/images`