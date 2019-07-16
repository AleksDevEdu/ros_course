# Designations

[Что-то, что надо заменить] - скобки, обычно используется в описании команд, так сказать аргументы команды.
##### > Задание, которое надо выполнить

# FAQ - Frequently Asked Question

## Как перейти в папку с пакетом?

Утилита `roscd` в помощь [T2]:
```bash
roscd study_pkg
```

## Как вывести список пакетов, которые видит ROS?

```bash
rospack list
```

## Как проверить, что ROS в системе настроен?

При установке ROS настраиваются следующие переменные окружения:
- ROS_DISTRO
- ROS_MASTER_URI
- ROS_ROOT
- ROS_ETC_DIR
- ROS_PACKAGE_PATH
- ROS_VERSION

Проверить их можно командой `echo`:
```bash
echo $ROS_DISTRO
```

Для ROS Kinetic результат будет `kinetic`.

## Как настроить сеть для взаимодействия нескольких машин?
- [Туториал](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)  
- [Статья](http://wiki.ros.org/ROS/NetworkSetup)

За работу по сети отвечают переменные `ROS_MASTER_URI`, `ROS_IP` и `ROS_HOSTNAME`  

| Переменная       | Назначение
|-----------------:|------------
|`ROS_MASTER_URI`  | Расположение мастера в сети, по-умолчанию настроен на локальную машину `http://localhost:11311`
|`ROS_IP`          | Адрес машины в сети, задается ip адрес
|`ROS_HOSTNAME`    | Имя машины в сети, вместо имени можно также задать адрес

> Думаю понятно, что машины для общения должны быть в одной сети =)

## Как настроить прокси в университете ЛЭТИ?

Скрипты для включения/отключения можно найти [здесь](https://gist.github.com/KaiL4eK/1c4b4f5581877cbd635c23594433d42b).

# ROS Utility list

[T0_1]
- catkin_make

[T0_2]
- rosbash (package)
- rospack

[T0_3]
- roscore
- rosnode
- rostopic
- rosmsg
- rqt_graph

[T0_8]
- roslaunch

[T0_10]
- rossrv
- rosservice

[T0_11]
- rosparam

[T1_4]
- rosed

