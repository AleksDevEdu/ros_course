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

## Как настроить прокси в университете ЛЭТИ?

В папке с репозиторием `ros_course/vm_tools` есть скрипт `proxy_leti.sh`. Его выполнение с правами админа (команда `sudo` - потребует пароль) настраивает систему и `git` на стандартный прокси:
```bash
sudo ~/ros_course/vm_tools/proxy_leti.sh
```

Для отключения:
```bash
sudo ~/ros_course/vm_tools/proxy_off.sh
```

# ROS Utility list

[T1]
- catkin_make

[T2]
- rosbash (package)
- rospack

[T3]
- roscore
- rosnode
- rostopic
- rosmsg
- rqt_graph

[T8]
- roslaunch

[T10]
- rossrv
- rosservice

[T11]
- rosparam

