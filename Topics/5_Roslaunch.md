# Roslaunch

Запуск узлов в большом количестве скорее всего потребует соответствующего количества терминалов + 1 (для мастера). В реальных системах может присутствовать 10, 20 и более узлов, что может вызвать огромную боль при включении/выключении/проверке всех узлов.

Для облегчения жизни придумали специальный формат, основанный на формате `xml`. Подерживаемые тэги описаны на [офф странице](http://wiki.ros.org/roslaunch/XML). Суть данного формата в том, что он позволяет настраивать и запускать группы узлов.

Для начала, попробуем рассмотреть простой launch-файл (так они называются, а хранятся внутри пакета в папке...ни за что не догадаетесь... `launch`).

```xml
<launch>
    <node name="listener" pkg="rospy_tutorials" type="listener.py" output="screen"/>
    <node name="talker" pkg="rospy_tutorials" type="talker.py" output="screen"/>
</launch>
```

Основа launch-файла лежит в тэге `<launch>`, он оборачивает весь файл.

Далее вложенные тэги `<node>` задают запуск узлов. В качестве параметров тэгов указываются:
- name - имя, которое присваивается узлу в системе ROS (аналог `__name`)
- pkg - название пакета, внутри которого лежит узел
- type - название узла внутри пакета
- output - режим вывода информации, если варианты `screen` и `log`

> При запуске launch-файла также запускается мастер (roscore), если он не был запущен ранее

Таким файлом из примера удобно пользоваться, так как вместо трех консолей потребуется единственная, в которую будет выкладываться вывод всех узлов, у которых `output="screen"`.

Утилита для запуска называется `roslaunch` и вот пример запуска такого файла из пакета `rospy_tutorials` (он там есть)
```bash
roslaunch rospy_tutorials talker_listener.launch
```

Выключение всех узлов из файла производится нажатием Ctrl+C в терминале.

> Попробуйте написать свой launch-файл с таким же содержанием и запустить его. Для этого нужно в пакете создать папку `launch` и в ней создать файл с расширением launch. С помощью утилиты roslaunch запустите файл из своего пакета и убедитесь, что все работает.

## Распространенные практики

А теперь поговорим о наиболее применяемх практиках относительно launch-файлов.

### Объединение узлов под одним пространством имен

Допустим мы хотим запустить узлы в одном пространстве имен, так как один выполняют определенную задачу (являются подсистемой). Можно это сделать красиво с помощью тэга `<group>`.

```xml
<launch>
    <group ns="my_namespace">
        <node name="listener" pkg="rospy_tutorials" type="listener.py" output="screen"/>
        <node name="talker" pkg="rospy_tutorials" type="talker.py" output="screen"/>
    </group>
</launch>
```

### Мапирование топиков

Часто неоходимо переименовать (мапировать) топики узлов. Делается это тэгами `<remap>` внутри тэга `<node>`.

```xml
<launch>
    <node name="listener" pkg="rospy_tutorials" type="listener.py" output="screen">
        <remap from="chatter" to="my_topic"/>
    </node>
    <node name="talker" pkg="rospy_tutorials" type="talker.py" output="screen">
        <remap from="chatter" to="my_topic"/>
    </node>
</launch>
```

### Подключение других launch-файлов

Иногда можно написать много простых launch-файлов и запустить все их с помощью одного launch-файла. Для этого существует тэг `<include>`.

```xml
<launch>
    <include file="$(find study_pkg)/launch/otherfile.launch" />
    <node name="listener" pkg="rospy_tutorials" type="listener.py" output="screen"/>
    <node name="talker" pkg="rospy_tutorials" type="talker.py" output="screen"/>
</launch>
```

Директива `(find study_pkg)` ищет пакет, имя которого передано аргументом и подставляет путь до него в случае удачного нахождения. Таким образом выполняется сначала launch-файл `otherfile.launch`, а затем остальное содержимое. Уровни вложенности launch-файлов не ограничены (насколько я знаю).

### Создание опций для launch-файлов

Иногда создание опреленной системы упрощается, если при запуске существует возможность передать опции файлу запуска. Для launch-файлов существует тэг `<arg>`, который добавляет аргументы launch-файлу.

```xml
<launch>
    <arg name="new_topic_name" default="chatter" />

    <node name="listener" pkg="rospy_tutorials" type="listener.py" output="screen">
        <remap from="chatter" to="$(arg new_topic_name)"/>
    </node>
    <node name="talker" pkg="rospy_tutorials" type="talker.py" output="screen">
        <remap from="chatter" to="$(arg new_topic_name)"/>
    </node>
</launch>
```

Директива `(arg new_topic_name)` подставляет значение аргумента. При наличии параметра `default` в тэге `<arg>` установка параметра при запуске launch-файла не обязательна. Для задания значения аргумента выполнение roslaunch происходит следующим образом

```bash
rolaunch rospy_tutorials talker_listener.launch new_topic_name:=my_topic
```

> Попробуйте каждую из представленных практик и проверь корректность с помощью рассмотренных ранее утилит.

## В результате

- Мы научились пользоваться утилитой roslaunch и создавать launch-файлы.
- Рассмотренные практики позволили также рассмотреть ряд тэгов, используемых в данном формате. Остальные тэги можно найти на офф страницах (ссылка есть в начале).
