# ROS TF Listener

В основе системы из примера
```bash
roslaunch turtle_tf turtle_tf_demo.launch
```
лежал принцип преобразования из топика информации о положении в систему TF с помощью узлов `/turtle*_tf_broadcaster` для каждой черепашки. Затем черепашка два использовала TF listener API, чтобы с помощью системы TF следовать за первой черепашкой. В данном топике предлагается воспользоваться TF listener API, чтобы следовать за морковкой первой черепашки.

Взглянем на исходный код узла, который создает вторую черепашку и двигает ее к первой:
```bash
roscat turtle_tf turtle_tf_listener.py
```

```python
#!/usr/bin/env python
# Здесь была лицензия =)

import rospy

import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf_turtle')

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        turtle_vel.publish(msg)

        rate.sleep()
```

Много интересного и много знакомого, но есть и новые фишки. Есть вызов сервиса `spawn`. Давайте посмотрим информацию о сервисе:
```bash
rosservice info /spawn
```
```
Node: /simulator
URI: rosrpc://user-vb:58409
Type: turtlesim/Spawn
Args: x y theta name
```

Так вызов сервиса в узле
```python
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')
```
аргументы соответствуют x, y, theta, name. Сам сервис создает черепашку по координатам, задает поворот и присваивает имя.

Теперь обратим внимание на вызовы:
```python
    listener = tf.TransformListener()
    (trans, rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time())
```
Создается объект, а затем, задавая сначала имя системы координат, данные по которой хотим получить, затем относительно которой хотим получить, и в конце время (пока просто время), мы получаем `trans` - вектор линейного смещения между черепашками, `rot` - вектор поворота черепашек относительно друг друга.

Все это используется для расчета задания скорости черепашки и передачи в топика задания `turtle2/cmd_vel`:
```python
    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
...
        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)

        msg = geometry_msgs.msg.Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        turtle_vel.publish(msg)
```

В остальном на данный момент код не должен вызывать трудностей, так как ранее уже разбиралось все, но если вопросы есть - обязательно задавайте, тема не такая уж и простая.

А теперь сделаем код немного проще и зададимся нашей начальной целью - (черепашка2 -> морковка черепашки1):
```python
#!/usr/bin/env python
import rospy

import math
import tf
from geometry_msgs.msg import Twist
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf_turtle')

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(10.0)
    msg = Twist()

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/turtle2', '/carrot', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        msg.linear.x = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        msg.angular.z = 4 * math.atan2(trans[1], trans[0])

        turtle_vel.publish(msg)

        rate.sleep()
```

##### > Задача - создайте в своем пакете (study_pkg) Python скрипт `turtle_tf_listener.py`. Не забудьте назначить права на исполнения файла утилитой `chmod` [T6].

##### > Дополните launch-файл запуска первой черепашки с морковкой:
- Запустить `turtle_tf_broadcaster.py` из нашего пакета и не забудьте смапировать топик из `input_pose` в `turtle2/pose`. Также не забудьте приватному параметру `turtle_tf_name` присвоить значение `turtle2`. Переименовать в `turtle2_tf_broadcaster`.
- Запустить `turtle_tf_listener.py` из нашего пакета.

В результате успешного выполнения коварная черепашка2 должна пытаться съесть морковку черепашки1.

## В результате
- Научились пользоваться программными средствами получения данных TF в узле.
- На самом деле по поводу времени преобразования есть еще ряд тем, которые стоит коснуться для продвинутого использвания, вот [ссылка1](http://wiki.ros.org/tf/Tutorials/tf%20and%20Time%20%28Python%29) и [ссылка2](http://wiki.ros.org/tf/Tutorials/Time%20travel%20with%20tf%20%28Python%29).
