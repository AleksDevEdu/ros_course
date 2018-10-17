# Создание ROS Subscriber с использованием rospy

А теперь перейдем к написанию узла подписки. Пример основы можно также подсмотреть на все той же странице о [написании узлов](http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber).

Также можно добавить к полезным ссылкам [API rospy](http://docs.ros.org/api/rospy/html/) и ссылку на общее описание [Subscribers and Publishers](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers).

Базовый код:
```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo("I heard %s", msg.data)

rospy.init_node('listener')
rospy.Subscriber('my_chat_topic', String, callback, queue_size=10)
rospy.spin()
```

Для начала, импортируем основной модуль `rospy` и модуль сообщения типа `std_msgs/String`.
```python
import rospy
from std_msgs.msg import String
```

Далее пишем обработчик приема сообщений из топика, регистрируем узел и подписываемся на топик с указанием обработчика. Обработчик вызывается каждый раз, как узел получает сообщение.
```python
def callback(msg):
    rospy.loginfo("I heard %s", msg.data)

rospy.init_node('listener')
# Не требуется сохранять объект подписки, возврат функции игнорируется
rospy.Subscriber('my_chat_topic', String, callback, queue_size=10)
```

После остается лишь оставить узел работать до завершения системы ROS или прерывания узла (Ctrl+C). В момент регистрации подписки на топик узел уже готов принимать сообщения (прием происходит в отдельном потоке).  
С одной стороны можно использовать `rospy.is_shutdown()`. Такая практика рапространена, если необходимо еще что-то делать в узле помимо приема сообщений.  
Для простого приема сообщений можно вызвать `rospy.spin()`, который будет удерживать программу рабочей до тех пор, пока ROS не завершится или узел не бует прерван. 
```python
# В данном случае достаточно спина, по факту замена `rospy.is_shutdown()`
rospy.spin()
```

##### > Создайте также и скрипт `listener.py` в пакете. Запустите оба узла и убедитесь в работоспособности (один отправляет, другой получает сообщения).

## В результате
- Был создан узел, подписывающийся на топик типа строки. Рассмотрено основное API пакета rospy.
