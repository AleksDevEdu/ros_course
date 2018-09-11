# Создание ROS Subscriber с использованием rospy

А теперь перейдем к написанию узла подписки. Пример основы можно также подсмотреть на все той же странице о [написании узлов](http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber).

Также можно добавить к полезным ссылкам [API rospy](http://docs.ros.org/api/rospy/html/) и ссылку на общее описание [Subscribers and Publishers](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers).

Для начала, импортируем основной модуль `rospy` и модуль сообщения типа `std_msgs/String`.

```python
import rospy
from std_msgs.msg import String
```
Далее пишем обработчик приема сообщений из топика и регистрируем его как узел, и как подписку на топик.

```python
def callback(msg):
    rospy.loginfo("I heard %s", msg.data)

rospy.init_node('listener')
# Не требуется сохранять объект подписки, возврат функции игнорируется
rospy.Subscriber('my_chat_topic', String, callback)
```

После остается лишь оставить узел работать до завершения системы ROS или прерывания узла (Ctrl+C). В момент регистрации подписки на топик узел уже готов принимать сообщения.  
С одной стороны можно использовать `rospy.is_shutdown()`. Такая практика рапространена, если необходимо еще что-то делать в узле помимо приема сообщений.  
Для простого приема сообщений можно вызвать `rospy.spin()`, который будет удерживать программу рабочей до тех пор, пока ROS не завершится или узел не бует прерван. 

```python
# В данном случае достаточно спина
rospy.spin()
```