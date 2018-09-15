# Работа с изображениями

Работа с ихображениями в ROS не представляет большой проблемы, но требует знания некоторых особенностей. Основной принцип работы с изображениями следующий - для получения изображения из сообщения, а также обратно, для размещения изображения в сообщении, используется специальный объект под названием [cv_bridge](http://docs.ros.org/kinetic/api/cv_bridge/html/python/). Из названия понятно, что он представляет "мост" между изображением в формате `CV` (`OpenCV`) и изображением в формате сообщения ROS. За основу возьмем [туториал по этой теме](http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython).

API для преобразования достаточно простое, необходимо создать объект `cv_bridge` (не забываем добавить модуль!):
```python
from cv_bridge import CvBridge

cv_bridge = CvBridge()
```
после можем производить преобразование из сообщения ROS типа `sensor_msgs/Image` в изображение формата `OpenCV`:
```python
cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
```
Аргумент `desired_encoding` сообщает, в каком формате представить изображение. Для `OpenCV` стандартным форматом является BGR (blue - green - red). Существуют следующие форматы:
- mono8: 8-битное серый
- mono16: 16-битный серый
- bgr8: BGR
- rgb8: RGB
- bgra8: BGR с афльфа каналом
- rgba8: RGB с альфа каналом

Аналогично, обратно в сообщение:
```python
image_message = cv2_to_imgmsg(cv_image, encoding="bgr8")
```
В данном случае аргумент `encoding` сообщает, в каком формате находится изображение.

> С кодированием формата важно не промахнуться, если постараться скормить функциям `OpenCV` формат RGB - цвет будет искажен (просто поменян порядок цветов, но все равно плохо).

В остальном, вся работа с изображением как шла, так и идет. Рассмотрим пример узла:
```python
#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def callback(msg):
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)

    blurred = cv2.blur(cv_image,(5,5))

    try:
        image_pub.publish(cv_bridge.cv2_to_imgmsg(blurred, "bgr8"))
    except CvBridgeError as e:
        rospy.logerr(e)


rospy.init_node('image_converter', anonymous=True)
image_pub = rospy.Publisher("image_topic_out", Image, queue_size=10)
cv_bridge = CvBridge()
rospy.Subscriber("image_topic_in", Image, callback, queue_size=10)

try:
    rospy.spin()
except KeyboardInterrupt:
    rospy.logerr("Shutting down")
```


Также не забудьте добавить в `package.xml` следующие зависимости:
```
sensor_msgs
opencv2
cv_bridge
rospy
std_msgs
```


