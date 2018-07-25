# ROS топики публикации и подписки на основе пакета roscpp

Разработка узлов на основе С++ немного отличается от и разработки на языке Python. За основу пример возьмем из [офф статьи](http://wiki.ros.org/roscpp_tutorials/Tutorials/WritingPublisherSubscriber). В данной теме разберем коды узлов `talker_cpp` и `listener_cpp` на основе ранее разработанных аналогичных узлов на языке Python. В конце темы будет ряд задач, рекомендованных к выполнению.

Поехали =)

## Cpp Publisher

За основу берется код узла
```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cpp_talker");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::String>("cpp_topic", 1000);

  ros::Rate loop_rate(1);

  int count = 0;
  while ( ros::ok() )
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count++;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
```

Начинаем разбор c подключения заголовков ROS и сообщения `std_msgs/String`, а также заголовок функций формирования строк `sstream` ([информация о нем](http://www.cplusplus.com/reference/sstream/stringstream/)).
```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
```

Далее производим подготовительный этап:
- Регистрация узла
- Создание объекта [интерфейса узла](http://docs.ros.org/kinetic/api/roscpp/html/classros_1_1NodeHandle.html)
- Регистрация топика `cpp_topic`, а также получение объекта публикации (заметьте, в тип шаблона передается тип сообщения). Вторым агргументом передается размер очереди сообщений
- Создание объекта `Rate` для реализации частоты публикации
```cpp
  ros::init(argc, argv, "cpp_talker");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::String>("cpp_topic", 1000);

  ros::Rate loop_rate(1);
```

Ну а теперь немного о логике итерации цикла `while`:
- Создаем объект сообщения (в c++ без этого никуда по сравнению с python)
- Создаем объект строчного потока (он используется для формирования результируюей строки с использовани потоков)
- С помощью потоков записываем фиксированную строку, а также значение счетчика
- Записываем полученную строку (функция `str()` строчного потока) в поле `data` нашего сообщения
```cpp
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count++;
    msg.data = ss.str();
```

После этого выводим в консоль данные, при этом используется `c_str()`, чтобы получить строку в формате С, так как `ROS_INFO()` - функция языка С.
```cpp
    ROS_INFO("%s", msg.data.c_str());
```

Далее публикуем сформированное сообщение и засыпаем до наступления следующего момента, чтобы выдержать частоту. Строка `ros::spinOnce();` в данном случае не играет роли, так как она используется в-основном для того, чтобы наша программа проверила приходящие сообщения и вызвала соответствующие callback функции. То есть, строка эта должна использоваться, если узел подписывается на какой-то топик. В данной ситуации подписки нет, соответственно - строка не так важна.
```cpp
    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
```

> В папке src пакета создайте файл talker.cpp и разместите в нем рассмотренный код, компиляцией займемся позже =)

## Cpp Subcriber

А теперь рассмотрим код узла, который подписывается на топик и выводит полученный сообщения
```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void topicCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("cpp_topic", 1000, topicCallback);

  ros::spin();

  return 0;
}
```

Все те же заголовки
```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
```

Создаем обработчик получения сообщений, в аргументе сидит ссылка на сообщение, пример доступа к полю сообщения прелставлен в примере. Обработчик выводит данные в сообщении в консоль.
```cpp
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
```

Аналогичная инициализация за единственным исключением - регистрируем и получаем объект подписки. В аргументах также передается функция-обработчик, которая будет вызываться на приход сообщения по данному топику.
```cpp
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("cpp_topic", 1000, topicCallback);
```

Вызов функции бесконечной обработки событий прихода сообщений через топик.
```cpp
  ros::spin();
```

По факту последний кусочек можно алтернативно написать таким образом, что он бесконечно проверяет приходящие сообщения пока система ROS в рабочем состоянии
```cpp
  while ( ros::ok() )
  {
      ros::spinOnce();
  }
```

> В папке src пакета создайте файл listener.cpp и разместите в нем рассмотренный код, компиляцией займемся уже скоро =)

## Compilation

## Задачи

## В результате
