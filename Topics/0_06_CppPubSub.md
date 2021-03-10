# ROS топики публикации и подписки на основе пакета roscpp

Разработка узлов на основе С++ немного отличается от разработки на языке Python. За основу пример возьмем из [офф статьи](http://wiki.ros.org/roscpp_tutorials/Tutorials/WritingPublisherSubscriber). В данной теме разберем коды узлов `talker_cpp` и `listener_cpp` на основе ранее разработанных аналогичных узлов на языке Python. В конце темы будет ряд задач, рекомендованных к выполнению.

Поехали =)

## Cpp Publisher

За основу берется код узла
```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cpp_talker");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::String>("cpp_chatter", 1000);

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
#include <ros/ros.h>
/* Our case:     std_msgs/String     -> #include <std_msgs/String.h>      */
/* Example:      geometry_msgs/Pose  -> #include <geometry_msgs/Pose.h>   */
#include <std_msgs/String.h>

#include <sstream>
```

Далее производим подготовительный этап:
- Регистрация узла, третий аргумент - имя регистрации узла в системе
```cpp
  ros::init(argc, argv, "cpp_talker");
```

- Создание объекта [интерфейса узла](http://docs.ros.org/melodic/api/roscpp/html/classros_1_1NodeHandle.html)
```cpp
  ros::NodeHandle n;
```

- Регистрация топика `cpp_topic`, а также получение объекта публикации (заметьте, в тип шаблона передается тип сообщения). Вторым агргументом передается размер очереди сообщений
```cpp
  ros::Publisher pub = n.advertise<std_msgs::String>("cpp_chatter", 1000);
```

- Создание объекта `Rate` для реализации частоты публикации
```cpp
  ros::Rate loop_rate(1);
```

Ну а теперь немного о логике итерации цикла `while`, который проверяет условие завершения (`ros::ok()`):
- Создаем объект сообщения
```cpp
    std_msgs::String msg;
```

- Создаем объект строчного потока (он используется для формирования результирующей строки с использовани потоков)
```cpp
    std::stringstream ss;
```

- С помощью потоков записываем фиксированную строку, а также значение счетчика
```cpp
    ss << "hello world " << count++;
```

- Записываем полученную строку (функция `str()` строчного потока) в поле `data` нашего сообщения
```cpp
    msg.data = ss.str();
```

После этого выводим в консоль данные, при этом используется `c_str()`, чтобы получить строку в формате С, так как `ROS_INFO()` - аналог функции `printf()`.
```cpp
    ROS_INFO("%s", msg.data.c_str());
```

Далее публикуем сформированное сообщение и засыпаем до наступления следующего момента, чтобы выдержать частоту. Строка `ros::spinOnce();` в данном случае не играет роли, так как она используется в основном для того, чтобы наша программа проверила приходящие сообщения и вызвала соответствующие `callback` функции. То есть, строка эта должна использоваться, если узел подписывается на какой-то топик. В данной ситуации подписки нет, соответственно - строка не так важна.
```cpp
    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
```

##### > В папке `src` пакета создайте файл `talker.cpp` и разместите в нем рассмотренный код, компиляцией займемся позже =)

## Cpp Subcriber

А теперь рассмотрим код узла, который подписывается на топик и выводит полученный сообщения
```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

void topicCallback(const std_msgs::String& msg)
{
  ROS_INFO("I heard: [%s]", msg.data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("cpp_chatter", 1000, topicCallback);

  ros::spin();

  return 0;
}
```

Все те же заголовки
```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>
```

Создаем обработчик получения сообщений, в аргументе сидит ссылка на сообщение, пример доступа к полю сообщения представлен в примере. Обработчик выводит данные в сообщении в консоль.
```cpp
/* 
 * Аргумент callback-функции:
 *   const - константная ссылка, чтобы мы не меняли содержание внутри этой функции
 *   std_msgs::String - по типу топика
 *   & - ссылка на полученное сообщение, фишка С++, вместо указателей
 *   msg - название аргумента
 */
void chatterCallback(const std_msgs::String& msg)
{
  ROS_INFO("I heard: [%s]", msg.data.c_str());
}
```

Аналогичная инициализация за единственным исключением - регистрируем и получаем объект подписки. В аргументах также передается функция-обработчик, которая будет вызываться на приход сообщения по данному топику.
```cpp
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("cpp_chatter", 1000, topicCallback);
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

##### > В папке src пакета создайте файл listener.cpp и разместите в нем рассмотренный код, компиляцией займемся уже скоро =)

## Сборка пакета

Для сборки проекта мы будем редактировать файл CMakeLists.txt внутри пакета. На данный момент можно удалить все комментарии, если они сильно мешают ориентированию по файлу. С другой стороны, в комментариях написано много подсказок, так что при дальнейшей настройке можно по комментариям искать соответствующие макросы и открывать их (раскомментировать). Поехали =)

Первое на что стоит обратить внимание - секция `catkin specific configuration` макрос `catkin_package()` сейчас не имеет аргументов, так это и остается.

Далее переходим к секции `build`. Для начала определим две переменные в cmake - TALKER_NODE_NAME и TALKER_NODE_SRC. В исходном CMakeLists.txt таких закомментированных строк нет. Делается это для организации сборки. Так, если мы захотим поменять имя узла (название исполняемого файла) - меняем одну переменную, если нужно добавить исходные тексты - добавляем в другую. Делается это следующим образом
```cmake
set(TALKER_NODE_NAME talker_cpp)
set(TALKER_NODE_SRC  src/talker.cpp)
```

Первый аргумент макроса `set` - название переменной, второй и дальнейшие (можно задавать целый список) - значение(-я).

Далее указываем cmake о том, что мы планируем создавать исполняемый файл макросом `add_executable()`. В нем первый аргумент - название бинарного файла (который будет создан), второй - список файлов исходных текстов (как удачно совпало, что мы для этого определили переменные!).
```cmake
add_executable(${TALKER_NODE_NAME} ${TALKER_NODE_SRC})
```

> Знаки доллара со скобками читают значение переменной.

После этого идут две строки с макросами:
- добавляют зависимости `add_dependencies()` - в комментах пишут, что это делается для создания порядка сборки, так, например, мы будет далее создавать свои сообщения и сообщения должны быть сгенерированы раньше, чем узел
- указывают линковку библиотек `target_link_libraries()` - опять же, выше в файле перечисляются библиотеки, которые подключаются в набор `catkin`, поэтому в макросе используется переменная `catkin_LIBRARIES`.

```cmake
add_dependencies(${TALKER_NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${TALKER_NODE_NAME} ${catkin_LIBRARIES} )
```

Как видно, во всех макросах используется имя бинарного файла (`TALKER_NODE_NAME`).

На этом все, после написания данных строк в CMakeLists.txt можно перходить к компиляции. Еще раз приведу полный набор макросов для генерации узла `talker_cpp`
```cmake
set(TALKER_NODE_NAME talker_cpp)
set(TALKER_NODE_SRC  src/talker.cpp)

add_executable(${TALKER_NODE_NAME} ${TALKER_NODE_SRC})

add_dependencies(${TALKER_NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${TALKER_NODE_NAME} ${catkin_LIBRARIES} )
```

После этого можно переходить в папку вашего ws (рабочего пространства) и вызывать команду `catkin_make`. При успешной компиляции в консоли не долно быть ошибок. Также, при успешной сборке, набор команды с автодополнением
```bash
rosrun study_pkg t[TAB]
```
должен дать
```bash
rosrun study_pkg talker_cpp
```

##### > Дополните макросы для сборки узла `listener_cpp`. Хорошей практикой является выделение комментариями блоков для сборки конкретных узлов. За основу выделения можно взять присутствующие в CMakeLists.txt блоки типа build.

## В результате

- Освоили базовые навыки создания узлов на языке С++.
