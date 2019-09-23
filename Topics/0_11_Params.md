# ROS параметры

> [Google-презентация (слайд 5)](https://docs.google.com/presentation/d/1eUkYZY5Bu6ioSfEmm5pyLeLaD0AQtegd/edit#slide=id.p4)

Есть еще один аспект, который называется [__сервер параметров__](http://wiki.ros.org/Parameter%20Server).

__Параметрами__ в ROS называются просто данные, которые хранятся под определенными именами и пространствами имен. Как было рассмотрено ранее, запуск узла в пространстве имен меняет конечное имя узла, а также топика. Аналогично с этим, вся работа узла с параметрами (чтение, запись) происходит в том пространстве имен, которому он принадлежит.

> Сервер параметров хранит параметры и привязан к мастеру. Перезапуск мастера приводит к потере всех ранее заданных параметров.

Пора знакомиться с основной утилитой работы с параметрами =)
```bash
rosparam help
```
```
rosparam is a command-line tool for getting, setting, and deleting parameters from the ROS Parameter Server.

Commands:
    rosparam set    set parameter
    rosparam get    get parameter
    rosparam load   load parameters from file
    rosparam dump   dump parameters to file
    rosparam delete delete parameter
    rosparam list   list parameter names
```

Попробуем проверить список параметров в системе
```bash
rosparam list
```
```
/rosdistro
/roslaunch/uris/host_user_vb__35559
/rosversion
/run_id
```

Давайте поработаем с параметром /rosdistro
```bash
rosparam get /rosdistro
```
```
kinetic
```
Ожидаемый вывод, не так ли?)

А теперь попробуем задать свой параметр и сразу прочитать его
```bash
rosparam set /my_param 'Hello =)'
rosparam set /my_set '{ 'P': 10.0, 'I': 1.0, 'D' : 0.1 }'

rosparam get /my_param
rosparam get /my_set
rosparam get /my_set/P
```
Результат:
```
Hello =)
{D: 0.1, I: 1.0, P: 10.0}
10.0
```

Вроде все логично =) А теперь попробуйте перезапустить ячейку с выводом списка параметров в системе.

Как видно из вывода хелпа, параметрами также можно управлять, удаляя их, также выгружать в файл и загружать из файла.

## ROS Python параметры

Теперь рассмотрим применение параметров внутри узлов. Опираться рекомендуется на [страницу из туториала про параметры](http://wiki.ros.org/rospy_tutorials/Tutorials/Parameters).
Дальнейшие примеры можно производить, вызвав в терминале команду `python`, тогда у вас откроется консоль Python и вводить туда по очереди. Или можно написать все в один скрипт с выводом с помощью `print()` или `rospy.loginfo()`. 

Для начала стандартный и знакомый для Python узла код:
```python
import rospy
rospy.init_node('params_study')
```

Ну и начнем рассматривать, что же можно сделать с параметрами в `rospy`?  
Рассмотрим основные типы обращений к параметрам:
```python
# Обращение глобально - как видно в начале стоит `/`
# Не обращаем внимания на ns, ищем именно такой путь параметра и никак иначе
distro = rospy.get_param('/rosdistro')

# Обращение локально - в начале не стоит `/`
# Допустим мы запустили узел, указав ns:=my_ns
# Тогда вызов данной функции будет пытаться найти парметр по пути - `/my_ns/my_set` 
my_set_param = rospy.get_param('my_set')

# Обращение приватно, поиск будет по пути /params_study/private_param
# Если задат ns - он будет добавлен перед именем узла
# Например, ns:=my_ns -> /my_ns/params_study/private_param
my_private_param = rospy.get_param('~private_param')
```

Теперь на примере, можно установить разные типы параметров и посмотреть, как они будут формироваться:
```python
# Зададим параметры из узла, локальный, глобальный и приватный
# Первый агрумент - название параметра, второй - значение
rospy.set_param('~ros_priv_param', 'Hi, I am private =)')
rospy.set_param('ros_loc_param', 'Hi, I am local =)')
rospy.set_param('/ros_glob_param', 'Hi, I am global =)')
```

Выведем список после задания параметров (для примера было задано дополнительный ns - `sample_ns`):
```bash
# Теперь проверим список параметров в системе
rosparam list
```
```
/sample_ns/params_study/ros_priv_param
/sample_ns/ros_loc_param
/ros_glob_param
```

Все получилось! Глобальный не имеет префикса ns. Приватный отличается тем, что в его префиксе присутствует имя узла. Значит он относится конкретно к узлу. Соответственно, можно запустить много одинаковых узлов (например с помощью флага анонимности) и получить такое же количество параметров.

##### > Задачка - с помощью утилиты `rosparam` проверьте значения заданных параметров `ros_priv_param`, `ros_glob_param`, `ros_loc_param`

Значит так, мы научились получать параметры от сервера параметров, задавать (если их не существовало - создавать, функция все равно одна и та же). Еще один момент. Бывает такое, хотим работать с параметром, а его нету в сервере (причины могут быть разные). В этом случае при запросе происходит это:
```python
not_exist_param = rospy.get_param('i_do_not_exist')
```
```
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
  File "/opt/ros/melodic/lib/python2.7/dist-packages/rospy/client.py", line 465, in get_param
    return _param_server[param_name] #MasterProxy does all the magic for us
  File "/opt/ros/melodic/lib/python2.7/dist-packages/rospy/msproxy.py", line 123, in __getitem__
    raise KeyError(key)
KeyError: 'i_do_not_exist'
```

Огромный ERROR, ааа!  
Не паникуем, сейчас будет финт ушами: для начала решим проблему по пути Python - ловим все исключения (ошибки) и после обрабатываем:
```python
try:
    not_exist_param = rospy.get_param('i_do_not_exist')
except:
    # Not exist or any kind of other problem - set default
    not_exist_param = 'Okay, now it`s default time =0'
```

А еще можно задать значение по-умолчанию прямо в вызов вторым аргументом:
```python
not_exist_param = rospy.get_param('i_do_not_exist', 'default_value')
```

И все работает! По факту Python трюк проделывает внутри себя эта функция, если мы задаем значение по-умолчанию =)  
Вот так мы научились еще и обрабатывать получение значения по-умолчанию. Есть еще функционал из туториала, удаление параметра, проверка существования параметра и получение списка. Зачем это может понадобиться решайте сами.
```python
# Мы этот параметр ставили ранее
param_name_2_delete = '/ros_glob_param'

# Проверим список параметров, только уже через Python
param_list = rospy.get_param_names()
rospy.loginfo(param_list)

# Наличие можно проверить через функционал ROS    
if rospy.has_param(param_name_2_delete):
    rospy.loginfo('[ROSWay] Parameter exist')
else:
    rospy.loginfo('[ROSWay] Parameter not exist')
    
# И с проверкой удаляем его
if rospy.has_param(param_name_2_delete):
    rospy.delete_param(param_name_2_delete)
    
# Еще раз проверим:
if rospy.has_param(param_name_2_delete):
    rospy.loginfo('[ROSWay] Parameter exist')
else:
    rospy.loginfo('[ROSWay] Parameter not exist')
```

Если вывод похож на
```
[ROSWay] Parameter exist
[ROSWay] Parameter not exist
```
значит все окей! До вызова удаления параметр существовал, после - нет =)

Вообще, при желании глянуть на офф доки - можно воспользоваться страницей [API rospy](http://docs.ros.org/api/rospy/html/).

## Управление параметрами

Данная тема может в будущем немного подрасти, сейчас просто хотелось бы обратить небольшое внимание на приватные параметры с точки зрения практики. Обычно узлы стартуют с помощью launch-файлов, поэтому задаются параметры внутри с помощью тэгов `<param>`. Пример из одного из файлов планера:
```xml
    <param name="base_global_planner" value="global_planner/GlobalPlanner" />
    <param name="planner_frequency" value="1.0" />
    <param name="planner_patience" value="5.0" />
```
Таким образом задаются локальные параметры (с учетом ns).

Еще немного для понимания, пример из драйвера камеры (внутри тэга `<node>` параметры задаются приватными!):
```xml
<node ns="stereo" name="left_camera" pkg="usb_cam" type="usb_cam_node" output="screen" >
    <param name="video_device" value="/dev/video0" />
	<param name="image_width" value="640" />
	<param name="image_height" value="480" />
</node>
```

Здесь с помощью параметров задается путь девайса (конкретный, так как таких может быть много) и размеры выходного изображения.

##### > Задание для рассуждения - обсудите с коллегами начинку тэга <node> в примере драйвера камеры.

### Сохранение и загрузка параметров

Так как при закрытии мастера параметры теряются, было бы неплохо узнать, а как сохранять параметры в файл? А как загружать из файла? Такой функционал есть и утилита все та же:
```bash
# Сохраним все параметры в файл '/tmp/my_dump.yaml' (обычно расширение '.yaml') начиная с пространства имен '/' - то есть все параметры
# Флаг -v для визуального контроля
rosparam dump -v '/tmp/my_dump.yaml' '/'
```

```bash
# Теперь загрузим параметры из файла '/tmp/my_dump.yaml', но в новое пространство имен
rosparam load -v '/tmp/my_dump.yaml' '/my_new_ns_just_to_make_it_long_for_control'
```

```bash
# Взглянем на список параметров
rosparam list
```
```
/my_new_ns_just_to_make_it_long_for_control/ros_glob_param
/my_new_ns_just_to_make_it_long_for_control/rosdistro
/my_new_ns_just_to_make_it_long_for_control/roslaunch/uris/host_user_vb__38669
/my_new_ns_just_to_make_it_long_for_control/rosversion
/my_new_ns_just_to_make_it_long_for_control/run_id
/my_new_ns_just_to_make_it_long_for_control/sample_ns/params_study/ros_priv_param
/my_new_ns_just_to_make_it_long_for_control/sample_ns/ros_loc_param
/ros_glob_param
/rosdistro
/roslaunch/uris/host_user_vb__38669
/rosversion
/run_id
/sample_ns/params_study/ros_priv_param
/sample_ns/ros_loc_param
```

Как видим, у нас появилась полная копия параметров, только в новом пространстве имен. Еще немного практики для понимания пространства имен

```bash
# Сохраним параметры только из пространства /sample_ns
rosparam dump -v '/tmp/my_dump_special_ns.yaml' '/sample_ns'
```


```bash
# Загрузим их в новое пространство для пущего примера
rosparam load -v '/tmp/my_dump_special_ns.yaml' '/new_ns_for_special'
```

```bash
# Взглянем на список параметров
rosparam list
```
```
/my_new_ns_just_to_make_it_long_for_control/ros_glob_param
/my_new_ns_just_to_make_it_long_for_control/rosdistro
/my_new_ns_just_to_make_it_long_for_control/roslaunch/uris/host_user_vb__38669
/my_new_ns_just_to_make_it_long_for_control/rosversion
/my_new_ns_just_to_make_it_long_for_control/run_id
/my_new_ns_just_to_make_it_long_for_control/sample_ns/params_study/ros_priv_param
/my_new_ns_just_to_make_it_long_for_control/sample_ns/ros_loc_param
/new_ns_for_special/params_study/ros_priv_param
/new_ns_for_special/ros_loc_param
/ros_glob_param
/rosdistro
/roslaunch/uris/host_user_vb__38669
/rosversion
/run_id
/sample_ns/params_study/ros_priv_param
/sample_ns/ros_loc_param
```

А теперь мозговой штурм! На этом моменте можно очень хорошо понять принцип пространства имен:
На это можно смотреть как на систему папок. Если мы указываем для сохранения конкретное пространство, то все, что лежит внутри ns (далее за `/` этой папки) будет сохранено. При загрузке, мы указываем папку, с которой начать запись. 

Думаю, объяснять функционал `delete` сильно не стоит:

```bash
# Сносим целое пространство и смотрим, что получилось
rosparam delete -v '/my_new_ns_just_to_make_it_long_for_control'

rosparam list
```
```
/new_ns_for_special/params_study/ros_priv_param
/new_ns_for_special/ros_loc_param
/ros_glob_param
/rosdistro
/roslaunch/uris/host_user_vb__38669
/rosversion
/run_id
/sample_ns/params_study/ros_priv_param
/sample_ns/ros_loc_param
```

Вот и подчистили наш сервер параметров =)  
Теперь к практическим навыкам - вспомним, что запуск launch-файла запускает также и мастера, если тот ранее не был запущен. А сервер параметров завязан на мастера. Значит может понадобиться функционал загрузки параметров на момент запуска узлов - есть он у меня для вас =)  
```xml
<rosparam file="config/costmap_common.yaml" command="load" ns="global_costmap" />
```

В этом примере показан тэг `<rosparam>` и его параметры. На самом деле, параметры схожи с опциями утилиты:
- file - файл с сохраненными/подгружаемыми параметрами;
- command - может быть [load / dump / delete];
- ns - пространство имен, куда загрузить / откуда сохранить / что удалить.

Формат <имя параметра> : <значение> - это специальный формат файлов `YAML`. Для массивов и вложенных параметров происходит обертка вложенности скобками `{}` или просто новой строкой и внутри по идентичному принципу.
```yaml
ros_glob_param: Hi, I am global =)
rosdistro: 'kinetic'
roslaunch:
  uris: {host_user_vb__38669: 'http://user-vb:38669/'}
run_id: 1b078410-b789-11e8-91b9-0800278832b1
sample_ns:
  params_study: {ros_priv_param: 'Hi, I am private =)'}
  ros_loc_param: Hi, I am local =)
```

## Параметры в C++

Чтож, во-первых, предлагаю посмотреть [туториал на офф странице](http://wiki.ros.org/roscpp_tutorials/Tutorials/Parameters). Во-вторых, принцип работы аналогичный, поэтому сильно расписывать и рассматривать не будем, обсудим только базовый функционал.

> Весь дальнейший функционал принадлежит классу `NodeHandle` и требует созданного объекта этого класса. Далее полагаем, что где-то вызвали `ros::NodeHandle n;` Вот вам еще [API NodeHandle](http://docs.ros.org/kinetic/api/roscpp/html/classros_1_1NodeHandle.html).

```cpp
bool getParam (const std::string& key, parameter_type& output_value) const;
```

Функция получает значение параметра по переданному ключу (имени параметра). `parameter_type` соответствует типу значения параметра (`string`, `bool`, `int`, `double`). Есть еще специальный тип `XmlRpcValue`, но его рассморение не та важно и может быть рассмотрено из [документации](http://docs.ros.org/kinetic/api/xmlrpcpp/html/classXmlRpc_1_1XmlRpcValue.html). Возвращает функция `true`, если параметр был удачно получен, и `false`, если параметр не удалось получить (не существует, например). Пример получения параметра:
```cpp
    std::string s;
    if (n.getParam("my_param", s))
    {
      ROS_INFO("Got param: %s", s.c_str());
    }
    else
    {
      ROS_ERROR("Failed to get param 'my_param'");
    }
```
В строку `s` будет записано значение параметра `my_param`, если таковой существует. Иначе будет выполнен блок `else`.

Функция в примере устанавливает параметр. Первый агрумент - имя параметра, второй - значение. Значение может быть типами `string`, `bool`, `int`, `double` или `XmlRpcValue`.
```cpp
n.setParam("my_param", "hello there");
```

Пример на проверку наличия параметра на сервере и удаления. Функции достаточно говорящие =)
```cpp
if ( n.hasParam("my_param") )
{
    ROS_INFO("Delete param named 'my_param'");
    n.deleteParam("my_param");
}
```

## В результате

- Мы познакомились с сервером параметров и утилитой работы с параметрами.
- Научились пользовать параметры в Python и С++.
- Рассмотрели практические применения утилит и параметров в rolaunch.
- Познакомились с утилитой `rosparam`.
