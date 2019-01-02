# Создадим нового робота

В этом топике мы будем творить, придерживаться постараемся [этого туториала](http://wiki.ros.org/urdf/Tutorials) =)

> В качестве базы пользуйтесь [описанием XML формата](http://wiki.ros.org/urdf/XML).

Есть разные форматы для создания описания роботов, мы же будем использовать URDF. Давайте зададимся целью, будем создавать машинку, автомобиль.

Подготовимся, создаем файл `my_robot.urdf` в папке `descriptions` в нашем пакете. В него записываем следующую начинку:
```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

Разберем немного
```xml
<!-- Просто определяем в начале файла, что файл XML формата -->
<?xml version="1.0"?>
<!-- Главный тэг <robot>, в нем есть только аттрибут name - имя робота -->
<robot name="my_robot">
	<!-- Тэг <link> определяет звено робота, у звена должно быть уникальное имя -->
  <link name="base_link">
  	<!-- Тэг <visual> определяет визуальное представление предмета -->
    <visual>
    	<!-- Тэг <geometry> описывает геометрию -->
      <geometry>
      	<!-- Тэг <box> опиывает форму геометрии, параметр - размер (WxLxH) -->
        <box size="0.6 0.1 0.2"/>
        ...
```

> К сожалению, обработка файла валится на обработке ASCII символов. Так что комменты исключены из самого файла.

Как видите, формат файла базируется на XML формате. Тэги включая друг друга формируют описание, пока мы создали только коробочку, давайте взглянем на нее. Создадим файл запуска `my_robot_gazebo.launch`:
```xml
<?xml version="1.0"?>
<launch>
  <!-- Аргумент, которым мы можем задать начальное положение робота в мире -->
  <!-- Параметры (-x, -y и т.д.) заданы как опции для узла spawn_model -->
  <arg name="initial_pose" default="-x 0 -y 0 -z 0 -R 0 -P 0 -Y 0" />

  <!-- Аргумент, который содержит имя файла -->
  <arg name="urdf_file" default="$(find study_pkg)/descriptions/my_robot.urdf" />
  
  <!-- Настраиваем параметр `robot_description`, который будет нашим описанием робота -->
  <!-- Прогоняем через узел xacro, который произведет обработку файла, остальной функционал увидим позже -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(arg urdf_file)'" />
  
  <!-- Создаем робота узлом spawn_model -->
  <node name="spawn_turtlebot_model" pkg="gazebo_ros" type="spawn_model"
        args="$(arg initial_pose) -unpause -urdf -param robot_description -model my_robot" />

  <!-- Опубликуем TF статические преобразования на основе описания `robot_description` -->
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />

  <!-- Опубликуем TF динамические преобразования на основе описания `robot_description` -->
  <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" />

  <!-- Просто запустим rviz, после настройки добавим запуск конфигурации -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find study_pkg)/rviz/my_robot.rviz" />
</launch>
```

В launch-файле в конце мы запускаем rviz без конфигурации. Давайте запустим созданный launch-файл и настроим:  
- Fixed Frame установить в `base_link` ;  
- Подключить просмотр `Robot Model` и установить в отображении Alpha в значение 0,5;  
- Подключить просмотр `TF`;

После настройки сохраним конфигурацию rviz в файл `my_robot.rviz` и добавим `-d $(find study_pkg)/rviz/my_robot.rviz` к аргументам запуска rviz.

Далее запустим снова наш файл и видим представление нашей базы машинки:
```bash
roslaunch study_pkg my_robot_rviz.launch
```
<p align="center">
<img src="img1/T8_rviz_model_step1.png">
</p>

Так как для `spawn_model` мы задавали множество параметров, то давайте на всякий взглянем на Usage сообщение узла:
```bash
rosrun gazebo_ros spawn_model -h

Commands:
    -[urdf|sdf|trimesh|gazebo] - specify incoming xml is urdf, sdf or trimesh format. gazebo arg is deprecated in ROS Hydro
    -[file|param|database] [<file_name>|<param_name>|<model_name>] - source of the model xml or the trimesh file
    -model <model_name> - name of the model to be spawned.
    -reference_frame <entity_name> - optinal: name of the model/body where initial pose is defined.
                                     If left empty or specified as "world", gazebo world frame is used.
    -gazebo_namespace <gazebo ros_namespace> - optional: ROS namespace of gazebo offered ROS interfaces.  Defaults to /gazebo/ (e.g. /gazebo/spawn_model).
    -robot_namespace <robot ros_namespace> - optional: change ROS namespace of gazebo-plugins.
    -unpause - optional: !!!Experimental!!! unpause physics after spawning model
    -wait - optional: !!!Experimental!!! wait for model to exist
    -trimesh_mass <mass in kg> - required if -trimesh is used: linear mass
    -trimesh_ixx <moment of inertia in kg*m^2> - required if -trimesh is used: moment of inertia about x-axis
    -trimesh_iyy <moment of inertia in kg*m^2> - required if -trimesh is used: moment of inertia about y-axis
    -trimesh_izz <moment of inertia in kg*m^2> - required if -trimesh is used: moment of inertia about z-axis
    -trimesh_gravity <bool> - required if -trimesh is used: gravity turned on for this trimesh model
    -trimesh_material <material name as a string> - required if -trimesh is used: E.g. Gazebo/Blue
    -trimesh_name <link name as a string> - required if -trimesh is used: name of the link containing the trimesh
    -x <x in meters> - optional: initial pose, use 0 if left out
    -y <y in meters> - optional: initial pose, use 0 if left out
    -z <z in meters> - optional: initial pose, use 0 if left out
    -R <roll in radians> - optional: initial pose, use 0 if left out
    -P <pitch in radians> - optional: initial pose, use 0 if left out
    -Y <yaw in radians> - optional: initial pose, use 0 if left out
    -J <joint_name joint_position> - optional: initialize the specified joint at the specified value
    -package_to_model - optional: convert urdf <mesh filename="package://..." to <mesh filename="model://..."
    -b - optional: bond to gazebo and delete the model when this program is interrupted
```



