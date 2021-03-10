# Actions

Сразу подкину вам ссыль на третий механизм взаимодействия узлов в ROS под названием [`actions`](http://wiki.ros.org/actionlib). Похож он на ранее рассмотренный механизм `services`, который организует синхронные команды через запросы. Как вы помните, мы посылаем запрос и ждем, пока он выполнется и вернется ответ. А ведь было бы неплохо, если бы мы могли отправить запрос и работать дальше, получая какие-нибудь уведомления, при этом не блокируясь, ожидая ответа. Есть у меня для вас такая штука =)

Давайте сразу начнем с создания своего `action` под названием `Learning.action`. Этот файл мы разместим в папке `action` в нашем пакете `study_pkg`:
```
# Define the goal
float32 target_accuracy
---
# Define the result
float32 result_accuracy
uint32  step_count
---
# Define a feedback message
float32 current_accuracy
uint32  step_number
```

Почему мы зацепили тему обучения? Да просто, это может быть одной из наиболее актуальных тем при асинхронной обработке информации.

Первый сектор `goal` - это то, что передается от `ActionClient` (тот, кто делает запрос) в качестве входной информации `ActionServer` (это тот, кто неустанно работает).

Второй сектор `result` - это информация, которая будет передана от `ActionServer` к `ActionClient` по окончанию обработки.

Третий сектор `feedback` - информация, которая с какой-то периодичностью будет передаваться от `ActionServer` к `ActionClient` в ходе работы.

> Топик - один сектор при определении, Сервисы - два сектора, Действия - три сектора, повышаем ставки =)

# Настраиваем сборку

Открываем `CMakeLists.txt` и добавляем:
```cmake
find_package(
    catkin REQUIRED COMPONENTS 
    ... 
    actionlib_msgs
    actionlib
    message_generation
)

add_action_files(
    FILES
    ...
    Learning.action
)

generate_messages( 
    DEPENDENCIES 
    ...
    actionlib_msgs
)
```

Открываем `package.xml` и добавляем:
```xml
	<build_depend>actionlib</build_depend>
	<build_depend>actionlib_msgs</build_depend>
	<exec_depend>actionlib</exec_depend>
	<exec_depend>actionlib_msgs</exec_depend>
```

В результате для нас создастся семь файлов, которыми мы воспользуемся в работе:
```
LearningAction.msg
LearningActionGoal.msg
LearningActionResult.msg
LearningActionFeedback.msg
LearningGoal.msg
LearningResult.msg
LearningFeedback.msg
```

На этом подготовка все, не забудьте пересобрать пакет командой `catkin_make` =)

# Пишем ActionServer на Python

Определим задачу нашего сервера, пускай он производит обучение некоторой неизвестной модели, каждый этап обучения будет занимать 0.5 секунды, на каждом этапе точность будет увеличиваться по экспоненциальной формуле `1 - 1/exp(step/10)`. По такой формуле обучение должно достигнуть 90 процентной точности на примерно 23 шаге обучения. Как мы ранее определили, к серверу приходит желаемая точность, сервер постепенно обучает модель и на каждом шаге отправляет нынешнюю точность и номер этапа. По окончании (превышении желаемой точности) сервер отправляет нынешнюю точность и количество затраченных шагов на это.

Так вот код сервера `learn_action_server.py`:
```python
#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
import actionlib
import numpy as np

from study_pkg.msg import LearningAction, LearningResult, LearningFeedback

action_server = None

# Определим функцию, которую будет выполнять наш сервер по приходу запроса с целью
def learn_execute(goal):
	step 		= 0
	accuracy 	= 0
	done 		= True

	while accuracy < goal.target_accuracy and step < 100:
		# Обработаем запрос на прерывание обработки
		if action_server.is_preempt_requested():
			# Установим состояние - были прерваны клиентом
			action_server.set_preempted()
			# Поставим флаг, чтобы потом не выставить результат в "успешно"
			done = False

			rospy.loginfo('Oops, client stopped us!')
			break

		time.sleep(0.5)
		step += 1
		accuracy = 1 - 1 / np.exp(step/10.)

		rospy.loginfo('Accuracy %.3f on step %d' % (accuracy, step))

		# Отправим LearningFeedback
		feedback = LearningFeedback()
		feedback.current_accuracy 	= accuracy
		feedback.step_number 		= step
		action_server.publish_feedback(feedback)

	if done:
		rospy.loginfo('Result accuracy %.3f on step %d' % (accuracy, step))

		# Отправим результат, если не прервали нас
		result = LearningResult()
		result.result_accuracy 	= accuracy
		result.step_count 		= step
		action_server.set_succeeded(result)

# Создадим объект сервера
action_server = actionlib.SimpleActionServer('model_learn', LearningAction, learn_execute, False)

if __name__ == '__main__':
	rospy.init_node('learn_server')

	# Запускаем сервер!
	action_server.start()
	rospy.spin()

```

# Пишем ActionClient на Python

У клиента задача одна, закинуть запрос, а дальше делать, что вздумается. Так он и поступит, предварительно настроив функции-обработчики, которые будут вызываться по мере работы сервера.

Вот код клиента `learn_action_client.py`:
```python
#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
import actionlib

from study_pkg.msg import LearningAction, LearningGoal

call_completed 	= False

# Создаем объект подключения к серверу (не перепутайте первый аргумент - название action)
client = actionlib.SimpleActionClient('model_learn', LearningAction)

# Обработчик - по завершению обработки
# По поводу первого аргумента - http://docs.ros.org/melodic/api/actionlib_msgs/html/msg/GoalStatus.html
def result_done_cb(state, result):
	global call_completed
	rospy.loginfo('Cb (%d): Result accuracy %.3f on step %d' % (state, result.result_accuracy, result.step_count))

	call_completed = True

# Обработчик - по переходу сервера к обработке цели
def server_active_cb():
	rospy.loginfo('Server active callback called')

# Обработчик - по приходу сообщений от сервера с обратной связью
def server_feedback_cb(feedback):
	rospy.loginfo('Feedback: Accuracy %.3f on step %d' % (feedback.current_accuracy, feedback.step_number))

	## Откройте этот блок, чтобы посмотреть, как работает прерывание задания
	# if feedback.step_number > 20:
	# 	rospy.loginfo('Feedback: Too many steps - cancel goal')
	# 	client.cancel_goal()
	# 	exit(1)

if __name__ == '__main__':
    rospy.init_node('learn_client')

    # Дождемся запуска сервера
    client.wait_for_server()

    # Заполним структуру цели
    goal = LearningGoal()
    goal.target_accuracy = 0.899

    # Засылаем гонца с установкой функций, которые будут вызваны по ходу работы
    client.send_goal(goal=goal, done_cb=result_done_cb, active_cb=server_active_cb, feedback_cb=server_feedback_cb)

    while not call_completed:
    	rospy.loginfo('I`m working! Don`t stop me =)')
    	time.sleep(2)

    # Получаем объект типа LearningResult
    result = client.get_result()

    rospy.loginfo('Main func: Result accuracy %.3f on step %d' % (result.result_accuracy, result.step_count))
```

# Выводы

На всякий закину [ссылку на доки](http://docs.ros.org/noetic/api/actionlib/html/namespaceactionlib.html), если потребуется раъяснить что-нибудь по функционалу. В остальном что сказать, вот и встретили мы асинхронный двунаправленный механизм для взаимодействия. К сожалению, функциональных утилит для него почти нет, а существующие не очень удобны. Но и с этим механизмом встречаются не так часто, как с топиками и сервисами. Так что в этой теме вы узнали, как с ним работать, а далее вы будете встречать разичные мануалы, которые будут давать достаточно подробные разъяснения, а на этой ноте позволю откланяться. Успехов!
