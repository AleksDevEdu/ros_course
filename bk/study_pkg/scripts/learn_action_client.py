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
