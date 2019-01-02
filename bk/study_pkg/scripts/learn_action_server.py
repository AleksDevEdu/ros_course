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
			# Поставим флга, чтобы потом не выставить результат в "успешно"
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
