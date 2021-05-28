#!/usr/bin/python3

from task_3.srv import DialogService
import rospy


def test():
    print("runing")
    rospy.wait_for_service("dialog_service")
    try:
        dialog_handle = rospy.ServiceProxy("dialog_service", DialogService)
        response = dialog_handle(5)
        answers = {}
        answers["already vaccinated"] = response.already_vaccinated
        answers["doctor"] = response.doctor
        answers["hours of exercise"] = response.hours_of_exercise
        answers["wants vaccine"] =  response.wants_vaccine

    
        print(answers)
        
        return answers
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    test()