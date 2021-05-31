"""
sudo apt install python3-pyaudio
pip3 install SpeechRecognition
"""
import rospy
import speech_recognition as sr

from task_3.srv import DialogService, DialogServiceResponse
from task_3.srv import TextToSpeechService, TextToSpeechServiceResponse


class SpeechTranscriber:
    def __init__(self):
        self.sr = sr.Recognizer()
        self.mic = sr.Microphone()

        self.affirmative_answers = ["yes", "i have", "i do"]
        self.negative_answers = ["no", "not", "i don't"]
        self.doctors = ["green", "red", "blue", "black", "yellow"]

        self.sr.dynamic_energy_threshold = False

        print("----")
        print("Waiting for text to speech service...")
        rospy.wait_for_service("text_to_speech")
        self.speak = rospy.ServiceProxy("text_to_speech", TextToSpeechService)
        print("Connected")

    # prilagodi mikrofon glede na hrup
    def adjustMicrophone(self):
        with self.mic as source:
            print("Adjusting mic for ambient noise...")
            self.sr.adjust_for_ambient_noise(source)
        self.sr.dynamic_energy_threshold = False

    # izpise prompt message in prepozna odgovor preko mikrofona
    def promptAndListen(self, prompt_message):
        self.speak(prompt_message)
        print(prompt_message)
        with self.mic as source:
            audio = self.sr.listen(source)
            # audio = self.sr.listen(source, timeout = 4)

        print("Processing...")
        recognized_text = ""
        try:
            recognized_text = self.sr.recognize_google(audio)
            # recognized_text = self.sr.recognize_bing(audio)
        except sr.RequestError as e:
            print("API is probably unavailable", e)
            return -1
        except sr.UnknownValueError:
            print("Did not manage to recognize anything.")
            return -1

        print("Recognized: ", recognized_text)

        return recognized_text.lower()

    # vpraša uporabnika dano vprašanje in čaka na odgovor, če odgovor vsebuje besedo iz seznama legalnih odgovorov vrne odgovor, čene vpraša ponovno
    # če ptevilo neuspelih poskusov preseže max_tries prosi za vnos preko konzole namesto prepoznave govora
    def askUser(self, question, legal_answers, max_tries=3):
        answer = ""
        i = 0
        while True:
            i += 1
            if i <= max_tries:
                answer = self.promptAndListen(question)
                if answer == -1:
                    print("Sorry, please try again")
                    self.speak("Sorry, please try again")
                    continue
            else:
                self.speak("Please provide an answear manually: ")
                answer = input("Please provide an answear manually: ")

            if legal_answers == "positive integer":
                # check if answer contains a number
                if any(char.isdigit() for char in answer):
                    # extract number from answer
                    number = int("".join(filter(str.isdigit, answer)))
                    if number > 0:
                        return number
                else:
                    print("Please provide a positive integer number")
                    self.speak("Please provide a positive integer number")

            # elif any (element in answer for element in legal_answers):
            else:
                for element in legal_answers:
                    if element in answer:
                        # print("Detected match: ", element)
                        return answer

            self.speak("Sorry, please try again")
            print("Sorry, please try again")

    def say(self, s):
        print(s)
        response = self.speak(s)
        rospy.sleep(response.response)

    # glavna funkcija
    def dialog(self):
        answers = {}

        print("-------NEW REQUEST-------")

        self.adjustMicrophone()

        self.speak("Hello")
        print("Hello")

        already_vaccinated_answer = self.askUser(
            "Have you been vaccinated?", self.affirmative_answers + self.negative_answers
        )
        if any(element in already_vaccinated_answer for element in self.affirmative_answers):
            answers["already vaccinated"] = True
        else:
            answers["already vaccinated"] = False

        doctor_answer = self.askUser("Who is your doctor?", self.doctors)
        for doc in self.doctors:
            if doc in doctor_answer:
                answers["doctor"] = doc
                break

        excercise_answer = self.askUser(
            "How many hours per week do you exercise?", "positive integer"
        )
        answers["hours of exercise"] = excercise_answer

        wants_vaccine_answer = self.askUser(
            "Do you want to be vaccinated?", self.affirmative_answers + self.negative_answers
        )
        if any(element in wants_vaccine_answer for element in self.affirmative_answers):
            answers["wants vaccine"] = True
        else:
            answers["wants vaccine"] = False

        self.speak("Thank you, that is it.")

        print("-------DONE-------")

        return answers

        """
        format vrnjenega odgovora:
        answers{
            "already vaccinated": True/False,
            "doctor": barva zdravnika iz self.doctors[],
            "hours of exercise": number,
            "wants vaccine": True/False
        }

        """


class Service:
    def __init__(self):
        self.st = SpeechTranscriber()
        rospy.init_node("dialog_server")
        self.srv = rospy.Service("dialog_service", DialogService, self.callBack)
        print("Running...")
        rospy.spin()

    def callBack(self, request):
        answers = self.st.dialog()
        return DialogServiceResponse(
            answers["already vaccinated"],
            answers["doctor"],
            int(answers["hours of exercise"]),
            answers["wants vaccine"],
        )


if __name__ == "__main__":
    Service()
