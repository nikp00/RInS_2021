import rospy
from google.cloud import texttospeech
import os
import sys

from task_1.srv import TextToSpeechService
from sound_play.msg import SoundRequest


class TextToSpeach:
    def __init__(self, base_path):
        self.node = rospy.init_node("text_to_speech")
        self.srv = rospy.Service("text_to_speech", TextToSpeechService, self.speak)

        self.voice_publisher = rospy.Publisher(
            "/robotsound", SoundRequest, queue_size=10
        )

        self.client = texttospeech.TextToSpeechClient()
        self.voice = texttospeech.VoiceSelectionParams(
            language_code="en-US", ssml_gender=texttospeech.SsmlVoiceGender.NEUTRAL
        )
        self.audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.OGG_OPUS
        )

        self.base_folder = base_path

        rospy.spin()

    def speak(self, req):
        self.synthesis_input = texttospeech.SynthesisInput(text=req.text)
        response = self.client.synthesize_speech(
            input=self.synthesis_input, voice=self.voice, audio_config=self.audio_config
        )
        with open(self.base_folder + "output.ogg", "wb") as out:
            out.write(response.audio_content)

        msg = SoundRequest()
        msg.sound = -2
        msg.command = 1
        msg.volume = 1
        msg.arg = self.base_folder + "output.ogg"

        self.voice_publisher.publish(msg)

        return 0


if __name__ == "__main__":
    base_path = sys.argv[1]
    auth = sys.argv[2]
    os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = auth
    TextToSpeach(base_path)
