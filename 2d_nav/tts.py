import pyttsx


class TextToSpeech(object):

    @staticmethod
    def speak(speech):
        engine = pyttsx.init('espeak')
        engine.setProperty('rate', 140)
        engine.setProperty('voice', 'english')
        engine.say(speech)
        engine.runAndWait()
