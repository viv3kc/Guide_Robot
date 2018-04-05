# NOTE: this example requires PyAudio because it uses the Microphone class

import speech_recognition as sr
import time
import atexit


class CommandListener(object):
    def __init__(self, landmarks, handle_command_callback):
        self.device_id = None
        self.working = True
        self.r = sr.Recognizer()
        self.GOOGLE_CLOUD_SPEECH_CREDENTIALS = r"""
            {
              "type": "service_account",
              "project_id": "penny-185111",
              "private_key_id": "ad98fe74e626441c562a4cd78152e1314fdd0b6f",
              "private_key": "-----BEGIN PRIVATE KEY-----\nMIIEvgIBADANBgkqhkiG9w0BAQEFAASCBKgwggSkAgEAAoIBAQCRwoFckCuNbS8p\n3AZqJI8nExb2QtDVIlmDlc/ZYyc0JLghirvRKvsdwHoJltqS5g+D0g3xjPFmFVoG\nh5z+y+QKsflgl8oyH1iHwegcEzbvfrZCvRNPN5y2/+vHs38Dcqq7J2/0TesLfvim\nM2+af6KbExbakp5R31A6GD1Grsi82Msk/b1yOUtEAnQKlIiHWBR/NbWaY8qEBh+B\nslsmB4FzJ2PmrcHPbsJMGHjCN8WBAGjSCOd+deLFfXwWLF0nwytoKsQned2rgAgS\n2QwBPZ2j4ILRAQBucJYB5XCFWjlMxZyIPQJctBC8XQNiRKL6S9tkMmGLtuqApv98\nXuSyRnV1AgMBAAECggEAG/kxvXN3wE0ng4eT/ZPlm1CpE/rvvC+68Stzt9s1H2c6\npUlMUVDH38ZcRXHbodygNqK+BVEQ1RAc9mrZYPb9XWYZn4WSCvuEEI3Qv26amOz/\nCqDhz06zd1V3SRy0MBO2G3Puovq/NLYZ/z+Y2BRNvygzNyBzXPAYJAYZ4HJT2RYR\n6yaUfcMBu+Kn1+auteYx4bjQEn1DZTwfUyCTi4aYOZ9TdDXAv2QjHc6amOsxc0JJ\nMbOIPNqJXPPrBFZO5W5WdHd2lytQWfR/vLCxOa5PU8DgiJeyyanOlpsbCqXaw1o2\n/60JTf++AKyZj8dHJ/bzqY/jw+y77Kw5O2K73Ll7owKBgQDLmiWiP/RXo3CrLZ0P\nrvR4n2UqpD21ZJB47qs35T63H1PJNBNuUbUsMBx382ad7ej0Fw6P/0pSMMueJIxs\n2ZeysLuJkohfBxMQ++ario+RMmbHhaL7Et3VmHsR6EicZih4FWv3x9G1xvuZnLWb\n9jBOHLsXlkQV/Qr4F8giMTmpHwKBgQC3RYy2fSvgONlF0GMz1YLOOJqpCc2/1I6k\n2W8Hu44/aIZ+Ce03xsbwNOH+MSPtDvCSLCP01YFkcZ6VFQs/19ku9VUDEGlWZbf3\nDiUr5G1IWni34CYciIhqu/TX/QIyWsicZ+f9mvV3re/DTOeT9hJMaUr+CvkEAvht\nSsK4DaMK6wKBgQCOAMQQxsRXv5GcgmLSBndSDfIUbJrxgeNVxTi78nLsa7gXU+/d\neLWAzUl9y9Cdg8p/O/mXKzod2xSZBuq+HIE2Sxdx2zar+JYuuVJoAGsjnOCcdjN3\nI/B+wivMEJNcKdA9+Mrl9YLpYpAm0mrEDVAhmt5OlCqGn4dgA849MtIBhwKBgQCl\nQQtOibOEbP5NW0f4Ny/Jb3TVs0dZD/rHXmAlYtMzDbcn8x95Sb2cXrLgl8LDQNNf\nmnmZjOMthN3vo5gPKzxkgUb0ilnRU8r79D9EQqbftORhOLZkyB/pzi1KjJLsgLuf\nfHnNTWrFTYuCWNf/rMR3LgI27AH5wgvr0JufhnxbnQKBgGCSIRHWM3pTZN3ZsGId\nSE8EFwV4Qx6R/YYcX+iLZmg77qoxJAPkTmATR1CNXdGIRRpAi3ZbHkS9GmnJJMv8\nN2IIkTf8zumXkkuB5tbljTh2DkS+Z9zn7RcoW3JRTQXfIs1zWW6h/IzNRxngIKlz\nuQ6se5ewe7lvgquk77ivDcTA\n-----END PRIVATE KEY-----\n",
              "client_email": "penny-597@penny-185111.iam.gserviceaccount.com",
              "client_id": "102142700637497379706",
              "auth_uri": "https://accounts.google.com/o/oauth2/auth",
              "token_uri": "https://accounts.google.com/o/oauth2/token",
              "auth_provider_x509_cert_url": "https://www.googleapis.com/oauth2/v1/certs",
              "client_x509_cert_url": "https://www.googleapis.com/robot/v1/metadata/x509/penny-597%40penny-185111.iam.gserviceaccount.com"
            }
        """

        self.handle_command_callback = handle_command_callback

        # self.preferred_phrases = [(landmark.name, 0.01) for landmark in landmarks]
        # self.preferred_phrases.extend([('cancel', 0.01), ('cancel command', 1.0), ('cancel destination', 0.01)])

        self.preferred_phrases = [landmark.name for landmark in landmarks]
        self.preferred_phrases.extend(['cancel', 'cancel command', 'cancel destination', 'tour', 'start tour'])

        mic_list = sr.Microphone.list_microphone_names()
        mic_name = 'ASTRA Pro: USB Audio (hw:2,0)'  # for astra camera microphone
        # mic_name = 'USB Device 0x46d:0x809: Audio (hw:2,0)'  # for usb camera microphone

        # the following loop aims to set the device ID of the mic that
        # we specifically want to use to avoid ambiguity.
        for i, microphone_name in enumerate(mic_list):
            # print(microphone_name)
            if microphone_name == mic_name:
                self.device_id = i
        self.microphone = sr.Microphone(self.device_id)

        self.stop_listening = None

    def __del__(self):
        print 'destroyed'
        if self.stop_listening:
            self.stop_listening()

    def listen_audio(self):
        with self.microphone as source:
            print '\nAdjusting for ambient noise. Please wait...'
            self.r.adjust_for_ambient_noise(source, duration=2)  # duration -> number of seconds it takes to adjust
            self.r.dynamic_energy_threshold = False
            # self.r.phrase_threshold = 0.4
            print '\nDone.'
        self.stop_listening = self.r.listen_in_background(source, self.handle_response, phrase_time_limit=10)
        print '\nListening...'
        atexit.register(self.stop_listening)

    def handle_response(self, recognizer, audio):
        # # recognize speech using Google Cloud Speech
        print 'Got audio'
        print self.r.energy_threshold
        try:
            text = self.r.recognize_google_cloud(
                audio,
                credentials_json=self.GOOGLE_CLOUD_SPEECH_CREDENTIALS,
                show_all=True,
                language='en-UK',
                preferred_phrases=self.preferred_phrases
            )

            # text = self.r.recognize_google(
            #     audio,
            #     language='en-UK'
            # )

            # print text
            s_to_t = ''
            if text:
                s_to_t = str(text['results'][0]['alternatives'][0]['transcript'])

            self.handle_command_callback(s_to_t)
            time.sleep(3)

        except sr.UnknownValueError:
            print("I could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Cloud Speech service; {0}".format(e))

        # # recognize speech using Google Speech Recognition
        # try:
        #     # for testing purposes, we're just using the default API key
        #     # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        #     # instead of `r.recognize_google(audio)`
        #     print("Google Speech Recognition thinks you said \n" + r.recognize_google(audio))
        # except sr.UnknownValueError:
        #     print("Google Speech Recognition could not understand audio")
        # except sr.RequestError as e:
        #     print("Could not request results from Google Speech Recognition service; {0}".format(e))


