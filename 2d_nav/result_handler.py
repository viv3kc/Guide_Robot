import time
from tts import TextToSpeech as tts


class ResultHandler(object):
    def __init__(self, main_node_ref):
        self.main_node = main_node_ref

    def handle_result(self, result):
        status = result.status.status
        if status == 3:
            self.handle_success()
        elif status == 4:
            self.handle_abort()

    def handle_success(self):
        self.main_node.is_moving = False
        if self.main_node.last_published_landmark != self.main_node.start_location:
            if not self.main_node.tour_started:
                tts.speak('Destination reached')
                time.sleep(5)
                tts.speak('Heading back to waiting position')
                self.main_node.publish_goal(self.main_node.start_location)
            else:
                tts.speak('Here you can find ' + self.main_node.last_published_landmark.name)
                time.sleep(5)
                self.main_node.publish_tour_landmark()
                tts.speak('Follow me, please!')
        else:
            tts.speak('Waiting position reached. Waiting for new input')
            if not self.main_node.listening_audio:
                self.main_node.command_listener.listen_audio()
                self.main_node.listening_audio = True

            self.main_node.last_published_landmark = None
            # self.main_node.pick_an_option()

    def handle_abort(self):
        position = self.main_node.last_estimated_pose.pose.pose.position
        landmark = self.main_node.last_published_landmark
        distance = (position.x - landmark.x) ** 2 + (position.y - landmark.y) ** 2
        print distance
        self.main_node.is_moving = False
        # print '\nPlan was aborted due to an error...'
        tts.speak('Plan was aborted due to an error...')
        time.sleep(5)
        # print '\nHeading back to the waiting position...'
        tts.speak('Heading back to waiting position')
        self.main_node.publish_goal(self.main_node.start_location)
