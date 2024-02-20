#
#
# This file uses the Mycroft Core Template
# and is part of the book "Artificial Intelligence for Robotics" by Francis Govers


from adapt.intent import IntentBuilder

from mycroft.skills.core import MycroftSkill
from mycroft.util.log import getLogger
from mycroft import intent_handler
import rospy
from std_msgs.msg import String

__author__ = 'fxgovers'

LOGGER = getLogger(__name__)
pub = rospy.Publisher('/syscommand', String, queue_size=1000)

def pubMessage(str):
	pub.publish(str)


class CleanRoomSkill(MycroftSkill):
    def __init__(self):
        super(CleanRoomSkill, self).__init__(name="CleanRoomSkill")

    def initialize(self):
        clean_room_intent = IntentBuilder("CleanRoomIntent"). \
            require("CleanRoomKeyword").build()
        self.register_intent(clean_room_intent, self.handle_clean_room_intent)



    def handle_clean_room_intent(self, message):
        self.speak_dialog("clean.up.room")
		pubMessage("PICK_UP_TOYS")

    def stop(self):
        pass


def create_skill():
    return CleanRoomSkill()
