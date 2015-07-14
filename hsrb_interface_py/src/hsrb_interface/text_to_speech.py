#!/usr/bin/env python
# vim: fileencoding=utf-8

import rospy

from . import robot
from . import settings
from . import exceptions

from tmc_msgs.msg import Voice



class TextToSpeech(robot.Resource):
    u"""音声合成サービス

    Attributes:
        language (int): 音声合成エンジンの言語モード

    """
    JAPANESE = Voice.kJapanese
    ENGLISH  = Voice.kEnglish

    def __init__(self, name):
        super(TextToSpeech, self).__init__()
        self._setting = settings.get_entry('text_to_speech', name)
        self._pub = rospy.Publisher(self._setting['topic'], Voice, queue_size=0)
        self._language = TextToSpeech.JAPANESE

    @property
    def language(self):
        return self._language

    @language.setter
    def language(self, value):
        if value not in (Voice.kJapanese, Voice.kEnglish):
            raise exceptions.InvalidLanguageError("Language code {0] is not supported}".format(value))
        self._language = value

    def say(self, text):
        u"""文字列を発声する

        Args:
            text (str): 音声に変換するテキスト(UTF-8)
        Returns:
            None
        """
        msg = Voice()
        msg.interrupting = False
        msg.queueing = False
        msg.language = self._language
        msg.sentence = text
        self._pub.publish(msg)
