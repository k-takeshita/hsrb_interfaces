# Copyright (C) 2016 Toyota Motor Corporation
# vim: fileencoding=utf-8
"""Text-to-speech interface"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import rclpy
from tmc_voice_msgs.msg import Voice

from . import exceptions
from . import robot
from . import settings


class TextToSpeech(robot.Item):
    """Abstract interface for text-to-speech service

    Examples:

        .. sourcecode:: python

            with Robot() as robot:
                tts = robot.get("default", Items.TEXT_TO_SPEECH)
                tts.language = tts.JAPANESE
                tts.say(u"Hello, World!")
    """

    JAPANESE = Voice.JAPANESE
    ENGLISH = Voice.ENGLISH

    def __init__(self, name):
        """Initialize an instance

        Args:
            name (str): A resource name
        """
        super(TextToSpeech, self).__init__()
        self._setting = settings.get_entry('text_to_speech', name)
        topic = self._setting['topic']
        self._pub = self.create_publisher(Voice, topic,0)
        self._language = TextToSpeech.JAPANESE

    @property
    def language(self):
        """(int): Language of speech"""
        return self._language

    @language.setter
    def language(self, value):
        if value not in (Voice.JAPANESE, Voice.ENGLISH):
            msg = "Language code {0} is not supported".format(value)
            raise exceptions.InvalidLanguageError(msg)
        self._language = value

    def say(self, text):
        """Speak a given text

        Args:
            text (str): A text to be converted to voice sound (UTF-8)
        Returns:
            None
        """
        msg = Voice()
        msg.interrupting = False
        msg.queueing = False
        msg.language = self._language
        msg.sentence = text
        self._pub.publish(msg)
