# Copyright (C) 2016 Toyota Motor Corporation
"""Unittest for hsrb_interface.text_to_speech module"""
from unittest.mock import patch

import hsrb_interface
import hsrb_interface.exceptions
import hsrb_interface.text_to_speech
from nose.tools import eq_
from nose.tools import raises
import rclpy
from tmc_voice_msgs.msg import Voice


@patch('hsrb_interface.Robot._connecting')
@patch('hsrb_interface.settings.get_entry')
@patch('rclpy.node.Node.create_publisher')
def test_text_to_speech(mock_pub_class, mock_get_entry, mock_connecting):
    """Test simple usage of TTS object."""
    rclpy.init()
    robot = hsrb_interface.Robot()  # noqa: F841
    mock_connecting.return_value = True

    tts = hsrb_interface.text_to_speech.TextToSpeech('default_tts')

    mock_get_entry.return_value = {"topic": "foo"}
    mock_get_entry.called_with_args("text_to_speech", "default_tts")
    mock_pub_class.called_with_args("foo", Voice, queue_size=0)
    mock_pub_instance = mock_pub_class.return_value

    eq_(tts.language, tts.JAPANESE)
    tts.language = tts.ENGLISH
    eq_(tts.language, tts.ENGLISH)

    expected_msg = Voice()
    expected_msg.interrupting = False
    expected_msg.queueing = False
    expected_msg.language = False
    expected_msg.sentence = "Hello, World!"
    tts.say(u"Hello, World!")
    mock_pub_instance.called_with_args(expected_msg)


@raises(hsrb_interface.exceptions.InvalidLanguageError)
@patch('hsrb_interface.Robot._connecting')
@patch('hsrb_interface.settings.get_entry')
@patch('rclpy.node.Node.create_publisher')
def test_invalid_language_error(mock_pub_class, mock_get_entry,
                                mock_connecting):
    """TTS object should refuse invalid language."""
    mock_connecting.return_value = True

    tts = hsrb_interface.text_to_speech.TextToSpeech('default_tts')

    mock_get_entry.return_value = {"topic": "foo"}

    tts.language = -1
