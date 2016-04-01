from nose.tools import ok_, eq_, raises
from mock import patch, call

import hsrb_interface
import hsrb_interface.exceptions
import hsrb_interface.text_to_speech

from tmc_msgs.msg import Voice

@patch.object(hsrb_interface.Robot, '_connecting')
@patch('hsrb_interface.settings.get_entry')
@patch('rospy.Publisher')
def test_text_to_speech(mock_pub_class, mock_get_entry, mock_connecting):
    mock_connecting.return_value = True

    tts = hsrb_interface.text_to_speech.TextToSpeech('default')

    mock_get_entry.return_value = { "topic": "foo" }
    mock_get_entry.called_with_args("text_to_speech", "default")
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
@patch.object(hsrb_interface.Robot, '_connecting')
@patch('hsrb_interface.settings.get_entry')
@patch('rospy.Publisher')
def test_invalid_language_error(mock_pub_class, mock_get_entry, mock_connecting):
    mock_connecting.return_value = True

    tts = hsrb_interface.text_to_speech.TextToSpeech('default')

    mock_get_entry.return_value = { "topic": "foo" }

    tts.language = -1
