"""Unittest for collision_world module"""
from mock import patch


@patch('hsrb_interface.settings.get_entry')
def test_creation(mock_get_entry):
    """Test CollisionWorld class"""
    assert mock_get_entry
