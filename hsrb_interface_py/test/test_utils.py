#!/usr/bin/env python
# vim: fileencoding=utf-8
"""Unittest for hsrb_interface.utils module."""

from nose.tools import eq_

from hsrb_interface import utils


def test_iterate():
    """Test iterate function."""
    data = list(utils.iterate(lambda: 2, 5))
    eq_(len(data), 5)
    for datum in data:
        eq_(2, datum)
