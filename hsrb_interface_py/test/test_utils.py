#!/usr/bin/env python
# vim: fileencoding=utf-8

import math

from unittest import TestCase
from nose.tools import ok_, eq_
import mock

from hsrb_interface import utils


def test_singleton_meta():
    class Foo(object):
        __metaclass__ = utils.SingletonMeta

        def __init__(self, value):
            self.value = value

    # ２回め以降のコンストラクタ引数は無視される
    foo1 = Foo(1)
    foo2 = Foo(2)
    eq_(1, foo1.value)
    eq_(1, foo2.value)
    # 同一性チェック
    ok_(foo1 is foo2)



def test_iterate():
    xs = list(utils.iterate(lambda: 2,  5))
    eq_(len(xs), 5)
    for x in xs:
        eq_(2, x)
