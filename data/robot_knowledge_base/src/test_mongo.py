#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import mongodb_interface
import numpy as np

label = "test_interface" ## Label for writing and reading
d = ["{{field1: topo}, {field2: gigio}}", "{{field1: gatto}, {field2: silvestro}}"]
d2 = ["{{field1: topo}, {field2: gigio}}", "{{field1: gatto}, {field2: silvestro}}", "{{field1: cocco}, {field2: bill}}"]

data = np.array(d, dtype=np.str)
data2 = np.array(d2, dtype=np.str)

db = mongodb_interface.MongoDBInterface()

class TestDataOnFilePersistence(unittest.TestCase):

    def test_normal_flow(self):
        db.write_robot_sv(data, label)
        info = db.read_robot_sv(label)
        for i in xrange(len(data)):
            self.assertEqual(data[i], info[i], "Value read and wrote are different.")

    def test_different_data(self):
        db.write_robot_sv(data2, label)
        info = db.read_robot_sv(label)
        self.assertNotEqual(len(data), len(info), "Value read and wrote must be different.")

suite = unittest.TestLoader().loadTestsFromTestCase(TestDataOnFilePersistence)
unittest.TextTestRunner(verbosity=2).run(suite)
