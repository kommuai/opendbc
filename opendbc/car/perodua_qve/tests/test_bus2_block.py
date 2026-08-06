#!/usr/bin/env python3
import unittest

from opendbc.car.perodua_qve.values import (
  BUS2_ALSO_FORWARD,
  BUS2_ALWAYS_FORWARD,
  BUS2_POOL,
  qve_bus2_split_addrs,
)


class TestQveBus2Block(unittest.TestCase):
  def test_block_all_except_0x50_and_0x80(self):
    blocked, forwarded = qve_bus2_split_addrs()
    self.assertEqual(blocked, [a for a in BUS2_POOL if a != BUS2_ALSO_FORWARD])
    self.assertEqual(forwarded, [BUS2_ALSO_FORWARD])
    self.assertNotIn(BUS2_ALWAYS_FORWARD, BUS2_POOL)
    self.assertIn(BUS2_ALSO_FORWARD, BUS2_POOL)


if __name__ == "__main__":
  unittest.main()
