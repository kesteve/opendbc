"""kepilot: ICBM cruise-button presses on alt-button CAN FD cars (CRUISE_BUTTONS_ALT, e.g. Kia Carnival 4th gen)."""
import unittest
from types import SimpleNamespace as NS

from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.values import HyundaiFlags, Buttons
from opendbc.sunnypilot.car.hyundai.icbm import IntelligentCruiseButtonManagementInterface, SendButtonState, ALT_BUTTON_COPIES

DBC = "hyundai_canfd_generated"
CAN = NS(ECAN=0, CAM=2)


def wheel_msg(packer, counter=74, **sig):
  """A wheel frame as carstate sees it: packed, then parsed back."""
  values = {"COUNTER": counter, "SET_ME_1": 1, "SET_ME_2": 5, "BYTE6": 90, "BYTE8": 45, **sig}
  _, dat, _ = packer.make_can_msg("CRUISE_BUTTONS_ALT", 0, values)
  cp = CANParser(DBC, [("CRUISE_BUTTONS_ALT", 0)], 0)
  cp.update([(1, [(0x1AA, bytes(dat), 0)])])
  return dict(cp.vl["CRUISE_BUTTONS_ALT"]), bytes(dat)


class TestAltButtonFrame(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker(DBC)

  def test_copy_of_the_wheel_frame_is_identical(self):
    msg, dat = wheel_msg(self.packer)
    addr, out, bus = hyundaicanfd.create_buttons_alt(self.packer, CAN, msg, int(msg["COUNTER"]), Buttons.NONE)
    self.assertEqual((addr, bytes(out), bus), (0x1AA, dat, 0))

  def test_press_changes_only_counter_button_and_crc(self):
    msg, dat = wheel_msg(self.packer)
    _, out, _ = hyundaicanfd.create_buttons_alt(self.packer, CAN, msg, 75, Buttons.SET_DECEL)
    self.assertEqual([i for i in range(16) if out[i] != dat[i]], [0, 1, 2, 4])
    self.assertEqual((out[4] >> 4) & 7, Buttons.SET_DECEL)
    self.assertEqual(out[2], 75)

  def test_never_presses_main_or_lda_buttons(self):
    msg, _ = wheel_msg(self.packer, ADAPTIVE_CRUISE_MAIN_BTN=1, LDA_BTN=1, NORMAL_CRUISE_MAIN_BTN=1)
    _, out, _ = hyundaicanfd.create_buttons_alt(self.packer, CAN, msg, 75, Buttons.RES_ACCEL)
    cp = CANParser(DBC, [("CRUISE_BUTTONS_ALT", 0)], 0)
    cp.update([(1, [(0x1AA, bytes(out), 0)])])
    v = cp.vl["CRUISE_BUTTONS_ALT"]
    self.assertEqual((v["ADAPTIVE_CRUISE_MAIN_BTN"], v["LDA_BTN"], v["NORMAL_CRUISE_MAIN_BTN"], v["CRUISE_BUTTONS"]),
                     (0, 0, 0, Buttons.RES_ACCEL))


class TestIcbmAltButtonSends(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker(DBC)
    CP = NS(flags=HyundaiFlags.CANFD | HyundaiFlags.CANFD_ALT_BUTTONS | HyundaiFlags.CANFD_RADAR_SCC,
            carFingerprint="KIA_CARNIVAL_4TH_GEN")
    self.icbm = IntelligentCruiseButtonManagementInterface(CP, NS())

  def _run(self, send, msg, seconds=1.0):
    CS = NS(buttons_counter=250, cruise_buttons_alt_msg=msg, is_metric=True)
    CC_SP = NS(intelligentCruiseButtonManagement=NS(sendButton=send))
    last, sent = 0, []
    for frame in range(int(seconds * 100)):
      out = self.icbm.update(CS, CC_SP, self.packer, frame, last, CAN)
      if out:
        last = frame
        sent.append((frame, out))
      if frame % 2 == 0:
        CS.buttons_counter = (CS.buttons_counter + 1) % 256
    return sent

  def test_presses_ahead_of_the_wheel_counter_on_the_pt_bus(self):
    msg, _ = wheel_msg(self.packer)
    sent = self._run(SendButtonState.decrease, msg)
    self.assertGreaterEqual(len(sent), 3)
    for _, out in sent:
      self.assertEqual(len(out), ALT_BUTTON_COPIES)
      for addr, dat, bus in out:
        self.assertEqual((addr, bus, (dat[4] >> 4) & 7), (0x1AA, 0, Buttons.SET_DECEL))
    # counters wrap at 256 (8-bit), not 15
    self.assertTrue(all(0 <= out[0][1][2] <= 255 for _, out in sent))

  def test_nothing_without_a_wheel_frame_or_a_request(self):
    self.assertEqual(self._run(SendButtonState.increase, {}), [])
    msg, _ = wheel_msg(self.packer)
    self.assertEqual(self._run(SendButtonState.none, msg), [])


if __name__ == "__main__":
  unittest.main()
