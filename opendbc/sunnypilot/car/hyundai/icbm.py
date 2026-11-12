"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.hyundai import hyundaican, hyundaicanfd
from opendbc.car.hyundai.values import HyundaiFlags, Buttons, CANFD_CAR
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

ButtonType = structs.CarState.ButtonEvent.Type
SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

BUTTON_COPIES = 2
ALT_BUTTON_COPIES = 5  # kepilot: CRUISE_BUTTONS_ALT frames per send (16 bytes each on CAN FD; a real press is 5 frames)
BUTTON_COPIES_TIME = 7
BUTTON_COPIES_TIME_IMPERIAL = [BUTTON_COPIES_TIME + 3, 70]
BUTTON_COPIES_TIME_METRIC = [BUTTON_COPIES_TIME, 40]

BUTTONS = {
  SendButtonState.increase: Buttons.RES_ACCEL,
  SendButtonState.decrease: Buttons.SET_DECEL,
}


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

  def create_can_mock_button_messages(self, packer, CS, send_button) -> list[CanData]:
    can_sends = []
    copies_xp = BUTTON_COPIES_TIME_METRIC if CS.is_metric else BUTTON_COPIES_TIME_IMPERIAL
    copies = int(np.interp(BUTTON_COPIES_TIME, copies_xp, [1, BUTTON_COPIES]))

    # send resume at a max freq of 10Hz
    if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
      # send 25 messages at a time to increases the likelihood of resume being accepted
      can_sends.extend([hyundaican.create_clu11(packer, self.frame, CS.clu11, send_button, self.CP)] * copies)
      if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
        self.last_button_frame = self.frame

    return can_sends

  def create_canfd_mock_button_messages(self, packer, CS, CAN, send_button) -> list[CanData]:
    can_sends = []
    if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
      # kepilot: the wheel sends CRUISE_BUTTONS_ALT at 50 Hz with an 8-bit counter on the bus the radar (SCC) listens
      # on, and we can't block it: the same timing as the CRUISE_BUTTONS presses below (frames ahead of the wheel's
      # next counters), from a copy of its latest frame
      if not CS.cruise_buttons_alt_msg:
        return can_sends
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.2:
        self.button_frame += 1
        button_counter_offset = [1, 1, 0, None][self.button_frame % 4]
        if button_counter_offset is not None:
          cnt = (int(CS.buttons_counter) + button_counter_offset) % 0x100
          for _ in range(ALT_BUTTON_COPIES):
            can_sends.append(hyundaicanfd.create_buttons_alt(packer, CAN, CS.cruise_buttons_alt_msg, cnt, send_button))
          self.last_button_frame = self.frame
    else:
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.2:
        self.button_frame += 1
        button_counter_offset = [1, 1, 0, None][self.button_frame % 4]
        if button_counter_offset is not None:
          for _ in range(20):
            can_sends.append(hyundaicanfd.create_buttons(packer, self.CP, CAN, (CS.buttons_counter + button_counter_offset) % 0xF, send_button))
          self.last_button_frame = self.frame

    return can_sends

  def update(self, CS, CC_SP, packer, frame, last_button_frame, CAN) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    if self.ICBM.sendButton != SendButtonState.none:
      send_button = BUTTONS[self.ICBM.sendButton]

      if self.CP.carFingerprint in CANFD_CAR:
        can_sends.extend(self.create_canfd_mock_button_messages(packer, CS, CAN, send_button))
      else:
        can_sends.extend(self.create_can_mock_button_messages(packer, CS, send_button))

    return can_sends
