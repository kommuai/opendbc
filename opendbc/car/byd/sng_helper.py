import cereal.messaging as messaging

SNG_INITIAL_PRESS_DELAY_FRAMES = 310
SNG_REPEAT_PRESS_DELAY_FRAMES = 110
SNG_MAX_RESUME_PRESSES = 3


class SngHelper:
  def __init__(self):
    self.plan_sm = messaging.SubMaster(['longitudinalPlan'])
    self.next_press_frame = 0
    self.resume_counter = 0
    self.is_sng_check = False

  @classmethod
  def create(cls, CP):
    return None if CP.openpilotLongitudinalControl else cls()

  @staticmethod
  def cruise_standstill(CS):
    return CS.out.cruiseState.standstill or CS.out.standstill

  @staticmethod
  def try_append_resume(sng, frame, CC, CS, resume_pressed, can_sends, resume_fn):
    if sng is None or frame % 2 != 0:
      return
    if sng.update_from_car(frame, CC, CS, resume_pressed):
      can_sends.append(resume_fn())

  def update_from_car(self, frame, CC, CS, resume_pressed):
    self.plan_sm.update(0)
    plan_allows_resume = self.plan_sm.valid['longitudinalPlan'] and (plan := self.plan_sm['longitudinalPlan']).aTarget > 0 and not plan.shouldStop

    if not (self.cruise_standstill(CS) and CC.enabled and CS.out.cruiseState.enabled and plan_allows_resume):
      self.is_sng_check = False
      return False

    if not self.is_sng_check:
      self.is_sng_check = True
      self.next_press_frame = frame + SNG_INITIAL_PRESS_DELAY_FRAMES
      self.resume_counter = 0
      return False

    if CS.out.gasPressed or resume_pressed or self.resume_counter >= SNG_MAX_RESUME_PRESSES:
      self.next_press_frame = max(self.next_press_frame, frame + SNG_REPEAT_PRESS_DELAY_FRAMES)
      self.resume_counter = 0
      return False

    if frame > self.next_press_frame and not CS.out.brakePressed:
      self.resume_counter += 1
      return True

    return False
