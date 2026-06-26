import cereal.messaging as messaging

try:
  from openpilot.common.swaglog import cloudlog
except ImportError:
  cloudlog = None

SNG_INITIAL_PRESS_DELAY_FRAMES = 310
SNG_REPEAT_PRESS_DELAY_FRAMES = 110
SNG_MAX_RESUME_PRESSES = 3
SNG_WAIT_LOG_INTERVAL_FRAMES = 200  # ~2s at 100Hz controller frame counter


class SngHelper:
  def __init__(self):
    self.plan_sm = messaging.SubMaster(['longitudinalPlan'])
    self.next_press_frame = 0
    self.resume_counter = 0
    self.is_sng_check = False
    self._prev_plan_allows = False
    self._brake_block_logged = False

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
      can_msg = resume_fn()
      can_sends.append(can_msg)
      sng._log_press(frame, can_msg)

  def _log(self, event: str, detail: str):
    if cloudlog is not None:
      cloudlog.warning(f"BYD_SNG {event} {detail}")

  def _plan_fields(self):
    if not self.plan_sm.valid['longitudinalPlan']:
      return 0.0, True, False, False, -1
    plan = self.plan_sm['longitudinalPlan']
    return (
      float(plan.aTarget),
      bool(plan.shouldStop),
      True,
      bool(plan.hasLead),
      int(plan.longitudinalPlanSource),
    )

  def _snapshot(self, frame, CC, CS, resume_pressed, plan_allows_resume):
    a_target, should_stop, plan_valid, has_lead, plan_src = self._plan_fields()
    set_btn = int(getattr(CS, 'set_btn', -1))
    acc_btn = int(getattr(CS, 'btn_acc_set_reset', -1))
    cruise_hud = int(getattr(CS, 'cruise_hud_state', getattr(CS, 'acc_state', -1)))
    return (
      f"f={frame} act={int(self.is_sng_check)} ego={int(CS.out.standstill)} "
      f"cstk={int(CS.out.cruiseState.standstill)} v={CS.out.vEgoRaw:.2f} "
      f"cruise={int(CS.out.cruiseState.enabled)} hud={cruise_hud} en={int(CC.enabled)} "
      f"plan_ok={int(plan_allows_resume)} pvalid={int(plan_valid)} aT={a_target:.2f} "
      f"stop={int(should_stop)} lead={int(has_lead)} psrc={plan_src} "
      f"gas={int(CS.out.gasPressed)} brk={int(CS.out.brakePressed)} "
      f"rx_res={int(resume_pressed)} set={set_btn} acc_btn={acc_btn} "
      f"cnt={self.resume_counter} next={self.next_press_frame}"
    )

  def _plan_block_reason(self):
    if not self.plan_sm.valid['longitudinalPlan']:
      return 'plan_invalid'
    plan = self.plan_sm['longitudinalPlan']
    if plan.aTarget <= 0:
      return 'plan_aT0'
    if plan.shouldStop:
      return 'plan_shouldStop'
    return 'plan_unknown'

  def _disarm_reason(self, CS, CC, plan_allows_resume):
    if not self.cruise_standstill(CS):
      return 'no_standstill'
    if not CC.enabled:
      return 'not_enabled'
    if not CS.out.cruiseState.enabled:
      return 'cruise_off'
    if not plan_allows_resume:
      return self._plan_block_reason()
    return 'unknown'

  def _log_press(self, frame, can_msg):
    addr, dat, bus = can_msg
    a_target, should_stop, plan_valid, has_lead, plan_src = self._plan_fields()
    self._log(
      'PRESS',
      f"f={frame} n={self.resume_counter} bus={bus} addr=0x{addr:x} dat={bytes(dat).hex()} "
      f"aT={a_target:.2f} stop={int(should_stop)} lead={int(has_lead)} psrc={plan_src} "
      f"pvalid={int(plan_valid)}",
    )

  def update_from_car(self, frame, CC, CS, resume_pressed):
    self.plan_sm.update(0)
    plan_allows_resume = self.plan_sm.valid['longitudinalPlan'] and (plan := self.plan_sm['longitudinalPlan']).aTarget > 0 and not plan.shouldStop

    if not (self.cruise_standstill(CS) and CC.enabled and CS.out.cruiseState.enabled and plan_allows_resume):
      if self.is_sng_check:
        self._log('DISARM', f"{self._disarm_reason(CS, CC, plan_allows_resume)} {self._snapshot(frame, CC, CS, resume_pressed, plan_allows_resume)}")
      self.is_sng_check = False
      self._brake_block_logged = False
      self._prev_plan_allows = plan_allows_resume
      return False

    if plan_allows_resume and not self._prev_plan_allows and self.cruise_standstill(CS) and CC.enabled and CS.out.cruiseState.enabled:
      self._log('PLAN_GO', self._snapshot(frame, CC, CS, resume_pressed, plan_allows_resume))

    if not self.is_sng_check:
      self.is_sng_check = True
      self.next_press_frame = frame + SNG_INITIAL_PRESS_DELAY_FRAMES
      self.resume_counter = 0
      self._brake_block_logged = False
      self._log('ARM', self._snapshot(frame, CC, CS, resume_pressed, plan_allows_resume))
      self._prev_plan_allows = plan_allows_resume
      return False

    if self.is_sng_check and frame % SNG_WAIT_LOG_INTERVAL_FRAMES == 0 and frame <= self.next_press_frame:
      self._log('WAIT', self._snapshot(frame, CC, CS, resume_pressed, plan_allows_resume))

    if CS.out.gasPressed or resume_pressed or self.resume_counter >= SNG_MAX_RESUME_PRESSES:
      reason = 'gas' if CS.out.gasPressed else 'manual_res' if resume_pressed else 'max_press'
      self._log('COOLDOWN', f"{reason} {self._snapshot(frame, CC, CS, resume_pressed, plan_allows_resume)}")
      self.next_press_frame = max(self.next_press_frame, frame + SNG_REPEAT_PRESS_DELAY_FRAMES)
      self.resume_counter = 0
      self._brake_block_logged = False
      self._prev_plan_allows = plan_allows_resume
      return False

    self._prev_plan_allows = plan_allows_resume

    if frame > self.next_press_frame and CS.out.brakePressed:
      if not self._brake_block_logged:
        self._log('BLOCK_BRAKE', self._snapshot(frame, CC, CS, resume_pressed, plan_allows_resume))
        self._brake_block_logged = True
      return False

    if frame > self.next_press_frame:
      self.resume_counter += 1
      return True

    return False
