import numpy as np
from time import monotonic
from datetime import datetime
from collections import deque
import os
import hashlib

from opendbc.can.packer import CANPacker
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_dist_to_meas_limits
from opendbc.car.proton.protoncan import _acc_cmd_values, create_acc_cmd, create_can_steer_command, send_buttons
from opendbc.car.proton.values import DBC, CAR

try:
  from openpilot.common.features import Features
except (ImportError, ModuleNotFoundError):
  class Features:
    def has(self, _):
      return False


def _clip(value, lo, hi):
  return float(np.clip(value, lo, hi))


def apply_proton_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, LIMITS):
  # Proton-specific driver torque envelope.
  driver_offset = driver_torque * 30
  max_steer_allowed = _clip(LIMITS.STEER_MAX + driver_offset, 0, LIMITS.STEER_MAX)
  min_steer_allowed = _clip(-LIMITS.STEER_MAX + driver_offset, -LIMITS.STEER_MAX, 0)
  apply_torque = _clip(apply_torque, min_steer_allowed, max_steer_allowed)

  # Delegate common ramp limiting to shared helper while keeping Proton-specific driver bounds.
  apply_torque = apply_dist_to_meas_limits(
    apply_torque,
    apply_torque_last,
    0.0,
    LIMITS.STEER_DELTA_UP,
    LIMITS.STEER_DELTA_DOWN,
    LIMITS.STEER_MAX,
    LIMITS.STEER_MAX,
  )

  return round(apply_torque)


class CarControllerParams:
  STEER_STEP = 1

  def __init__(self, CP):
    self.STEER_MAX = CP.lateralParams.torqueV[0]
    assert len(CP.lateralParams.torqueV) == 1

    if CP.carFingerprint == CAR.PROTON_X90:
      self.STEER_DELTA_UP = 4
      self.STEER_DELTA_DOWN = 8
    else:
      self.STEER_DELTA_UP = 15
      self.STEER_DELTA_DOWN = 35

class CarController(CarControllerBase):
  _shared_proton_acc_log_prefix = None
  _shared_proton_accel_log_header_written = False

  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])
    self.params = CarControllerParams(self.CP)

    self.last_steer = 0
    features = Features()
    self.always_lks_tactile = features.has("lks-tactile")
    self.openpilot_long = not features.has("stock-acc")

    self.prev_steer_enabled = False
    self.last_steer_disable = 0.0

    self.sng_next_press_frame = 0 # The frame where the next resume press is allowed
    self.resume_counter = 0       # Counter for tracking the progress of a resume press
    self.is_sng_check = False
    self.resume = False

    self.cancel_press_cnt = 0
    self.last_cancel_press = 0

    # Proton ACC debug logging (stock vs sim):
    # - buffer frame snapshots in RAM
    # - flush to /data txt files at most once per second
    # - write pre/post transition windows once transitions happen
    # Output: one file per plot/event type, but with the same debug prefix.
    if CarController._shared_proton_acc_log_prefix is None:
      ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
      fingerprint = str(CP.carFingerprint).replace(" ", "_")
      # Stable randnum derived from fingerprint (no PID, no Python hash()).
      fp_md5 = hashlib.md5(fingerprint.encode("utf-8")).hexdigest()
      randnum = int(fp_md5[-4:], 16) % 10000
      CarController._shared_proton_acc_log_prefix = f"/data/debug_{ts}_{randnum}"

    self._proton_acc_log_prefix = CarController._shared_proton_acc_log_prefix

    # pending lines split by plot key, so each event type can go to its own file.
    self._proton_acc_log_pending_lines_by_plot: dict[str, list[str]] = {}
    self._proton_acc_log_header_written_by_plot: set[str] = set()
    self._proton_acc_log_last_flush_t = 0.0
    # Keep enough history for up to 300-frame captures (300 before + current).
    self._proton_acc_log_recent = deque(maxlen=305)
    self._proton_acc_log_events: list[dict] = []
    self._proton_acc_log_prev_enabled = False
    self._proton_acc_log_prev_standstill = False
    self._proton_acc_log_prev_res = False

    # Ensure we don't let log line buffers grow forever if events never finish.
    self._proton_acc_log_max_pending = 50000

    # Proton ACC accel logger (per-frame stock vs actuators):
    # - only when CC.longActive == True and CS.out.standstill == False
    # - flush at most once per second
    # - do not exceed 100MB total file size; then stop logging
    # - queued lines live only in memory; sudden stop => queued lines dropped silently
    self._proton_accel_log_path = f"{self._proton_acc_log_prefix}_accel.txt"
    self._proton_accel_log_pending_lines: list[str] = []
    self._proton_accel_log_last_flush_t = 0.0
    self._proton_accel_log_max_bytes = 100 * 1024 * 1024
    self._proton_accel_log_disabled = False

  def _proton_accel_log_maybe_write(self):
    if self._proton_accel_log_disabled:
      return
    if not self._proton_accel_log_pending_lines:
      return

    now_t = monotonic()
    if (now_t - self._proton_accel_log_last_flush_t) < 1.0:
      return

    # If file already too large, stop writing.
    try:
      if os.path.exists(self._proton_accel_log_path) and os.path.getsize(self._proton_accel_log_path) >= self._proton_accel_log_max_bytes:
        self._proton_accel_log_disabled = True
        self._proton_accel_log_pending_lines.clear()
        return
    except OSError:
      self._proton_accel_log_disabled = True
      self._proton_accel_log_pending_lines.clear()
      return

    pending_bytes = sum(len(line) + 1 for line in self._proton_accel_log_pending_lines)
    try:
      existing_size = os.path.getsize(self._proton_accel_log_path) if os.path.exists(self._proton_accel_log_path) else 0
      if existing_size + pending_bytes > self._proton_accel_log_max_bytes:
        self._proton_accel_log_disabled = True
        self._proton_accel_log_pending_lines.clear()
        return
    except OSError:
      self._proton_accel_log_disabled = True
      self._proton_accel_log_pending_lines.clear()
      return

    write_header = not CarController._shared_proton_accel_log_header_written
    try:
      if os.path.exists(self._proton_accel_log_path) and os.path.getsize(self._proton_accel_log_path) > 0:
        write_header = False
        CarController._shared_proton_accel_log_header_written = True
    except OSError:
      pass

    try:
      mode = "w" if write_header else "a"
      with open(self._proton_accel_log_path, mode, encoding="utf-8") as f:
        if write_header:
          f.write("Proton ACC accel debug log (stock vs actuators)\n")
          f.write("fields: frame enabled brakePressed gasPressed resPressed outStandstill resume stockCMD actuatorsAccel actuatorsOutAccelScaled accelMultApplied accelCmdFromAccelMultScaled\n")
          CarController._shared_proton_accel_log_header_written = True
        for line in self._proton_accel_log_pending_lines:
          f.write(line + "\n")
    except OSError:
      # Stop logging on persistent failure.
      self._proton_accel_log_disabled = True
      self._proton_accel_log_pending_lines.clear()
      return

    self._proton_accel_log_pending_lines.clear()
    self._proton_accel_log_last_flush_t = now_t

  def _proton_accel_log_update(self, CC, CS, actuators_accel: float):
    if self._proton_accel_log_disabled:
      return
    if not CC.longActive:
      return
    if bool(CS.out.standstill):
      return

    enabled = bool(CS.out.cruiseState.enabled)
    brake_pressed = bool(CS.out.brakePressed)
    gas_pressed = bool(CS.out.gasPressed)
    res_pressed = bool(CS.res_btn_pressed)
    out_standstill = bool(CS.out.standstill)  # should be False due to gating
    resume_state = bool(self.resume)

    stock_cmd = None
    if CS.stock_acc_cmd_values:
      stock_cmd = CS.stock_acc_cmd_values.get("CMD", CS.stock_acc_cmd)
    else:
      stock_cmd = CS.stock_acc_cmd

    ac = float(actuators_accel)
    # Tuned from /data/*_accel.txt logs (stock CMD vs actuators accel),
    # using separate multipliers for accel vs decel.
    accel_mult_applied = 19 if ac >= 0.0 else 21
    accel_cmd_from_acc_mult = _clip(ac * accel_mult_applied, -95.0, 95.0)

    # This matches `new_actuators.accel = accel_cmd/15 if >=0 else /18` below.
    actuators_out_accel_scaled = (ac / 15.0) if ac >= 0.0 else (ac / 18.0)

    line = (
      f"frame={self.frame} enabled={int(enabled)} brakePressed={int(brake_pressed)} "
      f"gasPressed={int(gas_pressed)} resPressed={int(res_pressed)} outStandstill={int(out_standstill)} "
      f"resume={int(resume_state)} stockCMD={stock_cmd} actuatorsAccel={ac} "
      f"actuatorsOutAccelScaled={actuators_out_accel_scaled} accelMultApplied={accel_mult_applied} "
      f"accelCmdFromAccelMultScaled={accel_cmd_from_acc_mult}"
    )

    self._proton_accel_log_pending_lines.append(line)
    self._proton_accel_log_maybe_write()

  def _proton_acc_log_snapshot(self, CC, CS, accel_cmd_scaled, standstill_request):
    # Use same "enabled" source as control code.
    enabled = bool(CS.out.cruiseState.enabled)
    standstill = bool(CS.cruise_standstill)
    out_standstill = bool(CS.out.standstill)
    brake_pressed = bool(CS.out.brakePressed)
    gas_pressed = bool(CS.out.gasPressed)
    res_pressed = bool(CS.res_btn_pressed)
    resume_state = bool(self.resume)

    stock_cmd = dict(CS.stock_acc_cmd_values) if CS.stock_acc_cmd_values else {}

    gas_override = gas_pressed and enabled
    sim_cmd = {}
    if self.openpilot_long:
      sim_cmd = _acc_cmd_values(
        accel_cmd_scaled,
        CC.longActive,
        gas_override,
        standstill,
        self.resume,
        brake_pressed,
        standstill_request,
      )

    return {
      "frame": self.frame,
      "enabled": enabled,
      "standstill": standstill,
      "CSoutStandstill": out_standstill,
      "brakePressed": brake_pressed,
      "gasPressed": gas_pressed,
      "resButtonPressed": res_pressed,
      "resume": resume_state,
      "stockAccCmd": stock_cmd,
      "simAccCmd": sim_cmd,
    }

  def _proton_acc_log_maybe_write(self):
    if not self._proton_acc_log_pending_lines_by_plot:
      return
    if not any(self._proton_acc_log_pending_lines_by_plot.values()):
      return

    now_t = monotonic()
    if (now_t - self._proton_acc_log_last_flush_t) < 1.0:
      return

    # Flush buffered lines (<=1 flush per second), writing each plot's pending
    # buffer to its own file.
    plots = list(self._proton_acc_log_pending_lines_by_plot.keys())
    for plot in plots:
      pending = self._proton_acc_log_pending_lines_by_plot.get(plot)
      if not pending:
        continue

      path = f"{self._proton_acc_log_prefix}_{plot}.txt"
      write_header = plot not in self._proton_acc_log_header_written_by_plot
      if write_header:
        try:
          if os.path.exists(path) and os.path.getsize(path) > 0:
            write_header = False
            self._proton_acc_log_header_written_by_plot.add(plot)
        except OSError:
          pass

      try:
        mode = "w" if write_header else "a"
        with open(path, mode, encoding="utf-8") as f:
          if write_header:
            f.write("Proton ACC debug log (stock vs sim)\n")
            f.write(f"plot={plot}\n")
            f.write(f"created_at={datetime.now().isoformat()}\n")
            f.write("fields: frame, enabled, standstill, CSoutStandstill, brakePressed, gasPressed, resButtonPressed, resume, stockAccCmd, simAccCmd\n")
            self._proton_acc_log_header_written_by_plot.add(plot)
          for line in pending:
            f.write(line + "\n")
      except OSError:
        # If a write fails, discard pending lines for that plot to avoid repeated crashes.
        self._proton_acc_log_pending_lines_by_plot[plot] = []
        continue

      self._proton_acc_log_pending_lines_by_plot[plot] = []

    self._proton_acc_log_last_flush_t = now_t

  def _proton_acc_log_add_event_frames(self, event: dict, snapshots: list[dict]):
    # Add snapshot lines for frames in [start_frame, end_frame].
    start_frame = event["start_frame"]
    end_frame = event["end_frame"]
    reason = event["reason"]
    logged = event["logged_frames"]
    plot = event.get("plot_tag", "unknown")
    only_enabled_frames = bool(event.get("only_enabled_frames", False))
    pending = self._proton_acc_log_pending_lines_by_plot.setdefault(plot, [])
    for snap in snapshots:
      fr = snap["frame"]
      if start_frame <= fr <= end_frame and fr not in logged:
        if only_enabled_frames and not snap.get("enabled", False):
          continue
        logged.add(fr)
        # Keep one line per frame; dicts are compact enough for troubleshooting.
        pending.append(
          f"reason={reason} frame={fr} enabled={int(snap['enabled'])} standstill={int(snap['standstill'])} CSoutStandstill={int(snap['CSoutStandstill'])} "
          f"brakePressed={int(snap['brakePressed'])} gasPressed={int(snap['gasPressed'])} "
          f"resButtonPressed={int(snap['resButtonPressed'])} resume={int(snap['resume'])} stockAccCmd={snap['stockAccCmd']} simAccCmd={snap['simAccCmd']}"
        )
        if len(pending) >= self._proton_acc_log_max_pending:
          # Prevent RAM blow-up: force flush if pending is too large.
          self._proton_acc_log_maybe_write()

  def _proton_acc_log_maybe_create_events(self, CC, CS):
    enabled = bool(CS.out.cruiseState.enabled)
    standstill = bool(CS.cruise_standstill)
    res_pressed = bool(CS.res_btn_pressed)

    new_events: list[dict] = []

    # Event 1: not enabled -> enabled
    if (not self._proton_acc_log_prev_enabled) and enabled:
      start = self.frame - 10
      end = self.frame + 10
      start = max(0, start)
      new_events.append({
        "plot_tag": "enabled_off_to_on_10",
        "reason": "enabled_rising(not_enabled->enabled)",
        "start_frame": start,
        "end_frame": end,
        "trigger_frame": self.frame,
        "logged_frames": set(),
      })

    # Event 2: was enabled + cruise standstill + RES, then cruise standstill clears (still enabled).
    if (
      self._proton_acc_log_prev_enabled
      and self._proton_acc_log_prev_standstill
      and self._proton_acc_log_prev_res
      and enabled
      and (not standstill)
    ):
      start = self.frame - 200
      end = self.frame + 200
      start = max(0, start)
      new_events.append({
        "plot_tag": "res_on_standstill_to_nonstandstill_200",
        "reason": "standstill_clear_after_res_on_standstill(enabled+cruise_standstill+RES -> non_standstill)",
        "start_frame": start,
        "end_frame": end,
        "trigger_frame": self.frame,
        "logged_frames": set(),
      })
      start300 = max(0, self.frame - 300)
      new_events.append({
        "plot_tag": "res_on_standstill_to_nonstandstill_300",
        "reason": "standstill_clear_after_res_on_standstill(enabled+cruise_standstill+RES -> non_standstill)_300frames",
        "start_frame": start300,
        "end_frame": self.frame + 300,
        "trigger_frame": self.frame,
        "logged_frames": set(),
      })

    # Event 4: enabled cruise standstill clears (no RES requirement).
    # This records the generic transition from cruise standstill to non-standstill while enabled.
    if (
      self._proton_acc_log_prev_enabled
      and self._proton_acc_log_prev_standstill
      and enabled
      and (not standstill)
    ):
      start100 = max(0, self.frame - 100)
      new_events.append({
        "plot_tag": "enabled_standstill_to_nonstandstill_100",
        "reason": "standstill_clear_while_enabled(enabled+standstill->non_standstill)_100frames",
        "start_frame": start100,
        "end_frame": self.frame + 100,
        "trigger_frame": self.frame,
        "logged_frames": set(),
      })

      start300 = max(0, self.frame - 300)
      new_events.append({
        "plot_tag": "enabled_standstill_to_nonstandstill_300",
        "reason": "standstill_clear_while_enabled(enabled+standstill->non_standstill)_300frames",
        "start_frame": start300,
        "end_frame": self.frame + 300,
        "trigger_frame": self.frame,
        "logged_frames": set(),
      })

    # Event 3: enabled cruise standstill rising (not standstill -> standstill)
    # This captures the moment the car transitions into standstill while cruise remains enabled.
    if (
      self._proton_acc_log_prev_enabled
      and (not self._proton_acc_log_prev_standstill)
      and enabled
      and standstill
    ):
      start = self.frame - 200
      end = self.frame + 200
      start = max(0, start)
      new_events.append({
        "plot_tag": "nonstandstill_to_standstill_200",
        "reason": "standstill_rising(enabled+not_standstill->standstill)",
        "start_frame": start,
        "end_frame": end,
        "trigger_frame": self.frame,
        "only_enabled_frames": True,
        "logged_frames": set(),
      })

    self._proton_acc_log_prev_enabled = enabled
    self._proton_acc_log_prev_standstill = standstill
    self._proton_acc_log_prev_res = res_pressed

    if not new_events:
      return

    # Header lines for each event.
    for ev in new_events:
      plot = ev.get("plot_tag", "unknown")
      pending = self._proton_acc_log_pending_lines_by_plot.setdefault(plot, [])
      pending.append(
        f"# plot={plot} reason={ev['reason']} trigger_frame={ev['trigger_frame']} log_range=[{ev['start_frame']},{ev['end_frame']}]"
      )
      self._proton_acc_log_events.append(ev)

  def _proton_acc_log_update(self, CC, CS, accel_cmd_scaled, standstill_request):
    # Capture snapshot for current frame and add to rolling history.
    snap = self._proton_acc_log_snapshot(CC, CS, accel_cmd_scaled, standstill_request)
    self._proton_acc_log_recent.append(snap)

    # Create any new events based on current frame state.
    self._proton_acc_log_maybe_create_events(CC, CS)

    # If events were created, immediately log pre-window frames from history.
    if self._proton_acc_log_events:
      # For each event, add any snapshots already in history that fall into its window.
      recent_snaps = list(self._proton_acc_log_recent)
      for ev in self._proton_acc_log_events:
        self._proton_acc_log_add_event_frames(ev, recent_snaps)

    # Drop finished events and flush when we are done (helps file visibility).
    still_active = []
    for ev in self._proton_acc_log_events:
      if self.frame <= ev["end_frame"]:
        still_active.append(ev)
    self._proton_acc_log_events = still_active

    # Flush at most once per second.
    self._proton_acc_log_maybe_write()

  def _compute_steer(self, CC, CS):
    new_steer = round(CC.actuators.torque * self.params.STEER_MAX)
    apply_steer = apply_proton_steer_torque_limits(new_steer, self.last_steer, 0, self.params)

    steer_enabled = CC.latActive
    if not steer_enabled and self.prev_steer_enabled:
      self.last_steer_disable = monotonic()
    self.prev_steer_enabled = steer_enabled

    # Stock Lane Departure Prevention / Centering Control (LKS Auxiliary / Blue line)
    lat_active = steer_enabled
    if (
      not steer_enabled
      and CS.stock_ldp_cmd > 0
      and not ((CS.out.rightBlinker and CS.stock_ldp_right) or (CS.out.leftBlinker and CS.stock_ldp_left))
    ):
      # Prevents sudden pull from LDP/ICC/LKA Centering after steer disable.
      blend = _clip((monotonic() - self.last_steer_disable - 0.55) / 0.5, 0.0, 1.0)
      apply_steer = round(CS.stock_ldp_cmd * (-1 if CS.stock_steer_dir else 1) * blend) & ~1 # Ensure cmd LSB 0 for 11-bit cmd
      lat_active = True

    return apply_steer, lat_active, steer_enabled

  def _update_sng(self, CC, CS, can_sends):
    if not (CS.cruise_standstill and CC.longActive):
      self.is_sng_check = False
      self.resume = False
      return

    self.resume = CS.res_btn_pressed
    if not self.is_sng_check:
      self.is_sng_check = True
      self.sng_next_press_frame = self.frame + 310
      self.resume_counter = 0
      return

    if self.resume or self.resume_counter >= 2:
      self.sng_next_press_frame = max(self.sng_next_press_frame, self.frame + 110)
      self.resume_counter = 0
      return

    # Brake check added for resume because S70 can still increase speed when standstill if brake pressed.
    if CC.actuators.accel > 0 and self.frame > self.sng_next_press_frame and not CS.out.brakePressed:
      self.resume = True
      can_sends.append(send_buttons(self.packer, False))
      self.resume_counter += 1

  def _update_cancel_spam(self, CS, pcm_cancel_cmd, can_sends):
    if not pcm_cancel_cmd:
      self.cancel_press_cnt = 0
      self.last_cancel_press = 0
      return

    if self.frame > self.last_cancel_press + 15 and not (CS.out.brakePressed and not CS.cruise_standstill):
      can_sends.append(send_buttons(self.packer, True))
      self.cancel_press_cnt += 1
      if self.cancel_press_cnt == 2:
        self.cancel_press_cnt = 0
        self.last_cancel_press = self.frame

  def update(self, CC, CS, now_nanos):
    del now_nanos
    can_sends = []
    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel
    accel_cmd = actuators.accel
    accel_cmd_raw = accel_cmd
    standstill_request = CS.out.standstill and CC.longActive

    apply_steer, lat_active, steer_enabled = self._compute_steer(CC, CS)

    if (self.frame % 2) == 0:
      # stock lane departure settings
      ldw_steering = CS.stock_ldw_steering
      if self.always_lks_tactile:
        ldw_steering = ldw_steering or CS.has_audio_ldw
        lks_audio, lks_tactile = False, True
      else:
        lks_audio, lks_tactile = CS.lks_audio, CS.lks_tactile

      self._update_sng(CC, CS, can_sends)

      is_x90 = self.CP.carFingerprint == CAR.PROTON_X90

      # TODO: Remove line below and test on X90 since stock LKA last bit is always 0 for any Proton car.
      steer_cmd = (round(apply_steer) * 2) if (is_x90 and CC.latActive) else apply_steer

      can_sends.append(
        create_can_steer_command(
          self.packer,
          steer_cmd,
          lat_active,
          CS.hand_on_wheel_warning and CS.is_icc_on,
          CS.hand_on_wheel_warning_2 and CS.is_icc_on,
          CS.lks_aux,
          lks_audio,
          lks_tactile,
          CS.lks_assist_mode,
          CS.lka_enable,
          ldw_steering,
          steer_enabled,
          is_x90,
        )
      )

      if self.openpilot_long:
        # Test mode: keep simulated ACC_CMD field logic, but replace actuators.accel
        # input with the stock observed ACC CMD value.
        if CS.stock_acc_cmd_values:
          accel_cmd_sim = float(CS.stock_acc_cmd_values.get("CMD", CS.stock_acc_cmd))
        else:
          accel_cmd_sim = float(CS.stock_acc_cmd)

        gas_ov = CS.out.gasPressed and CS.out.cruiseState.enabled
        can_sends.append(
          create_acc_cmd(
            self.packer,
            accel_cmd_sim,
            CC.longActive,
            gas_ov,
            CS.cruise_standstill,
            self.resume,
            CS.out.brakePressed,
            standstill_request,
          ),
        )

    self._update_cancel_spam(CS, pcm_cancel_cmd, can_sends)

    # Proton debug logger runs every controller frame.
    # It logs stock ACC_CMD vs the simulated ACC_CMD based on the same sim math used in protoncan.
    if self.openpilot_long:
      if CS.stock_acc_cmd_values:
        accel_cmd_scaled_for_log = float(CS.stock_acc_cmd_values.get("CMD", CS.stock_acc_cmd))
      else:
        accel_cmd_scaled_for_log = float(CS.stock_acc_cmd)
    else:
      accel_cmd_scaled_for_log = accel_cmd_raw
    self._proton_acc_log_update(CC, CS, accel_cmd_scaled_for_log, standstill_request)

    # Per-frame accel logging (stock vs actuators) for debugging/supprlt analysis.
    self._proton_accel_log_update(CC, CS, accel_cmd)

    self.last_steer = apply_steer
    new_actuators = actuators.as_builder()
    new_actuators.accel = accel_cmd / 15 if accel_cmd >= 0 else accel_cmd / 18
    new_actuators.torque = apply_steer / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_steer

    self.frame += 1
    return new_actuators, can_sends
