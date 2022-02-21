from enum import IntEnum
from typing import Dict, Union, Callable

from cereal import log, car
import cereal.messaging as messaging
from common.realtime import DT_CTRL
from selfdrive.config import Conversions as CV
from selfdrive.locationd.calibrationd import MIN_SPEED_FILTER

AlertSize = log.ControlsState.AlertSize
AlertStatus = log.ControlsState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
EventName = car.CarEvent.EventName


# Alert priorities
class Priority(IntEnum):
  LOWEST = 0
  LOWER = 1
  LOW = 2
  MID = 3
  HIGH = 4
  HIGHEST = 5


# Event types
class ET:
  ENABLE = 'enable'
  PRE_ENABLE = 'preEnable'
  NO_ENTRY = 'noEntry'
  WARNING = 'warning'
  USER_DISABLE = 'userDisable'
  SOFT_DISABLE = 'softDisable'
  IMMEDIATE_DISABLE = 'immediateDisable'
  PERMANENT = 'permanent'


# get event name from enum
EVENT_NAME = {v: k for k, v in EventName.schema.enumerants.items()}


class Events:
  def __init__(self):
    self.events = []
    self.static_events = []
    self.events_prev = dict.fromkeys(EVENTS.keys(), 0)

  @property
  def names(self):
    return self.events

  def __len__(self):
    return len(self.events)

  def add(self, event_name, static=False):
    if static:
      self.static_events.append(event_name)
    self.events.append(event_name)

  def clear(self):
    self.events_prev = {k: (v + 1 if k in self.events else 0) for k, v in self.events_prev.items()}
    self.events = self.static_events.copy()

  def any(self, event_type):
    for e in self.events:
      if event_type in EVENTS.get(e, {}).keys():
        return True
    return False

  def create_alerts(self, event_types, callback_args=None):
    if callback_args is None:
      callback_args = []

    ret = []
    for e in self.events:
      types = EVENTS[e].keys()
      for et in event_types:
        if et in types:
          alert = EVENTS[e][et]
          if not isinstance(alert, Alert):
            alert = alert(*callback_args)

          if DT_CTRL * (self.events_prev[e] + 1) >= alert.creation_delay:
            alert.alert_type = f"{EVENT_NAME[e]}/{et}"
            alert.event_type = et
            ret.append(alert)
    return ret

  def add_from_msg(self, events):
    for e in events:
      self.events.append(e.name.raw)

  def to_msg(self):
    ret = []
    for event_name in self.events:
      event = car.CarEvent.new_message()
      event.name = event_name
      for event_type in EVENTS.get(event_name, {}).keys():
        setattr(event, event_type, True)
      ret.append(event)
    return ret


class Alert:
  def __init__(self,
               alert_text_1: str,
               alert_text_2: str,
               alert_status: log.ControlsState.AlertStatus,
               alert_size: log.ControlsState.AlertSize,
               priority: Priority,
               visual_alert: car.CarControl.HUDControl.VisualAlert,
               audible_alert: car.CarControl.HUDControl.AudibleAlert,
               duration: float,
               alert_rate: float = 0.,
               creation_delay: float = 0.):

    self.alert_text_1 = alert_text_1
    self.alert_text_2 = alert_text_2
    self.alert_status = alert_status
    self.alert_size = alert_size
    self.priority = priority
    self.visual_alert = visual_alert
    self.audible_alert = audible_alert

    self.duration = int(duration / DT_CTRL)

    self.alert_rate = alert_rate
    self.creation_delay = creation_delay

    self.alert_type = ""
    self.event_type = None

  def __str__(self) -> str:
    return f"{self.alert_text_1}/{self.alert_text_2} {self.priority} {self.visual_alert} {self.audible_alert}"

  def __gt__(self, alert2) -> bool:
    return self.priority > alert2.priority


class NoEntryAlert(Alert):
  def __init__(self, alert_text_2, visual_alert=VisualAlert.none):
    super().__init__("openpilot No disponible", alert_text_2, AlertStatus.normal,
                     AlertSize.mid, Priority.LOW, visual_alert,
                     AudibleAlert.refuse, 3.)


class SoftDisableAlert(Alert):
  def __init__(self, alert_text_2):
    super().__init__("TOME EL CONTROL INMEDIATAMENTE", alert_text_2,
                     AlertStatus.userPrompt, AlertSize.full,
                     Priority.MID, VisualAlert.steerRequired,
                     AudibleAlert.warningSoft, 2.),


# less harsh version of SoftDisable, where the condition is user-triggered
class UserSoftDisableAlert(SoftDisableAlert):
  def __init__(self, alert_text_2):
    super().__init__(alert_text_2),
    self.alert_text_1 = "openpilot irá desengatar"


class ImmediateDisableAlert(Alert):
  def __init__(self, alert_text_2):
    super().__init__("TOME EL CONTROL INMEDIATAMENTE", alert_text_2,
                     AlertStatus.critical, AlertSize.full,
                     Priority.HIGHEST, VisualAlert.steerRequired,
                     AudibleAlert.warningImmediate, 4.),


class EngagementAlert(Alert):
  def __init__(self, audible_alert: car.CarControl.HUDControl.AudibleAlert):
    super().__init__("", "",
                     AlertStatus.normal, AlertSize.none,
                     Priority.MID, VisualAlert.none,
                     audible_alert, .2),


class NormalPermanentAlert(Alert):
  def __init__(self, alert_text_1: str, alert_text_2: str = "", duration: float = 0.2, priority: Priority = Priority.LOWER, creation_delay: float = 0.):
    super().__init__(alert_text_1, alert_text_2,
                     AlertStatus.normal, AlertSize.mid if len(alert_text_2) else AlertSize.small,
                     priority, VisualAlert.none, AudibleAlert.none, duration, creation_delay=creation_delay),


class StartupAlert(Alert):
  def __init__(self, alert_text_1: str, alert_text_2: str = "Para usar MADS, oprima el botón LKAS", alert_status=AlertStatus.normal):
    super().__init__(alert_text_1, alert_text_2,
                     alert_status, AlertSize.mid,
                     Priority.LOWER, VisualAlert.none, AudibleAlert.none, 10.),


# ********** helper functions **********
def get_display_speed(speed_ms: float, metric: bool) -> str:
  speed = int(round(speed_ms * (CV.MS_TO_KPH if metric else CV.MS_TO_MPH)))
  unit = 'km/h' if metric else 'mph'
  return f"{speed} {unit}"


# ********** alert callback functions **********

AlertCallbackType = Callable[[car.CarParams, messaging.SubMaster, bool, int], Alert]


def soft_disable_alert(alert_text_2: str) -> AlertCallbackType:
  def func(CP: car.CarParams, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
    if soft_disable_time < int(0.5 / DT_CTRL):
      return ImmediateDisableAlert(alert_text_2)
    return SoftDisableAlert(alert_text_2)
  return func


def user_soft_disable_alert(alert_text_2: str) -> AlertCallbackType:
  def func(CP: car.CarParams, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
    if soft_disable_time < int(0.5 / DT_CTRL):
      return ImmediateDisableAlert(alert_text_2)
    return UserSoftDisableAlert(alert_text_2)
  return func


def below_engage_speed_alert(CP: car.CarParams, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return NoEntryAlert(f"Velocidade por debajo de {get_display_speed(CP.minEnableSpeed, metric)}")


def below_steer_speed_alert(CP: car.CarParams, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return Alert(
    f"Conducción no disponible por debajo de {get_display_speed(CP.minSteerSpeed, metric)}",
    "",
    AlertStatus.userPrompt, AlertSize.small,
    Priority.MID, VisualAlert.steerRequired, AudibleAlert.prompt, 0.4)


def calibration_incomplete_alert(CP: car.CarParams, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return Alert(
    "Calibración em Processo: %d%%" % sm['liveCalibration'].calPerc,
    f"Maneje por encima de {get_display_speed(MIN_SPEED_FILTER, metric)}",
    AlertStatus.normal, AlertSize.mid,
    Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .2)


def no_gps_alert(CP: car.CarParams, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  gps_integrated = sm['peripheralState'].pandaType in [log.PandaState.PandaType.uno, log.PandaState.PandaType.dos]
  return Alert(
    "Baja recepción GPS",
    "Si el cielo es visible, contacte a soporte" if gps_integrated else "Check GPS antenna placement",
    AlertStatus.normal, AlertSize.mid,
    Priority.LOWER, VisualAlert.none, AudibleAlert.none, .2, creation_delay=300.)


def wrong_car_mode_alert(CP: car.CarParams, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  text = "Modo Crucero Deshabilitado"
  if CP.carName == "honda":
    text = "Interruptor principal apagado"
  return NoEntryAlert(text)


def joystick_alert(CP: car.CarParams, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  axes = sm['testJoystick'].axes
  gb, steer = list(axes)[:2] if len(axes) else (0., 0.)
  vals = f"Gas: {round(gb * 100.)}%, Steer: {round(steer * 100.)}%"
  return NormalPermanentAlert("Joystick Mode", vals)

def speed_limit_adjust_alert(CP: car.CarParams, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  speedLimit = sm['longitudinalPlan'].speedLimit
  speed = round(speedLimit * (CV.MS_TO_KPH if metric else CV.MS_TO_MPH))
  message = f'Ajustando límite de velididad a {speed} {"km/h" if metric else "mph"}'
  return Alert(
    message,
    "",
    AlertStatus.normal, AlertSize.small,
    Priority.LOW, VisualAlert.none, AudibleAlert.none, 4.)


EVENTS: Dict[int, Dict[str, Union[Alert, AlertCallbackType]]] = {
  # ********** events with no alerts **********

  EventName.stockFcw: {},

  # ********** events only containing alerts displayed in all states **********

  EventName.joystickDebug: {
    ET.WARNING: joystick_alert,
    ET.PERMANENT: NormalPermanentAlert("Joystick Mode"),
  },

  EventName.controlsInitializing: {
    ET.NO_ENTRY: NoEntryAlert("Iniciando Sistema"),
  },

  EventName.startup: {
    ET.PERMANENT: StartupAlert("sunnypilot Iniciado")
  },

  EventName.startupMaster: {
    ET.PERMANENT: StartupAlert("WARNING: This branch is not tested",
                               alert_status=AlertStatus.userPrompt),
  },

  # Car is recognized, but marked as dashcam only
  EventName.startupNoControl: {
    ET.PERMANENT: StartupAlert("modo Dashcam"),
  },

  # Car is not recognized
  EventName.startupNoCar: {
    ET.PERMANENT: StartupAlert("modo Dashcam para autos no soportados"),
  },

  EventName.startupNoFw: {
    ET.PERMANENT: StartupAlert("Auto no reconocido",
                               "Revise coneciones eléctricas del Comma",
                               alert_status=AlertStatus.userPrompt),
  },

  EventName.dashcamMode: {
    ET.PERMANENT: NormalPermanentAlert("Modo Dashcam",
                                       priority=Priority.LOWEST),
  },

  EventName.invalidLkasSetting: {
    ET.PERMANENT: NormalPermanentAlert("Stock LKAS esta encendido",
                                       "Apague stock LKAS para activar"),
  },

  EventName.cruiseMismatch: {
    #ET.PERMANENT: ImmediateDisableAlert("openpilot failed to cancel cruise"),
  },

  # Some features or cars are marked as community features. If openpilot
  # detects the use of a community feature it switches to dashcam mode
  # until these features are allowed using a toggle in settings.
  EventName.communityFeatureDisallowed: {
    ET.PERMANENT: NormalPermanentAlert("openpilot No disponible",
                                       "En configuración, habilite Community Features"),
  },

  # openpilot doesn't recognize the car. This switches openpilot into a
  # read-only mode. This can be solved by adding your fingerprint.
  # See https://github.com/commaai/openpilot/wiki/Fingerprinting for more information
  EventName.carUnrecognized: {
    ET.PERMANENT: NormalPermanentAlert("Modo Dashcam",
                                       "Auto no Reconocido",
                                       priority=Priority.LOWEST),
  },

  EventName.stockAeb: {
    ET.PERMANENT: Alert(
      "FRENE!",
      "AEB Stock: Riesgo de colisión",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.fcw, AudibleAlert.none, 2.),
    ET.NO_ENTRY: NoEntryAlert("AEB Stock: Riesgo de colisión"),
  },

  EventName.fcw: {
    ET.PERMANENT: Alert(
      "FRENE!",
      "Riesgo de Colisión",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.fcw, AudibleAlert.warningSoft, 2.),
  },

  EventName.ldw: {
    ET.PERMANENT: Alert(
      "Salida de carril detectada",
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.ldw, AudibleAlert.prompt, 3.),
  },

  # ********** events only containing alerts that display while engaged **********

  EventName.gasPressed: {
    ET.PRE_ENABLE: Alert(
      "Levante acelerador para activar",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .1, creation_delay=1.),
  },

  # openpilot tries to learn certain parameters about your car by observing
  # how the car behaves to steering inputs from both human and openpilot driving.
  # This includes:
  # - steer ratio: gear ratio of the steering rack. Steering angle divided by tire angle
  # - tire stiffness: how much grip your tires have
  # - angle offset: most steering angle sensors are offset and measure a non zero angle when driving straight
  # This alert is thrown when any of these values exceed a sanity check. This can be caused by
  # bad alignment or bad sensor data. If this happens consistently consider creating an issue on GitHub
  EventName.vehicleModelInvalid: {
    ET.NO_ENTRY: NoEntryAlert("Vehicle Parameter Identification Failed"),
    ET.SOFT_DISABLE: soft_disable_alert("Vehicle Parameter Identification Failed"),
  },

  EventName.steerTempUnavailableSilent: {
    ET.WARNING: Alert(
      "Dirección no disponible temporalmente",
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.prompt, 1.),
  },

  EventName.preDriverDistracted: {
    ET.WARNING: Alert(
      "Preste atención",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1),
  },

  EventName.promptDriverDistracted: {
    ET.WARNING: Alert(
      "Preste atención",
      "Conductor distraído",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.promptDistracted, .1),
  },

  EventName.driverDistracted: {
    ET.WARNING: Alert(
      "DESCONECTAR INMEDIATAMENTE",
      "Conductor distraído",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.warningImmediate, .1),
  },

  EventName.preDriverUnresponsive: {
    ET.WARNING: Alert(
      "Toque el volante: Rostro no detectado",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.none, .1, alert_rate=0.75),
  },

  EventName.promptDriverUnresponsive: {
    ET.WARNING: Alert(
      "Toque el Volante",
      "Conductor sin respuesta",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.promptDistracted, .1),
  },

  EventName.driverUnresponsive: {
    ET.WARNING: Alert(
      "DESCONECTAR INMEDIATAMENTE",
      "Conductor sin respuesta",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.warningImmediate, .1),
  },

  EventName.preKeepHandsOnWheel: {
    ET.WARNING: Alert(
      "No se detectan manos en el volante",
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.none, .1, alert_rate=0.75),
  },

  EventName.promptKeepHandsOnWheel: {
    ET.WARNING: Alert(
      "MANOS FUERA DEL VOLANTE",
      "Coloque las manos en el volante",
      AlertStatus.critical, AlertSize.mid,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.promptDistracted, .1),
  },

  EventName.keepHandsOnWheel: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Conductor mantiene las manos retiradas del volante"),
  },

  EventName.manualRestart: {
    ET.WARNING: Alert(
      "TOME EL CONTROL",
      "Tome la conducción manualmente",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .2),
  },

  EventName.resumeRequired: {
    ET.WARNING: Alert(
      "DETENIDO",
      "Presione Resumir para continuar",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .2),
  },

  EventName.belowSteerSpeed: {
    ET.WARNING: below_steer_speed_alert,
  },

  EventName.preLaneChangeLeft: {
    ET.WARNING: Alert(
      "Cuando sea seguro, gire volante a la izquierda para iniciar el cambio de carril",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1, alert_rate=0.75),
  },

  EventName.preLaneChangeRight: {
    ET.WARNING: Alert(
      "Cuando sea seguro, gire volante a la derecha para iniciar el cambio de carril",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1, alert_rate=0.75),
  },

  EventName.laneChangeBlocked: {
    ET.WARNING: Alert(
      "Vehículo detectado en punto ciego",
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.prompt, .1),
  },

  EventName.laneChange: {
    ET.WARNING: Alert(
      "Cambiando de carril",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1),
  },

  EventName.manualSteeringRequired: {
    ET.WARNING: Alert(
      "MADS está APAGADO",
      "Dirección manual necesaria",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.disengage, 2.),
  },

  EventName.manualLongitudinalRequired: {
    ET.WARNING: Alert(
      "Control de crucero está APAGADO",
      "Accelerador/freno manual necesarios",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 2.),
  },

  EventName.steerSaturated: {
    ET.WARNING: Alert(
      "Tome el control",
      "La curva excede el límite de dirección",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.promptRepeat, 1.),
  },

  # Thrown when the fan is driven at >50% but is not rotating
  EventName.fanMalfunction: {
    ET.PERMANENT: NormalPermanentAlert("Daño en el ventilador", "Contacte a Soporte"),
  },

  # Camera is not outputting frames at a constant framerate
  EventName.cameraMalfunction: {
    ET.PERMANENT: NormalPermanentAlert("Daño en la Camera", "Contacte a Soporte"),
  },

  # Unused
  EventName.gpsMalfunction: {
    ET.PERMANENT: NormalPermanentAlert("Daño en el GPS", "Contacte a Soporte"),
  },

  # When the GPS position and localizer diverge the localizer is reset to the
  # current GPS position. This alert is thrown when the localizer is reset
  # more often than expected.
  EventName.localizerMalfunction: {
    ET.PERMANENT: NormalPermanentAlert("Error en el Sensor", "Contacte a Soporte"),
  },

  EventName.speedLimitActive: {
    ET.WARNING: Alert(
      "Control Crucero ajustado al límite de velocidad",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 2.),
  },

  EventName.speedLimitValueChange: {
    ET.WARNING: speed_limit_adjust_alert,
  },

  # ********** events that affect controls state transitions **********

  EventName.pcmEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.engage),
  },

  EventName.buttonEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.engage),
  },

  EventName.silentButtonEnable: {
    ET.ENABLE: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.MID, VisualAlert.none, AudibleAlert.none, .2),
  },

  EventName.pcmDisable: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
  },

  EventName.buttonCancel: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
  },

  EventName.brakeHold: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert("Brake Hold Activo"),
  },

  EventName.silentBrakeHold: {
    ET.USER_DISABLE: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.MID, VisualAlert.none, AudibleAlert.none, .2),
    ET.NO_ENTRY: NoEntryAlert("Brake Hold Activo"),
  },

  EventName.parkBrake: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert("Freno de parqueo activado"),
  },

  EventName.pedalPressed: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert("Pedal presionado",
                              visual_alert=VisualAlert.brakePressed),
  },

  EventName.silentPedalPressed: {
    ET.USER_DISABLE: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.MID, VisualAlert.none, AudibleAlert.none, .2),
    ET.NO_ENTRY: NoEntryAlert("Pedal presionado durante el intento",
                              visual_alert=VisualAlert.brakePressed),
  },

  EventName.wrongCarMode: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: wrong_car_mode_alert,
  },

  EventName.wrongCruiseMode: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert("Control Crucero Adaptativo desactivado"),
  },

  EventName.steerTempUnavailable: {
    ET.SOFT_DISABLE: soft_disable_alert("Volante no disponible temporalmente"),
    ET.NO_ENTRY: NoEntryAlert("Volante no disponible temporalmente"),
  },

  EventName.outOfSpace: {
    ET.PERMANENT: NormalPermanentAlert("Out of Storage"),
    ET.NO_ENTRY: NoEntryAlert("Out of Storage"),
  },

  EventName.belowEngageSpeed: {
    ET.NO_ENTRY: below_engage_speed_alert,
  },

  EventName.sensorDataInvalid: {
    ET.PERMANENT: Alert(
      "No Data from Device Sensors",
      "Reboot your Device",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOWER, VisualAlert.none, AudibleAlert.none, .2, creation_delay=1.),
    ET.NO_ENTRY: NoEntryAlert("No Data from Device Sensors"),
  },

  EventName.noGps: {
    ET.PERMANENT: no_gps_alert,
  },

  EventName.soundsUnavailable: {
    ET.PERMANENT: NormalPermanentAlert("Speaker not found", "Reboot your Device"),
    ET.NO_ENTRY: NoEntryAlert("Speaker not found"),
  },

  EventName.tooDistracted: {
    ET.NO_ENTRY: NoEntryAlert("Nivel de distracción muy alto"),
  },

  EventName.overheat: {
    ET.PERMANENT: NormalPermanentAlert("Sistema sobrecalentado"),
    ET.SOFT_DISABLE: soft_disable_alert("Sistema sobrecalentado"),
    ET.NO_ENTRY: NoEntryAlert("Sistema sobrecalentado"),
  },

  EventName.wrongGear: {
    ET.SOFT_DISABLE: user_soft_disable_alert("Palanca de cambios no está en D"),
    ET.NO_ENTRY: NoEntryAlert("Palanca de cambios no está en D"),
  },

  EventName.silentWrongGear: {
    ET.SOFT_DISABLE: Alert(
      "Palanca de cambios no está en D",
      "Openpilot no esta disponible",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 0., 2., 3.),
    ET.NO_ENTRY: Alert(
      "Palanca de cambios no está en D",
      "Openpilot no esta disponible",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 0., 2., 3.),
  },

  # This alert is thrown when the calibration angles are outside of the acceptable range.
  # For example if the device is pointed too much to the left or the right.
  # Usually this can only be solved by removing the mount from the windshield completely,
  # and attaching while making sure the device is pointed straight forward and is level.
  # See https://comma.ai/setup for more information
  EventName.calibrationInvalid: {
    ET.PERMANENT: NormalPermanentAlert("Calibración Inválida", "Reubique Dispositivo y Recalibre"),
    ET.SOFT_DISABLE: soft_disable_alert("Calibración Inválida: Reubique Dispositivo y Recalibre"),
    ET.NO_ENTRY: NoEntryAlert("Calibración Inválida: Reubique Dispositivo y Recalibre"),
  },

  EventName.calibrationIncomplete: {
    ET.PERMANENT: calibration_incomplete_alert,
    ET.SOFT_DISABLE: soft_disable_alert("Calibración en progreso"),
    ET.NO_ENTRY: NoEntryAlert("Calibración en progreso"),
  },

  EventName.doorOpen: {
    ET.SOFT_DISABLE: user_soft_disable_alert("Puerta abierta"),
    ET.NO_ENTRY: NoEntryAlert("Puerta abierta"),
  },

  EventName.seatbeltNotLatched: {
    ET.SOFT_DISABLE: user_soft_disable_alert("Cinturón de seguridad desabrochado"),
    ET.NO_ENTRY: NoEntryAlert("Cinturón de seguridad desabrochado"),
  },

  EventName.espDisabled: {
    ET.SOFT_DISABLE: soft_disable_alert("ESP Off"),
    ET.NO_ENTRY: NoEntryAlert("ESP Off"),
  },

  EventName.lowBattery: {
    ET.SOFT_DISABLE: soft_disable_alert("Low Battery"),
    ET.NO_ENTRY: NoEntryAlert("Low Battery"),
  },

  # Different openpilot services communicate between each other at a certain
  # interval. If communication does not follow the regular schedule this alert
  # is thrown. This can mean a service crashed, did not broadcast a message for
  # ten times the regular interval, or the average interval is more than 10% too high.
  EventName.commIssue: {
    ET.SOFT_DISABLE: soft_disable_alert("Communication Issue between Processes"),
    ET.NO_ENTRY: NoEntryAlert("Communication Issue between Processes"),
  },

  # Thrown when manager detects a service exited unexpectedly while driving
  EventName.processNotRunning: {
    ET.NO_ENTRY: NoEntryAlert("Mal funcionamiento del sistema: Reinicie su dispositivo"),
  },

  EventName.radarFault: {
    ET.SOFT_DISABLE: soft_disable_alert("Radar Error: Restart the Car"),
    ET.NO_ENTRY: NoEntryAlert("Radar Error: Restart the Car"),
  },

  # Every frame from the camera should be processed by the model. If modeld
  # is not processing frames fast enough they have to be dropped. This alert is
  # thrown when over 20% of frames are dropped.
  EventName.modeldLagging: {
    ET.SOFT_DISABLE: soft_disable_alert("Driving model lagging"),
    ET.NO_ENTRY: NoEntryAlert("Driving model lagging"),
  },

  # Besides predicting the path, lane lines and lead car data the model also
  # predicts the current velocity and rotation speed of the car. If the model is
  # very uncertain about the current velocity while the car is moving, this
  # usually means the model has trouble understanding the scene. This is used
  # as a heuristic to warn the driver.
  EventName.posenetInvalid: {
    ET.SOFT_DISABLE: soft_disable_alert("Model Output Uncertain"),
    ET.NO_ENTRY: NoEntryAlert("Model Output Uncertain"),
  },

  # When the localizer detects an acceleration of more than 40 m/s^2 (~4G) we
  # alert the driver the device might have fallen from the windshield.
  EventName.deviceFalling: {
    ET.SOFT_DISABLE: soft_disable_alert("Device Fell Off Mount"),
    ET.NO_ENTRY: NoEntryAlert("Device Fell Off Mount"),
  },

  EventName.lowMemory: {
    ET.SOFT_DISABLE: soft_disable_alert("Low Memory: Reboot Your Device"),
    ET.PERMANENT: NormalPermanentAlert("Low Memory", "Reboot your Device"),
    ET.NO_ENTRY: NoEntryAlert("Low Memory: Reboot Your Device"),
  },

  EventName.highCpuUsage: {
    #ET.SOFT_DISABLE: soft_disable_alert("System Malfunction: Reboot Your Device"),
    #ET.PERMANENT: NormalPermanentAlert("System Malfunction", "Reboot your Device"),
    ET.NO_ENTRY: NoEntryAlert("Mal funcionamiento del sistema: Reinicie su dispositivo"),
  },

  EventName.accFaulted: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Cruise Faulted"),
    ET.PERMANENT: NormalPermanentAlert("Cruise Faulted", ""),
    ET.NO_ENTRY: NoEntryAlert("Cruise Faulted"),
  },

  EventName.controlsMismatch: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Controls Mismatch"),
  },

  EventName.roadCameraError: {
    ET.PERMANENT: NormalPermanentAlert("Camera Error",
                                       duration=1.,
                                       creation_delay=30.),
  },

  EventName.driverCameraError: {
    ET.PERMANENT: NormalPermanentAlert("Camera Error",
                                       duration=1.,
                                       creation_delay=30.),
  },

  EventName.wideRoadCameraError: {
    ET.PERMANENT: NormalPermanentAlert("Camera Error",
                                       duration=1.,
                                       creation_delay=30.),
  },

  # Sometimes the USB stack on the device can get into a bad state
  # causing the connection to the panda to be lost
  EventName.usbError: {
    ET.SOFT_DISABLE: soft_disable_alert("USB Error: Reboot Your Device"),
    ET.PERMANENT: NormalPermanentAlert("USB Error: Reboot Your Device", ""),
    ET.NO_ENTRY: NoEntryAlert("USB Error: Reboot Your Device"),
  },

  # This alert can be thrown for the following reasons:
  # - No CAN data received at all
  # - CAN data is received, but some message are not received at the right frequency
  # If you're not writing a new car port, this is usually cause by faulty wiring
  EventName.canError: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("CAN Error: Check Connections"),
    ET.PERMANENT: Alert(
      "CAN Error: Check Connections",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1., creation_delay=1.),
    ET.NO_ENTRY: NoEntryAlert("CAN Error: Check Connections"),
  },

  EventName.steerUnavailable: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("LKAS Fault: Restart the Car"),
    ET.PERMANENT: NormalPermanentAlert("LKAS Fault: Restart the car to engage"),
    ET.NO_ENTRY: NoEntryAlert("LKAS Fault: Restart the Car"),
  },

  EventName.brakeUnavailable: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Cruise Fault: Restart the Car"),
    ET.PERMANENT: NormalPermanentAlert("Cruise Fault: Restart the car to engage"),
    ET.NO_ENTRY: NoEntryAlert("Cruise Fault: Restart the Car"),
  },

  EventName.reverseGear: {
    ET.PERMANENT: Alert(
      "Marcha\nAtrás",
      "",
      AlertStatus.normal, AlertSize.full,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .2, creation_delay=0.5),
    ET.USER_DISABLE: ImmediateDisableAlert("Marcha atrás"),
    ET.NO_ENTRY: NoEntryAlert("Marcha atrás"),
  },

  # On cars that use stock ACC the car can decide to cancel ACC for various reasons.
  # When this happens we can no long control the car so the user needs to be warned immediately.
  EventName.cruiseDisabled: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Cruise Is Off"),
  },

  # For planning the trajectory Model Predictive Control (MPC) is used. This is
  # an optimization algorithm that is not guaranteed to find a feasible solution.
  # If no solution is found or the solution has a very high cost this alert is thrown.
  EventName.plannerError: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Planner Solution Error"),
    ET.NO_ENTRY: NoEntryAlert("Planner Solution Error"),
  },

  # When the relay in the harness box opens the CAN bus between the LKAS camera
  # and the rest of the car is separated. When messages from the LKAS camera
  # are received on the car side this usually means the relay hasn't opened correctly
  # and this alert is thrown.
  EventName.relayMalfunction: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Harness Malfunction"),
    ET.PERMANENT: NormalPermanentAlert("Harness Malfunction", "Check Hardware"),
    ET.NO_ENTRY: NoEntryAlert("Harness Malfunction"),
  },

  EventName.noTarget: {
    ET.IMMEDIATE_DISABLE: Alert(
      "openpilot Canceled",
      "No close lead car",
      AlertStatus.normal, AlertSize.mid,
      Priority.HIGH, VisualAlert.none, AudibleAlert.disengage, 3.),
    ET.NO_ENTRY: NoEntryAlert("No Close Lead Car"),
  },

  EventName.speedTooLow: {
    ET.IMMEDIATE_DISABLE: Alert(
      "openpilot Canceled",
      "Speed too low",
      AlertStatus.normal, AlertSize.mid,
      Priority.HIGH, VisualAlert.none, AudibleAlert.disengage, 3.),
  },

  # When the car is driving faster than most cars in the training data, the model outputs can be unpredictable.
  EventName.speedTooHigh: {
    ET.WARNING: Alert(
      "Velocidad muy alta",
      "Model uncertain at this speed",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.promptRepeat, 4.),
    ET.NO_ENTRY: NoEntryAlert("Reduzca para activar"),
  },

  EventName.lowSpeedLockout: {
    ET.PERMANENT: NormalPermanentAlert("Error de Control Crucero: Reinicie el vehiculo"),
    ET.NO_ENTRY: NoEntryAlert("Error de Control Crucero: Reinicie el vehiculo"),
  },

}
