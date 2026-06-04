#!/usr/bin/env python3

import sys
import math
import time
import traceback
from xarm import version
from xarm.wrapper import XArmAPI

# =============================================================================
# ESTADOS DE LA TAREA
# =============================================================================
#  0  — Vacío / disponible
#  1  — Base depositada en Banda 1 (esperando CI3, no bloqueante)
#  2  — Transferido a Banda 2, CO1 enviado (esperando ACK del PLC, no bloqueante)
#  3  — ACK recibido, listo para ensamblar tapa (esperando turno del robot)
#  4  — Tapa ensamblada, listo para paletizar (esperando turno del robot)
#  5  — En paletizado — estado transitorio, nunca persiste entre iteraciones
#  6  — Completado — el slot se libera al inicio de la siguiente iteración
# =============================================================================

MAX_CONCURRENT = 2          # Máximo de pasteles en vuelo simultáneamente
CI3_TIMEOUT    = 1200.0      # Segundos antes de abortar espera de fin de banda 1
PLC_TIMEOUT    = 1200.0      # Segundos antes de abortar espera de ACK del PLC


class CakeTask:
    """Representa un pastel en tránsito por la línea de producción."""
    _next_id = 1

    def __init__(self, flavor: str, idx: int):
        self.id              = CakeTask._next_id
        CakeTask._next_id   += 1
        self.flavor          = flavor
        self.idx             = idx
        self.state           = 0
        self.lid_ack_received = False
        self.ci3_deadline    = None
        self.plc_deadline    = None

    def __repr__(self):
        return (f"<CakeTask id={self.id} flavor={self.flavor} "
                f"idx={self.idx} state={self.state} "
                f"lid_ack={self.lid_ack_received}>")


class RobotMain(object):
    """Robot Main Class — versión con pipelining y corrección de índices."""

    def __init__(self, robot, **kwargs):
        self.alive = True
        self._arm  = robot
        self._ignore_exit_state = False
        self._tcp_speed  = 100
        self._tcp_acc    = 2000
        self._angle_speed = 20
        self._angle_acc   = 500
        self._vars  = {}
        self._funcs = {}

        self.X_1ST = [5.0, 55.0, 105.0]
        self.X_2ND = [155.0, 205.0, 255.0]
        self.Y_FLAVOR = {
            'choco':    -200.0,
            'vainilla': -250.0,
            'straw':    -300.0,
        }
        self.Z_PLACE_1ST = 144.7

        self.ASSEMBLY_X     = 252.1
        self.ASSEMBLY_Y     = -2.6
        self.Z_BASE_BANDA2  = 56.4
        self.Z_TAPA_BANDA2  = 69.4
        self.Z_PICK_PASTEL  = 55.0

        # Contador de columna de gradilla por sabor (0, 1 o 2)
        self.cake_count = {'choco': 0, 'vainilla': 0, 'straw': 0}

        self.PALLET_POSITIONS = [
            (145.0, 280.0),
            (185.0, 280.0),
            (145.0, 240.0),
            (185.0, 240.0),
        ]
        self.PALLET_Z_PISO1   = 10.0
        self.PALLET_Z_PISO2   = 45.0
        self.global_pallet_count = 0

        self.active_tasks: list[CakeTask] = []
        self._robot_init()

    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'register_count_changed_callback'):
            self._arm.register_count_changed_callback(self._count_changed_callback)
        try:
            self._arm.set_cgpio_digital(1, 0)
            self._arm.set_cgpio_digital(2, 0)
            self._arm.set_cgpio_digital(3, 0)
        except Exception as e:
            self.pprint(f'GPIO init warning (no crítico): {e}')

    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    def _state_changed_callback(self, data):
        if not self._ignore_exit_state and data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    def _count_changed_callback(self, data):
        if self.is_alive:
            self.pprint('counter val: {}'.format(data['count']))

    def _check_code(self, code, label):
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}, ret2={}'.format(
                label, code, self._arm.connected, self._arm.state,
                self._arm.error_code, ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(
                time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())),
                stack_tuple[1],
                ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def arm(self):
        return self._arm

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._ignore_exit_state:
                return True
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        return False

    def _gpio_match(self, pattern, timeout=0.05):
        return self._arm.arm.get_cgpio_li_state(pattern, timeout=timeout, is_ci=True)

    def _detect_flavor(self):
        if self._gpio_match([1, 0, 0, 1, 1, 1, 1, 1]):
            return 'straw'
        if self._gpio_match([1, 0, 1, 1, 1, 1, 1, 1]):
            return 'choco'
        if self._gpio_match([1, 1, 0, 1, 1, 1, 1, 1]):
            return 'vainilla'
        return None

    def _detect_plc_ack(self, flavor):
        if flavor == 'choco'    and self._gpio_match([1, 0, 1, 0, 1, 1, 1, 1]):
            return True
        if flavor == 'vainilla' and self._gpio_match([1, 1, 0, 0, 1, 1, 1, 1]):
            return True
        if flavor == 'straw'    and self._gpio_match([1, 0, 0, 0, 1, 1, 1, 1]):
            return True
        return False

    def _diagnostico_inicial(self):
        print("=" * 55)
        print("DIAGNÓSTICO INICIAL")
        print(f"   connected   = {self._arm.connected}")
        print(f"   state       = {self._arm.state}")
        print(f"   error_code  = {self._arm.error_code}")
        print(f"   alive       = {self.alive}")
        print(f"   is_alive    = {self.is_alive}")
        print()
        for i in range(4):
            pat = [1] * 8
            pat[i] = 0
            estado = self._gpio_match(pat, timeout=0.02)
            print(f"   CI{i} = {'LOW (activo)' if estado else 'HIGH / no activo'}")
        print("=" * 55)

    def _pick_place_1st(self, flavor, idx):
        target_x = self.X_1ST[idx]
        target_y = self.Y_FLAVOR[flavor]
        print(f"[PASO A] {flavor} col={idx+1}  x={target_x}, y={target_y}")

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False

        code = self._arm.set_position(*[200.0, -100.0, 200.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'): return False

        code = self._arm.set_position(*[target_x, target_y, 50.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        code = self._arm.set_position(*[target_x, target_y, 10.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.8)

        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'): return False
        time.sleep(1.2)

        code = self._arm.set_position(*[target_x, target_y, 50.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        code = self._arm.set_position(*[200.0, -100.0, 200.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'): return False

        code = self._arm.set_position(*[200.0, 110.0, 200.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'): return False

        code = self._arm.set_position(*[-37.8, 243.4, 193.3, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        code = self._arm.set_position(*[-37.8, 243.4, self.Z_PLACE_1ST, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False
        time.sleep(0.8)

        code = self._arm.set_position(*[-37.8, 243.4, 193.3, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        code = self._arm.set_position(*[100.0, 243.4, 193.3, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)
        return True

    def _conveyor_to_conveyor_transfer(self):
        print("[PASO C] Trasladando pieza Banda 1 → Banda 2...")
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False

        code = self._arm.set_position(*[279.5, 163.5, 190.0, 180.0, 0.0, 90.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(1.0)

        code = self._arm.set_position(*[279.5, 163.5, 59.4, 180.0, 0.0, 90.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(1.0)

        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'): return False
        time.sleep(1.4)

        code = self._arm.set_position(*[279.5, 163.5, 127.4, 180.0, 0.0, 90.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(1.0)

        code = self._arm.set_position(*[self.ASSEMBLY_X, self.ASSEMBLY_Y, 190.0, 180.0, 0.0, 90.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(1.0)

        code = self._arm.set_position(*[self.ASSEMBLY_X, self.ASSEMBLY_Y, self.Z_BASE_BANDA2, 180.0, 0.0, 90.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(1.0)

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False
        time.sleep(1.0)

        code = self._arm.set_position(*[self.ASSEMBLY_X, self.ASSEMBLY_Y, 86.5, 180.0, 0.0, 90.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        return True

    def _send_co1(self):
        print("[PASO D] Enviando CO1=1 al PLC (base lista en Banda 2)...")
        try:
            self._arm.set_cgpio_digital(1, 1)
            self._arm.set_cgpio_digital(2, 0)
            self._arm.set_cgpio_digital(3, 0)
            time.sleep(0.5)
            self._arm.set_cgpio_digital(1, 0)
            self._arm.set_cgpio_digital(2, 0)
            self._arm.set_cgpio_digital(3, 0)
        except Exception as e:
            self.pprint(f'GPIO CO1 warning: {e}')

    def _pick_place_2nd(self, flavor, idx):
        target_x = self.X_2ND[idx]
        target_y = self.Y_FLAVOR[flavor]
        print(f"[PASO F] {flavor} tapa col={idx+1}  x={target_x}, y={target_y}")

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False

        code = self._arm.set_position(*[200.0, -100.0, 200.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'): return False

        code = self._arm.set_position(*[target_x, target_y, 50.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        code = self._arm.set_position(*[target_x, target_y, 10.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.8)

        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'): return False
        time.sleep(1.2)

        code = self._arm.set_position(*[target_x, target_y, 50.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        code = self._arm.set_position(*[self.ASSEMBLY_X, self.ASSEMBLY_Y, 190.0, 180.0, 0.0, 90.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        code = self._arm.set_position(*[self.ASSEMBLY_X, self.ASSEMBLY_Y, self.Z_TAPA_BANDA2, 180.0, 0.0, 90.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False
        time.sleep(0.8)

        code = self._arm.set_position(*[self.ASSEMBLY_X, self.ASSEMBLY_Y, 190.0, 180.0, 0.0, 90.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        code = self._arm.set_position(*[100.0, 243.4, 193.3, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)
        return True

    def _paletizar_pastel(self, flavor):
        grid_idx           = self.global_pallet_count % 4
        piso               = 1 if self.global_pallet_count < 4 else 2
        target_x, target_y = self.PALLET_POSITIONS[grid_idx]
        target_z           = self.PALLET_Z_PISO1 if piso == 1 else self.PALLET_Z_PISO2

        print(f"[PASO G] Pastel {self.global_pallet_count+1}/8 → "
              f"Espacio {grid_idx+1}/4, Piso {piso}, "
              f"destino=({target_x}, {target_y}, {target_z})")

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False

        code = self._arm.set_position(*[self.ASSEMBLY_X, self.ASSEMBLY_Y, 150.0, 180.0, 0.0, 90.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        code = self._arm.set_position(*[self.ASSEMBLY_X, self.ASSEMBLY_Y, self.Z_PICK_PASTEL, 180.0, 0.0, 90.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.8)

        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'): return False
        time.sleep(1.2)

        code = self._arm.set_position(*[self.ASSEMBLY_X, self.ASSEMBLY_Y, 150.0, 180.0, 0.0, 90.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        code = self._arm.set_position(*[target_x, target_y, 150.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        code = self._arm.set_position(*[target_x, target_y, target_z, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False
        time.sleep(0.8)

        code = self._arm.set_position(*[target_x, target_y, 150.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        try:
            if flavor == 'choco':
                print("[PASO G] Enviando bus binario 010 (Chocolate) al PLC...")
                self._arm.set_cgpio_digital(1, 0); self._arm.set_cgpio_digital(2, 1); self._arm.set_cgpio_digital(3, 0)
            elif flavor == 'vainilla':
                print("[PASO G] Enviando bus binario 011 (Vainilla) al PLC...")
                self._arm.set_cgpio_digital(1, 1); self._arm.set_cgpio_digital(2, 1); self._arm.set_cgpio_digital(3, 0)
            elif flavor == 'straw':
                print("[PASO G] Enviando bus binario 100 (Fresa) al PLC...")
                self._arm.set_cgpio_digital(1, 0); self._arm.set_cgpio_digital(2, 0); self._arm.set_cgpio_digital(3, 1)
            time.sleep(0.7)
            self._arm.set_cgpio_digital(1, 0); self._arm.set_cgpio_digital(2, 0); self._arm.set_cgpio_digital(3, 0)
        except Exception as e:
            self.pprint(f'GPIO Pallet output warning: {e}')

        code = self._arm.set_position(*[100.0, 243.4, 193.3, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        self.global_pallet_count += 1
        return True

    def _colocar_tapa_separadora(self, es_tapa_final=False):
        z_pick  = 33.0 if es_tapa_final else 80.0
        z_place = 82.0 if es_tapa_final else 40.0
        label   = "TAPA FINAL" if es_tapa_final else "TAPA INTERMEDIA"
        print(f"[TAPA SEP] Colocando {label} — pick Z={z_pick}, place Z={z_place}")

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False

        code = self._arm.set_position(*[280.0, 270.0, 150.0, 180.0, 0.0, -45.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        code = self._arm.set_position(*[280.0, 270.0, z_pick, 180.0, 0.0, -45.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.8)

        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'): return False
        time.sleep(1.2)

        code = self._arm.set_position(*[280.0, 270.0, 150.0, 180.0, 0.0, -45.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        code = self._arm.set_position(*[165.0, 260.0, 150.0, 180.0, 0.0, -45.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        code = self._arm.set_position(*[165.0, 260.0, z_place, 180.0, 0.0, -45.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False
        time.sleep(0.8)

        code = self._arm.set_position(*[165.0, 260.0, 150.0, 180.0, 0.0, -45.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        code = self._arm.set_position(*[100.0, 243.4, 193.3, 180.0, 0.0, -45.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        return True

    def _active_count(self):
        return len(self.active_tasks)

    def _tasks_in_state(self, state):
        return [t for t in self.active_tasks if t.state == state]

    def _purge_completed(self):
        before = len(self.active_tasks)
        self.active_tasks = [t for t in self.active_tasks if t.state != 6]
        removed = before - len(self.active_tasks)
        if removed:
            print(f"[GESTIÓN] {removed} tarea(s) completada(s) liberada(s). "
                  f"Slots activos: {len(self.active_tasks)}/{MAX_CONCURRENT}")

    def run(self):
        self._diagnostico_inicial()

        print("Sincronizando estado inicial con el brazo...")
        for i in range(50):
            if self._arm.connected and self._arm.state < 4:
                print(f"   Brazo listo en iteración {i} (state={self._arm.state})")
                break
            time.sleep(0.1)
        else:
            print(f"   ADVERTENCIA: brazo no listo tras 5 s "
                  f"(connected={self._arm.connected}, state={self._arm.state})")

        if not self.is_alive:
            self.pprint("is_alive=False al entrar a run(). Abortando.")
            return

        try:
            print("Sistema listo. Iniciando loop de pipelining (máx. "
                  f"{MAX_CONCURRENT} pasteles simultáneos)...")

            while self.is_alive:
                self._purge_completed()

                # --- CAMBIO CLAVE PARA PROBLEMA B ---
                # Este bloque se ejecuta SIEMPRE, independientemente de si hay 
                # piezas en la banda o de lo que esté haciendo el robot.
                # Garantiza que el ACK del PLC se capture en cuanto llegue.
                for task in self._tasks_in_state(2):
                    if not task.lid_ack_received:
                        if self._detect_plc_ack(task.flavor):
                            task.lid_ack_received = True
                            task.state = 3
                            print(f"[ACK PLC] Tarea {task.id} ({task.flavor}): tapa confirmada → state=3")
                        elif time.monotonic() > task.plc_deadline:
                            self.pprint(f"[TIMEOUT PLC] Tarea {task.id} ({task.flavor}) — sin ACK en {PLC_TIMEOUT}s. Abortando.")
                            return

                ci3_activo = self._gpio_match([1, 1, 1, 0, 1, 1, 1, 1])
                
                # Verificación de timeout de CI3 (sin bloquear)
                for task in self._tasks_in_state(1):
                    if not ci3_activo and time.monotonic() > task.ci3_deadline:
                        self.pprint(f"[TIMEOUT CI3] Tarea {task.id} ({task.flavor}) — CI3 no detectado en {CI3_TIMEOUT}s. Abortando.")
                        return

                action_taken = False

                # PRIORIDAD 1
                ready_to_pallet = self._tasks_in_state(4)
                if not action_taken and ready_to_pallet:
                    task = ready_to_pallet[0]
                    print(f"\n[P1] Paletizando tarea {task.id} ({task.flavor})")
                    if not self._paletizar_pastel(task.flavor): return
                    if self.global_pallet_count == 4:
                        if not self._colocar_tapa_separadora(es_tapa_final=False): return
                    elif self.global_pallet_count == 8:
                        if not self._colocar_tapa_separadora(es_tapa_final=True): return
                        self.global_pallet_count = 0
                    task.state = 6
                    action_taken = True

                # PRIORIDAD 2
                waiting_ci3 = self._tasks_in_state(1)
                if not action_taken and waiting_ci3 and ci3_activo:
                    task = waiting_ci3[0]
                    print(f"\n[P2] CI3 activo — trasladando tarea {task.id} ({task.flavor})")
                    if not self._conveyor_to_conveyor_transfer(): return
                    self._send_co1()
                    task.state = 2
                    task.plc_deadline = time.monotonic() + PLC_TIMEOUT
                    print(f"[P2] Tarea {task.id} → state=2, esperando ACK PLC (timeout {PLC_TIMEOUT}s)")
                    action_taken = True

                # PRIORIDAD 3
                ready_to_assemble = self._tasks_in_state(3)
                if not action_taken and ready_to_assemble:
                    task = ready_to_assemble[0]
                    print(f"\n[P3] Ensamblando tapa tarea {task.id} ({task.flavor})")
                    if not self._pick_place_2nd(task.flavor, task.idx): return
                    
                    # --- CAMBIO CLAVE ELIMINADO DE AQUÍ ---
                    # (Ya no actualizamos cake_count aquí, se movió a Prioridad 4)
                    
                    task.state = 4
                    print(f"[P3] Tarea {task.id} → state=4, lista para paletizar")
                    action_taken = True

                # PRIORIDAD 4
                if not action_taken and self._active_count() < MAX_CONCURRENT:
                    flavor = self._detect_flavor()
                    if flavor is not None:
                        idx = self.cake_count[flavor]
                        
                        already_in_progress = any(t.flavor == flavor and t.idx == idx for t in self.active_tasks)
                        
                        if not already_in_progress:
                            print(f"\n[P4] Nueva tarea: {flavor} col={idx+1}, palet={self.global_pallet_count+1}/8")
                            new_task = CakeTask(flavor, idx)
                            
                            if not self._pick_place_1st(flavor, idx):
                                return
                            
                            # --- CAMBIO CLAVE PARA PROBLEMA A ---
                            # ¡ÉXITO! La base salió de la gradilla. 
                            # Actualizamos el índice INMEDIATAMENTE para que la 
                            # siguiente pieza del mismo sabor use la siguiente columna.
                            self.cake_count[flavor] = (idx + 1) % 3

                            new_task.state = 1
                            new_task.ci3_deadline = time.monotonic() + CI3_TIMEOUT
                            self.active_tasks.append(new_task)
                            print(f"[P4] Tarea {new_task.id} registrada → state=1, esperando CI3. "
                                  f"Tareas activas: {self._active_count()}/{MAX_CONCURRENT}")
                            action_taken = True

                if not action_taken:
                    time.sleep(0.01)

        except Exception as e:
            self.pprint(f"Excepción en el loop de ejecución: {e}")
            traceback.print_exc()
        finally:
            self.alive = False
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
            self._arm.release_state_changed_callback(self._state_changed_callback)
            if hasattr(self._arm, 'release_count_changed_callback'):
                self._arm.release_count_changed_callback(self._count_changed_callback)


if __name__ == '__main__':
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    IP_ROBOT = '192.168.1.168'
    print(f"Conectando directamente al xArm en: {IP_ROBOT}...")
    arm = XArmAPI(IP_ROBOT, baud_checkset=False)
    robot_main = RobotMain(arm)
    robot_main.run()
