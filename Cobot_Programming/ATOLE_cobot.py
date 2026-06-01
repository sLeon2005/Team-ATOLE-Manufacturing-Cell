#!/usr/bin/env python3

import sys
import math
import time
import queue
import datetime
import random
import traceback
import threading
from xarm import version
from xarm.wrapper import XArmAPI


class RobotMain(object):
    """Robot Main Class"""
    def __init__(self, robot, **kwargs):
        self.alive = True
        self._arm = robot
        self._ignore_exit_state = False
        self._tcp_speed = 100
        self._tcp_acc = 2000
        self._angle_speed = 20
        self._angle_acc = 500
        self._vars = {}
        self._funcs = {}

        # ---------------------------------------------------------------
        # POSICIONES GRADILLA DE ORIGEN POR SABOR
        # ---------------------------------------------------------------
        self.X_1ST = [5.0, 55.0, 105.0]
        self.X_2ND = [155.0, 205.0, 255.0]
        self.Y_FLAVOR = {
            'choco':    -200.0,
            'vainilla': -250.0,
            'straw':    -300.0,
        }

        self.Z_PLACE_1ST = 144.7

        # ---------------------------------------------------------------
        # ESTACIÓN DE ENSAMBLE EN BANDA 2
        # ---------------------------------------------------------------
        self.ASSEMBLY_X = 252.1
        self.ASSEMBLY_Y = -2.6
        self.Z_BASE_BANDA2  = 56.4
        self.Z_TAPA_BANDA2  = 68.4
        self.Z_PICK_PASTEL  = 55.0

        # Contador de origen de gradilla por sabor (0, 1 o 2)
        self.cake_count = {'choco': 0, 'vainilla': 0, 'straw': 0}

        # ---------------------------------------------------------------
        # CONFIGURACIÓN PALETIZADO 2x2 — DOS PISOS
        # ---------------------------------------------------------------
        self.PALLET_POSITIONS = [
            (145.0, 280.0),
            (185.0, 280.0),
            (145.0, 240.0),
            (185.0, 240.0),
        ]
        self.PALLET_Z_PISO1 = 10.0
        self.PALLET_Z_PISO2 = 45.0
        self.global_pallet_count = 0

        self._robot_init()

    # -------------------------------------------------------------------------
    # INICIALIZACIÓN
    # -------------------------------------------------------------------------
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

    # -------------------------------------------------------------------------
    # CALLBACKS
    # -------------------------------------------------------------------------
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
        else:
            return False

    # -------------------------------------------------------------------------
    # LECTURA GPIO — usa get_cgpio_li_state igual que la v1 funcional.
    #
    # El array de 8 posiciones corresponde a [CI0, CI1, CI2, CI3, CI4, CI5, CI6, CI7].
    # Valor 0 = "este pin debe estar en LOW para que la condición sea verdadera".
    # Valor 1 = "no me importa el estado de este pin".
    #
    # Patrones de sabor (mismos que la v1):
    #   choco    → CI1=LOW            → [1, 0, 1, 1, 1, 1, 1, 1]
    #   vainilla → CI2=LOW            → [1, 1, 0, 1, 1, 1, 1, 1]
    #   straw    → CI1=LOW y CI2=LOW  → [1, 0, 0, 1, 1, 1, 1, 1]
    #   CI3      → CI3=LOW            → [1, 1, 1, 0, 1, 1, 1, 1]
    #
    # ACK del PLC para tapa lista (patrones a confirmar con el equipo):
    #   tapa choco    → CI0=LOW            → [0, 1, 1, 1, 1, 1, 1, 1]
    #   tapa vainilla → CI0=LOW y CI1=LOW  → [0, 0, 1, 1, 1, 1, 1, 1]
    #   tapa straw    → CI0=LOW y CI2=LOW  → [0, 1, 0, 1, 1, 1, 1, 1]
    # -------------------------------------------------------------------------
    def _gpio_match(self, pattern, timeout=0.05):
        """
        Wrapper sobre get_cgpio_li_state para mantener la lógica centralizada.
        Devuelve True si los pines coinciden con el patrón dentro del timeout.
        """
        return self._arm.arm.get_cgpio_li_state(pattern, timeout=timeout, is_ci=True)

    # -------------------------------------------------------------------------
    # DIAGNÓSTICO
    # -------------------------------------------------------------------------
    def _diagnostico_inicial(self):
        print("=" * 55)
        print("DIAGNÓSTICO INICIAL")
        print(f"   connected   = {self._arm.connected}")
        print(f"   state       = {self._arm.state}")
        print(f"   error_code  = {self._arm.error_code}")
        print(f"   alive       = {self.alive}")
        print(f"   is_alive    = {self.is_alive}")
        print()
        # Imprime el estado raw de get_cgpio_li_state para cada pin
        for i in range(4):
            # Construir patrón que detecta pin i en LOW
            pat = [1] * 8
            pat[i] = 0
            estado = self._gpio_match(pat, timeout=0.02)
            print(f"   CI{i} = {'LOW (activo)' if estado else 'HIGH / no activo'}")
        print("=" * 55)

    # -------------------------------------------------------------------------
    # PASO A — PICK BASE (1er piso gradilla) → depositar en banda 1
    # -------------------------------------------------------------------------
    def _pick_place_1st(self, flavor, idx):
        target_x = self.X_1ST[idx]
        target_y = self.Y_FLAVOR[flavor]
        print(f"[1er piso] {flavor} — pastel {idx + 1}/3  x={target_x}, y={target_y}")

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

    # -------------------------------------------------------------------------
    # PASO C — TRASLADO ENTRE BANDAS (Banda 1 → Banda 2)
    # -------------------------------------------------------------------------
    def _conveyor_to_conveyor_transfer(self):
        print("Trasladando pieza de Banda 1 a Banda 2...")

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

    # -------------------------------------------------------------------------
    # PASO F — PICK TAPA (2do piso gradilla) → depositar sobre base en banda 2
    # -------------------------------------------------------------------------
    def _pick_place_2nd(self, flavor, idx):
        target_x = self.X_2ND[idx]
        target_y = self.Y_FLAVOR[flavor]
        print(f"[2do piso] {flavor} — tapa {idx + 1}/3  x={target_x}, y={target_y}")

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

    # -------------------------------------------------------------------------
    # PASO G — PALETIZADO: agarre en banda 2 → depósito en cuadrícula 2x2
    # -------------------------------------------------------------------------
    def _paletizar_pastel(self, flavor):
        grid_idx       = self.global_pallet_count % 4
        piso           = 1 if self.global_pallet_count < 4 else 2
        target_x, target_y = self.PALLET_POSITIONS[grid_idx]
        target_z       = self.PALLET_Z_PISO1 if piso == 1 else self.PALLET_Z_PISO2

        print(f"[PALETIZADO] Pastel {self.global_pallet_count + 1}/8 → "
              f"Espacio {grid_idx + 1}/4, Piso {piso}, "
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

        code = self._arm.set_position(*[self.ASSEMBLY_X, self.ASSEMBLY_Y, 240.0, 180.0, 0.0, 90.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        code = self._arm.set_position(*[target_x, target_y, 240.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        code = self._arm.set_position(*[target_x, target_y, target_z, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False
        time.sleep(0.8)

        # Envío de código binario al PLC según sabor
        try:
            if flavor == 'choco':
                print("Paletizado completo: Enviando bus binario 010 (Chocolate) al PLC...")
                self._arm.set_cgpio_digital(1, 0)
                self._arm.set_cgpio_digital(2, 1)
                self._arm.set_cgpio_digital(3, 0)
            elif flavor == 'vainilla':
                print("Paletizado completo: Enviando bus binario 011 (Vainilla) al PLC...")
                self._arm.set_cgpio_digital(1, 1)
                self._arm.set_cgpio_digital(2, 1)
                self._arm.set_cgpio_digital(3, 0)
            elif flavor == 'straw':
                print("Paletizado completo: Enviando bus binario 100 (Fresa) al PLC...")
                self._arm.set_cgpio_digital(1, 0)
                self._arm.set_cgpio_digital(2, 0)
                self._arm.set_cgpio_digital(3, 1)
            time.sleep(0.7)
            self._arm.set_cgpio_digital(1, 0)
            self._arm.set_cgpio_digital(2, 0)
            self._arm.set_cgpio_digital(3, 0)
        except Exception as e:
            self.pprint(f'GPIO Pallet output warning: {e}')

        code = self._arm.set_position(*[target_x, target_y, 240.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        code = self._arm.set_position(*[100.0, 243.4, 193.3, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        self.global_pallet_count += 1
        return True

    # -------------------------------------------------------------------------
    # PASO G.1 — TAPA SEPARADORA
    # -------------------------------------------------------------------------
    def _colocar_tapa_separadora(self, es_tapa_final=False):
        z_pick  = 33.0 if es_tapa_final else 80.0
        z_place = 82.0 if es_tapa_final else 40.0
        label   = "TAPA FINAL" if es_tapa_final else "TAPA INTERMEDIA"

        print(f"[TAPA] Colocando {label} — pick Z={z_pick}, place Z={z_place}")

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False

        code = self._arm.set_position(*[280.0, 270.0, 150.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        code = self._arm.set_position(*[280.0, 270.0, z_pick, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.8)

        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'): return False
        time.sleep(1.2)

        code = self._arm.set_position(*[280.0, 270.0, 150.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        code = self._arm.set_position(*[165.0, 260.0, 150.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        code = self._arm.set_position(*[165.0, 260.0, z_place, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False
        time.sleep(0.8)

        code = self._arm.set_position(*[165.0, 260.0, 150.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        code = self._arm.set_position(*[100.0, 243.4, 193.3, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        return True

    # -------------------------------------------------------------------------
    # LOOP PRINCIPAL
    # -------------------------------------------------------------------------
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
            print("Sistema listo. Esperando señales del PLC...")
            while self.is_alive:

                flavor = None

                # ----------------------------------------------------------------
                # DETECCIÓN DE SABOR — idéntica a la v1 funcional.
                # get_cgpio_li_state devuelve True cuando los pines marcados con 0
                # están en LOW dentro del timeout indicado.
                # ----------------------------------------------------------------
                if self._gpio_match([1, 0, 1, 1, 1, 1, 1, 1]):
                    flavor = 'choco'
                elif self._gpio_match([1, 1, 0, 1, 1, 1, 1, 1]):
                    flavor = 'vainilla'
                elif self._gpio_match([1, 0, 0, 1, 1, 1, 1, 1]):
                    flavor = 'straw'

                if flavor is not None:
                    idx = self.cake_count[flavor]
                    print(f"\n>>> Sabor: {flavor}, columna {idx + 1}/3, "
                          f"paleta {self.global_pallet_count + 1}/8")

                    # PASO A: pick base → banda 1
                    if not self._pick_place_1st(flavor, idx): return

                    # ----------------------------------------------------------------
                    # PASO B: esperar CI3 en LOW (fin de banda 1) — igual que la v1.
                    # Se agrega timeout de 60 s para no bloquear indefinidamente.
                    # ----------------------------------------------------------------
                    print("Esperando CI3 (fin de banda 1)...")
                    timeout_ci3 = time.monotonic() + 60.0
                    ci3_detectado = False
                    while self.is_alive and time.monotonic() < timeout_ci3:
                        if self._gpio_match([1, 1, 1, 0, 1, 1, 1, 1]):
                            print("CI3 activo (LOW). Iniciando traslado.")
                            ci3_detectado = True
                            break
                        time.sleep(0.05)

                    if not ci3_detectado:
                        self.pprint("Timeout esperando CI3 — abortando ciclo.")
                        return

                    # PASO C: traslado banda 1 → banda 2
                    if not self._conveyor_to_conveyor_transfer(): return

                    # PASO D: pulso CO1 al PLC (base depositada en banda 2)
                    print("Enviando CO1=1 al PLC (base lista en banda 2)...")
                    try:
                        self._arm.set_cgpio_digital(1, 1)
                        self._arm.set_cgpio_digital(2, 0)
                        self._arm.set_cgpio_digital(3, 0)
                        time.sleep(0.5)
                        self._arm.set_cgpio_digital(1, 0)
                        self._arm.set_cgpio_digital(2, 0)
                        self._arm.set_cgpio_digital(3, 0)
                    except Exception as e:
                        self.pprint(f'GPIO output warning: {e}')

                    # ----------------------------------------------------------------
                    # PASO E: esperar ACK del PLC (tapa lista).
                    # Los patrones de confirmación deben ser acordados con el PLC;
                    # se usan los mismos pines CI0-CI2 pero en contexto de respuesta.
                    # Timeout de 60 s para no bloquear si el PLC no responde.
                    # ----------------------------------------------------------------
                    print(f"Esperando confirmación PLC para tapa de {flavor}...")
                    plc_ready = False
                    timeout_plc = time.monotonic() + 60.0
                    while self.is_alive and not plc_ready and time.monotonic() < timeout_plc:
                        if flavor == 'choco'    and self._gpio_match([0, 1, 1, 1, 1, 1, 1, 1]):
                            print("PLC: tapa chocolate lista.")
                            plc_ready = True
                        elif flavor == 'vainilla' and self._gpio_match([0, 0, 1, 1, 1, 1, 1, 1]):
                            print("PLC: tapa vainilla lista.")
                            plc_ready = True
                        elif flavor == 'straw'    and self._gpio_match([0, 1, 0, 1, 1, 1, 1, 1]):
                            print("PLC: tapa fresa lista.")
                            plc_ready = True
                        time.sleep(0.05)

                    if not plc_ready:
                        self.pprint(f"Timeout esperando ACK del PLC para {flavor} — abortando ciclo.")
                        return

                    # PASO F: pick tapa → depositar sobre base en banda 2
                    if not self._pick_place_2nd(flavor, idx): return

                    # Avanzar contador de gradilla para este sabor
                    self.cake_count[flavor] = (idx + 1) % 3

                    # PASO G: paletizar el pastel ensamblado
                    if not self._paletizar_pastel(flavor): return

                    # Tapas separadoras por lote
                    if self.global_pallet_count == 4:
                        if not self._colocar_tapa_separadora(es_tapa_final=False): return
                    elif self.global_pallet_count == 8:
                        if not self._colocar_tapa_separadora(es_tapa_final=True): return
                        self.global_pallet_count = 0

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


# -----------------------------------------------------------------------------
# PUNTO DE ARRANQUE
# -----------------------------------------------------------------------------
if __name__ == '__main__':
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))

    IP_ROBOT = '192.168.1.168'
    print(f"Conectando directamente al xArm en: {IP_ROBOT}...")
    arm = XArmAPI(IP_ROBOT, baud_checkset=False)

    robot_main = RobotMain(arm)
    robot_main.run()
