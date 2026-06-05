#!/usr/bin/env python3

import sys
import math
import time
import traceback
from xarm import version
from xarm.wrapper import XArmAPI

# =============================================================================
# TASK STATE MACHINE DEFINITIONS
# =============================================================================
# 0 — Idle / Available slot
# 1 — Base deposited on Conveyor 5 (Non-blocking wait for CI3 sensor)
# 2 — Transferred to Conveyor 6, CO1 sent (Non-blocking wait for PLC ACK)
# 3 — PLC ACK received, ready to assemble lid (Waiting for robot availability)
# 4 — Lid assembled, ready for palletizing (Waiting for robot availability)
# 5 — Palletizing in progress (Transient state, does not persist between loops)
# 6 — Completed (Slot is marked for purging at the start of the next iteration)
# =============================================================================

MAX_CONCURRENT = 2          # Maximum number of cakes being processed simultaneously
CI3_TIMEOUT    = 120        # Seconds before aborting if Conveyor 5 end sensor (CI3) is not triggered
PLC_TIMEOUT    = 120        # Seconds before aborting if PLC lid confirmation (ACK) is not received


class CakeTask:
    """Represents a single cake moving through the production pipeline."""
    _next_id = 1

    def __init__(self, flavor: str, idx: int):
        self.id                = CakeTask._next_id
        CakeTask._next_id     += 1
        self.flavor            = flavor
        self.idx               = idx
        self.state             = 0
        self.lid_ack_received  = False
        # Timestamps for non-blocking timeout evaluation
        self.ci3_deadline      = None   # Set when entering state 1
        self.plc_deadline      = None   # Set when entering state 2

    def __repr__(self):
        return (f"<CakeTask id={self.id} flavor={self.flavor} "
                f"idx={self.idx} state={self.state} "
                f"lid_ack={self.lid_ack_received}>")


class RobotMain(object):
    """Main Robot Controller Class with concurrent pipelining support."""

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

        # -------------------------------------------------------------------------
        # PHYSICAL COORDINATE MAPPING
        # -------------------------------------------------------------------------
        # TRAY 1st FLOOR (Bases):  X columns = [5.0, 55.0, 105.0]
        # TRAY 2nd FLOOR (Lids):   X columns = [155.0, 205.0, 255.0]
        # FLAVOR ROWS (Y axis):    Choco = -200.0, Vanilla = -250.0, Strawberry = -300.0
        # ASSEMBLY STATION (Conv 6): X = 252.1, Y = -2.6
        # PALLET GRID 2x2:         (145, 280), (185, 280), (145, 240), (185, 240)
        # -------------------------------------------------------------------------
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
        self.Z_TAPA_BANDA2  = 71.4
        self.Z_PICK_PASTEL  = 55.0

        # Tracks the next available column (0, 1, or 2) in the tray for each flavor
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

        # List of active tasks currently in the pipeline
        self.active_tasks: list[CakeTask] = []
        
        self._robot_init()

    # =========================================================================
    # INITIALIZATION & CALLBACKS
    # =========================================================================
    def _robot_init(self):
        """Initializes the xArm, clears errors, and sets up GPIO/State callbacks."""
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
            # Initialize digital outputs to LOW (0)
            self._arm.set_cgpio_digital(1, 0)
            self._arm.set_cgpio_digital(2, 0)
            self._arm.set_cgpio_digital(3, 0)
        except Exception as e:
            self.pprint(f'GPIO init warning (non-critical): {e}')

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
        """Validates the return code of xArm SDK commands. Halts if an error occurs."""
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
        """Custom print function that includes timestamp and calling function name."""
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
        """Checks if the robot is connected, error-free, and in a valid operational state (< 4)."""
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

    # =========================================================================
    # GPIO LOGIC & SENSING
    # =========================================================================
    def _gpio_match(self, pattern, timeout=0.05):
        """
        Checks Custom GPIO (CI) inputs against a pattern.
        Pattern logic: 0 = Pin must be LOW (Active), 1 = Don't Care.
        Array maps to: [CI0, CI1, CI2, CI3, CI4, CI5, CI6, CI7]
        """
        return self._arm.arm.get_cgpio_li_state(pattern, timeout=timeout, is_ci=True)

    def _detect_flavor(self):
        """
        Detects the requested cake flavor from PLC inputs.
        Order matters: 'straw' is checked first because it activates both CI1 and CI2, 
        which are subsets of the 'choco' and 'vainilla' patterns.
        """
        if self._gpio_match([1, 0, 0, 1, 1, 1, 1, 1]): # CI1=LOW, CI2=LOW
            return 'straw'
        if self._gpio_match([1, 0, 1, 1, 1, 1, 1, 1]): # CI1=LOW
            return 'choco'
        if self._gpio_match([1, 1, 0, 1, 1, 1, 1, 1]): # CI2=LOW
            return 'vainilla'
        return None

    def _detect_plc_ack(self, flavor):
        """
        Detects PLC confirmation that the lid is ready.
        The PLC reuses the flavor pins (CI1/CI2) and adds CI3=LOW as an ACK bit.
        This allows the robot to distinguish a "new order" from a "lid ready" signal.
        """
        if flavor == 'choco'    and self._gpio_match([1, 0, 1, 0, 1, 1, 1, 1]): # CI1=LOW, CI3=LOW
            return True
        if flavor == 'vainilla' and self._gpio_match([1, 1, 0, 0, 1, 1, 1, 1]): # CI2=LOW, CI3=LOW
            return True
        if flavor == 'straw'    and self._gpio_match([1, 0, 0, 0, 1, 1, 1, 1]): # CI1, CI2, CI3=LOW
            return True
        return False

    def _diagnostico_inicial(self):
        """Prints initial system status and raw state of the first 4 CI inputs."""
        print("=" * 55)
        print("INITIAL DIAGNOSTICS")
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
            print(f"   CI{i} = {'LOW (active)' if estado else 'HIGH / inactive'}")
        print("=" * 55)

    # =========================================================================
    # MOTION SEQUENCES (STEPS)
    # =========================================================================
    def _pick_place_1st(self, flavor, idx):
        """STEP A: Pick base from 1st floor of tray and deposit on the Cell."""
        target_x = self.X_1ST[idx]
        target_y = self.Y_FLAVOR[flavor]
        print(f"[STEP A] {flavor} col={idx+1}  x={target_x}, y={target_y}")

        self._arm.set_lite6_gripper_enable(True)
        time.sleep(0.2)
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False

        # Approach waypoint
        code = self._arm.set_position(*[200.0, -100.0, 200.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'): return False

        # Hover over tray
        code = self._arm.set_position(*[target_x, target_y, 50.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        # Descend to pick
        code = self._arm.set_position(*[target_x, target_y, 10.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.8)

        # PROTECTION AGAINST HANGS: Enable and stabilize before closing
        self._arm.set_lite6_gripper_enable(True)
        time.sleep(0.2)
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'): return False
        time.sleep(1.2)

        # Retract vertically
        code = self._arm.set_position(*[target_x, target_y, 50.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        # Clear tray area waypoints
        code = self._arm.set_position(*[200.0, -100.0, 200.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'): return False
        code = self._arm.set_position(*[200.0, 110.0, 200.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'): return False

        # Approach drop zone
        code = self._arm.set_position(*[-37.8, 243.4, 193.3, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        # Drop
        code = self._arm.set_position(*[-37.8, 243.4, self.Z_PLACE_1ST, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        self._arm.set_lite6_gripper_enable(True)
        time.sleep(0.2)
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False
        time.sleep(0.8)

        # Retract and move to neutral waiting position
        code = self._arm.set_position(*[-37.8, 243.4, 193.3, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        code = self._arm.set_position(*[100.0, 243.4, 193.3, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)
        return True

    def _conveyor_to_conveyor_transfer(self):
        """STEP C: Transfer cake from Conveyor 5 to Assembly Station on Conveyor 6 (BLOCKING)."""
        print("[STEP C] Transferring piece Conveyor 5 → Conveyor 6...")
        
        self._arm.set_lite6_gripper_enable(True)
        time.sleep(0.2)
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

        self._arm.set_lite6_gripper_enable(True)
        time.sleep(0.2)
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

        self._arm.set_lite6_gripper_enable(True)
        time.sleep(0.2)
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False
        time.sleep(1.0)

        code = self._arm.set_position(*[self.ASSEMBLY_X, self.ASSEMBLY_Y, 86.5, 180.0, 0.0, 90.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        return True

    def _send_co1(self):
        """STEP D: Send a 0.5s pulse on CO1 to inform the PLC the base is ready on Conveyor 6."""
        print("[STEP D] Sending CO1=1 to PLC (base ready on Conveyor 6)...")
        try:
            self._arm.set_cgpio_digital(1, 1)
            self._arm.set_cgpio_digital(2, 0)
            self._arm.set_cgpio_digital(3, 0)
            time.sleep(1.5)
            self._arm.set_cgpio_digital(1, 0)
            self._arm.set_cgpio_digital(2, 0)
            self._arm.set_cgpio_digital(3, 0)
        except Exception as e:
            self.pprint(f'GPIO CO1 warning: {e}')

    def _pick_place_2nd(self, flavor, idx):
        """STEP F: Pick lid from 2nd floor of tray and assemble onto base on Conveyor 6 (BLOCKING)."""
        target_x = self.X_2ND[idx]
        target_y = self.Y_FLAVOR[flavor]
        print(f"[STEP F] {flavor} lid col={idx+1}  x={target_x}, y={target_y}")

        self._arm.set_lite6_gripper_enable(True)
        time.sleep(0.2)
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

        # PROTECTION AGAINST HANGS: Extra pause after descending to Z=10.0
        self._arm.set_lite6_gripper_enable(True)
        time.sleep(0.3)
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'): return False
        time.sleep(1.2)

        # Safe waypoint: retract vertically before moving horizontally to assembly
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

        self._arm.set_lite6_gripper_enable(True)
        time.sleep(0.2)
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False
        time.sleep(0.8)

        code = self._arm.set_position(*[self.ASSEMBLY_X, self.ASSEMBLY_Y, 150.0, 180.0, 0.0, 90.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        
        time.sleep(0.5)
        return True

    def _paletizar_pastel(self, flavor):
        """STEP G: Pick assembled cake from Conveyor 6 and place on 2x2 pallet grid (BLOCKING)."""
        grid_idx           = self.global_pallet_count % 4
        piso               = 1 if self.global_pallet_count < 4 else 2
        target_x, target_y = self.PALLET_POSITIONS[grid_idx]
        target_z           = self.PALLET_Z_PISO1 if piso == 1 else self.PALLET_Z_PISO2

        print(f"[STEP G] Cake {self.global_pallet_count+1}/8 → "
              f"Slot {grid_idx+1}/4, Floor {piso}, "
              f"dest=({target_x}, {target_y}, {target_z})")

        self._arm.set_lite6_gripper_enable(True)
        time.sleep(0.2)
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False

        code = self._arm.set_position(*[self.ASSEMBLY_X, self.ASSEMBLY_Y, 150.0, 180.0, 0.0, 90.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        code = self._arm.set_position(*[self.ASSEMBLY_X, self.ASSEMBLY_Y, self.Z_PICK_PASTEL, 180.0, 0.0, 90.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.8)

        self._arm.set_lite6_gripper_enable(True)
        time.sleep(0.2)
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'): return False
        time.sleep(1.2)

        # Safe transit height after grasping the cake
        code = self._arm.set_position(*[self.ASSEMBLY_X, self.ASSEMBLY_Y, 150.0, 180.0, 0.0, 90.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        # Horizontal transit to pallet column at safe height
        code = self._arm.set_position(*[target_x, target_y, 150.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        # Descend to final placement height
        code = self._arm.set_position(*[target_x, target_y, target_z, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        self._arm.set_lite6_gripper_enable(True)
        time.sleep(0.2)
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False
        time.sleep(0.8)

        # Retract vertically before leaving the pallet zone
        code = self._arm.set_position(*[target_x, target_y, 150.0, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        # Send binary flavor code to PLC
        try:
            if flavor == 'choco':
                print("[STEP G] Sending binary bus 010 (Chocolate) to PLC...")
                self._arm.set_cgpio_digital(1, 0)
                self._arm.set_cgpio_digital(2, 1)
                self._arm.set_cgpio_digital(3, 0)
            elif flavor == 'vainilla':
                print("[STEP G] Sending binary bus 011 (Vanilla) to PLC...")
                self._arm.set_cgpio_digital(1, 1)
                self._arm.set_cgpio_digital(2, 1)
                self._arm.set_cgpio_digital(3, 0)
            elif flavor == 'straw':
                print("[STEP G] Sending binary bus 100 (Strawberry) to PLC...")
                self._arm.set_cgpio_digital(1, 0)
                self._arm.set_cgpio_digital(2, 0)
                self._arm.set_cgpio_digital(3, 1)
            
            time.sleep(1.5) 
            
            # Reset all outputs to 0
            self._arm.set_cgpio_digital(1, 0)
            self._arm.set_cgpio_digital(2, 0)
            self._arm.set_cgpio_digital(3, 0)
            
        except Exception as e:
            self.pprint(f'GPIO Pallet output warning: {e}')

        code = self._arm.set_position(*[100.0, 243.4, 193.3, 180.0, 0.0, -45.0],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False

        self.global_pallet_count += 1
        return True

    def _colocar_tapa_separadora(self, es_tapa_final=False):
        """STEP G.1: Place intermediate or final separator lid on the pallet (BLOCKING)."""
        z_pick  = 33.0 if es_tapa_final else 80.0
        z_place = 82.0 if es_tapa_final else 40.0
        label   = "FINAL LID" if es_tapa_final else "INTERMEDIATE LID"
        print(f"[SEP LID] Placing {label} — pick Z={z_pick}, place Z={z_place}")

        self._arm.set_lite6_gripper_enable(True)
        time.sleep(0.2)
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False

        # Safe pick approach
        code = self._arm.set_position(*[280.0, 270.0, 150.0, 180.0, 0.0, -45.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        code = self._arm.set_position(*[280.0, 270.0, z_pick, 180.0, 0.0, -45.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.8)

        self._arm.set_lite6_gripper_enable(True)
        time.sleep(0.2)
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'): return False
        time.sleep(1.2)

        # Safe pick retract
        code = self._arm.set_position(*[280.0, 270.0, 150.0, 180.0, 0.0, -45.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        # Safe place approach
        code = self._arm.set_position(*[165.0, 260.0, 150.0, 180.0, 0.0, -45.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        code = self._arm.set_position(*[165.0, 260.0, z_place, 180.0, 0.0, -45.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        self._arm.set_lite6_gripper_enable(True)
        time.sleep(0.2)
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'): return False
        time.sleep(0.8)

        # Safe place retract
        code = self._arm.set_position(*[165.0, 260.0, 150.0, 180.0, 0.0, -45.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        time.sleep(0.5)

        code = self._arm.set_position(*[100.0, 243.4, 193.3, 180.0, 0.0, -45.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'): return False
        return True

    # =========================================================================
    # TASK MANAGEMENT HELPERS
    # =========================================================================
    def _active_count(self):
        return len(self.active_tasks)

    def _tasks_in_state(self, state):
        return [t for t in self.active_tasks if t.state == state]

    def _purge_completed(self):
        """Removes tasks in state 6 (Completed) to free up pipeline slots."""
        before = len(self.active_tasks)
        self.active_tasks = [t for t in self.active_tasks if t.state != 6]
        removed = before - len(self.active_tasks)
        if removed:
            print(f"[MGMT] {removed} completed task(s) purged. "
                  f"Active slots: {len(self.active_tasks)}/{MAX_CONCURRENT}")

    # =========================================================================
    # MAIN EXECUTION LOOP
    # =========================================================================
    def run(self):
        self._diagnostico_inicial()

        print("Synchronizing initial state with the arm...")
        for i in range(50):
            if self._arm.connected and self._arm.state < 4:
                print(f"   Arm ready at iteration {i} (state={self._arm.state})")
                break
            time.sleep(0.1)
        else:
            print(f"   WARNING: Arm not ready after 5s "
                  f"(connected={self._arm.connected}, state={self._arm.state})")

        if not self.is_alive:
            self.pprint("is_alive=False upon entering run(). Aborting.")
            return

        try:
            print("System ready. Starting pipelining loop (max. "
                  f"{MAX_CONCURRENT} concurrent cakes)...")

            while self.is_alive:
                # 1. PRE-ITERATION: Purge completed tasks to free slots
                self._purge_completed()

                # 2. NON-BLOCKING PLC ACK POLLING (State 2)
                # This runs every loop iteration, ensuring ACK is captured immediately 
                # regardless of what physical action the robot is currently performing.
                for task in self._tasks_in_state(2):
                    if not task.lid_ack_received:
                        if self._detect_plc_ack(task.flavor):
                            task.lid_ack_received = True
                            task.state = 3
                            print(f"[PLC ACK] Task {task.id} ({task.flavor}): lid confirmed → state=3")
                        elif time.monotonic() > task.plc_deadline:
                            self.pprint(f"[TIMEOUT PLC] Task {task.id} ({task.flavor}) — no ACK in {PLC_TIMEOUT}s. Aborting.")
                            return

                # 3. NON-BLOCKING CI3 TIMEOUT CHECK (State 1)
                ci3_activo = self._gpio_match([1, 1, 1, 0, 1, 1, 1, 1])
                for task in self._tasks_in_state(1):
                    if not ci3_activo and time.monotonic() > task.ci3_deadline:
                        self.pprint(f"[TIMEOUT CI3] Task {task.id} ({task.flavor}) — CI3 not detected in {CI3_TIMEOUT}s. Aborting.")
                        return

                # 4. PRIORITY EVALUATION
                # The robot executes only ONE blocking physical action per loop iteration.
                action_taken = False

                # PRIORITY 1: Clear the assembly station (Palletize)
                # Highest priority to prevent Conveyor 6 from jamming.
                ready_to_pallet = self._tasks_in_state(4)
                if not action_taken and ready_to_pallet:
                    task = ready_to_pallet[0]
                    print(f"\n[P1] Palletizing task {task.id} ({task.flavor})")
                    if not self._paletizar_pastel(task.flavor): return
                    
                    # Handle separator lids every 4 and 8 cakes
                    if self.global_pallet_count == 4:
                        if not self._colocar_tapa_separadora(es_tapa_final=False): return
                    elif self.global_pallet_count == 8:
                        if not self._colocar_tapa_separadora(es_tapa_final=True): return
                        self.global_pallet_count = 0
                        
                    task.state = 6
                    action_taken = True

                # PRIORITY 2: Prevent Conveyor 5 jam (Transfer)
                if not action_taken and self._tasks_in_state(1) and ci3_activo:
                    task = self._tasks_in_state(1)[0]
                    print(f"\n[P2] CI3 active — transferring task {task.id} ({task.flavor})")
                    if not self._conveyor_to_conveyor_transfer(): return
                    self._send_co1()
                    task.state = 2
                    task.plc_deadline = time.monotonic() + PLC_TIMEOUT
                    print(f"[P2] Task {task.id} → state=2, awaiting PLC ACK (timeout {PLC_TIMEOUT}s)")
                    action_taken = True

                # PRIORITY 3: Assemble pending lid
                if not action_taken and self._tasks_in_state(3):
                    task = self._tasks_in_state(3)[0]
                    print(f"\n[P3] Assembling lid for task {task.id} ({task.flavor})")
                    if not self._pick_place_2nd(task.flavor, task.idx): return
                    
                    task.state = 4
                    print(f"[P3] Task {task.id} → state=4, ready for palletizing")
                    action_taken = True

                # PRIORITY 4: Initiate new production
                if not action_taken and self._active_count() < MAX_CONCURRENT:
                    flavor = self._detect_flavor()
                    if flavor is not None:
                        idx = self.cake_count[flavor]
                        
                        # SAFETY CHECK: Prevent requesting the same tray column twice 
                        # if the PLC sends a new signal before the previous task advances.
                        already_in_progress = any(t.flavor == flavor and t.idx == idx for t in self.active_tasks)
                        
                        if not already_in_progress:
                            print(f"\n[P4] New task: {flavor} col={idx+1}, pallet={self.global_pallet_count+1}/8")
                            new_task = CakeTask(flavor, idx)
                            
                            if not self._pick_place_1st(flavor, idx):
                                return
                            
                            # CRITICAL LOGIC: Update the tray column index IMMEDIATELY 
                            # upon successful pick. This reserves the next column for 
                            # any subsequent PLC requests of the same flavor, preventing 
                            # the robot from trying to pick from an already-empty slot.
                            self.cake_count[flavor] = (idx + 1) % 3

                            new_task.state = 1
                            new_task.ci3_deadline = time.monotonic() + CI3_TIMEOUT
                            self.active_tasks.append(new_task)
                            print(f"[P4] Task {new_task.id} registered → state=1, awaiting CI3. "
                                  f"Active tasks: {self._active_count()}/{MAX_CONCURRENT}")
                            action_taken = True

                # 5. IDLE: Brief pause to prevent GPIO bus saturation when no action is taken
                if not action_taken:
                    time.sleep(0.01)

        except Exception as e:
            self.pprint(f"Exception in execution loop: {e}")
            traceback.print_exc()
        finally:
            self.alive = False
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
            self._arm.release_state_changed_callback(self._state_changed_callback)
            if hasattr(self._arm, 'release_count_changed_callback'):
                self._arm.release_count_changed_callback(self._count_changed_callback)


# =============================================================================
# ENTRY POINT
# =============================================================================
if __name__ == '__main__':
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    IP_ROBOT = '192.168.1.168'
    print(f"Connecting directly to xArm at: {IP_ROBOT}...")
    arm = XArmAPI(IP_ROBOT, baud_checkset=False)
    robot_main = RobotMain(arm)
    robot_main.run()
