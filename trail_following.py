# --------------------------------------------------------------
#  ENCODER-SYNCED VERSION
# --------------------------------------------------------------
from machine import Pin, Timer
from time import sleep_ms, ticks_ms
from math import pi

# ----- Drivers -------------------------------------------------
from motor_driver import MotorDriver
from encoded_motor_driver import EncodedMotorDriver   # <-- fixed version

# ----- Specs ---------------------------------------------------
WHEEL_RADIUS = 0.023          # m
CPR = 28
GEAR_RATIO = 98.5
COUNTS_PER_WHEEL_REV = CPR * GEAR_RATIO          # 2758
DIST_PER_COUNT = (2 * pi * WHEEL_RADIUS) / COUNTS_PER_WHEEL_REV
AXLE_LENGTH = 0.14               # meters – distance between wheel centers



T1_DIST = 0.75
T1_COUNTS = int(T1_DIST / DIST_PER_COUNT)        # ~14312
T2_DIST = 0.52
T2_COUNTS = int(T2_DIST / DIST_PER_COUNT)

# ----- Turn geometry ------------------------------------------------
WHEEL_CIRC = 2 * pi * WHEEL_RADIUS
ARC_ONE_REV = pi * AXLE_LENGTH                     # distance one wheel travels for 360° robot turn
TURN_COUNTS_ONE_REV = int(ARC_ONE_REV / DIST_PER_COUNT)   # approximately 8370

# ----- Pinout --------------------------------------------------
PWMA, AIN1, AIN2 = 16, 18, 17   # LEFT
PWMB, BIN1, BIN2 = 15, 13, 14   # RIGHT
EA_L, EB_L = 10, 11
EA_R, EB_R = 19, 20
STBY = Pin(12, Pin.OUT)
STBY.value(1)

R = Pin(28, Pin.OUT); G = Pin(27, Pin.OUT); B = Pin(26, Pin.OUT)

# ----- Wheels --------------------------------------------------
right  = EncodedMotorDriver((PWMA, AIN2, AIN1), (EB_L, EA_L))
left = EncodedMotorDriver((PWMB, BIN1, BIN2), (EA_R, EB_R))

# ----- LED helpers ---------------------------------------------
def led_red():   R.value(1); G.value(0); B.value(0)
def led_green(): R.value(0); G.value(1); B.value(0)
def led_off():   R.value(0); G.value(0); B.value(0)

# ----- Encoder helpers -----------------------------------------
def reset_encoders():
    left.reset_encoder_counts()
    right.reset_encoder_counts()
    left.prev_counts = left.encoder_counts
    right.prev_counts = right.encoder_counts

def avg_counts():
    return (left.encoder_counts + right.encoder_counts) // 2

def stop():
    left.stop()
    right.stop()

# ----- Global sync variables -----------------------------------
SYNC_TIMER = Timer(-1)
SYNC_ACTIVE = False
BASE_DUTY = 0.0
KP = 0.8                    # proportional gain – tweak if needed
SYNC_INTERVAL_MS = 10         # 100 Hz sync

# ----------------------------------------------------------------
#  Sync callback – runs in the background while the robot moves
# ----------------------------------------------------------------
# ----- SCALING CONSTANTS -----
LEFT_SCALE  = 1.0  # ← TUNE THIS
RIGHT_SCALE = 1.04

def _sync_callback(_):
    """Runs at 100 Hz – keeps wheels synced AND applies per-wheel scaling."""
    if not SYNC_ACTIVE:
        return

    l_cnt = left.encoder_counts
    r_cnt = right.encoder_counts

    # Proportional error: positive → left ahead
    error = l_cnt - r_cnt
    correction = KP * error

    # APPLY SCALING + CORRECTION
    left_duty  = max(min((BASE_DUTY * LEFT_SCALE)  - correction, 1.0), -1.0)
    right_duty = max(min((BASE_DUTY * RIGHT_SCALE) + correction, 1.0), -1.0)

    # Drive motors independently
    left.forward(abs(left_duty))
    right.forward(abs(right_duty))
    
    

# ----------------------------------------------------------------
#  Helper: start/stop the sync timer
# ----------------------------------------------------------------
def _start_sync(duty):
    global SYNC_ACTIVE, BASE_DUTY
    BASE_DUTY = duty
    SYNC_ACTIVE = True
    SYNC_TIMER.init(period=SYNC_INTERVAL_MS, mode=Timer.PERIODIC, callback=_sync_callback)

def _stop_sync():
    global SYNC_ACTIVE
    SYNC_ACTIVE = False
    SYNC_TIMER.deinit()
    stop()                     # make sure motors are off

# ----------------------------------------------------------------
#  Generic move-with-sync (used by all motion primitives)
# ----------------------------------------------------------------
def _move_with_sync(base_duty, target_counts, debug_modulo=500):
    """Move until average encoder counts reach *target_counts* while keeping wheels synced."""
    reset_encoders()
    _start_sync(base_duty)

    print(f"Target counts: {target_counts}")
    while True:
        avg = avg_counts()
        dist = avg * DIST_PER_COUNT

        if avg % debug_modulo == 0:
            print(f" L:{left.encoder_counts:6} R:{right.encoder_counts:6} avg:{avg:6} dist:{dist:.3f}m")

        if avg >= target_counts:
            _stop_sync()
            print(f"\nSTOP! counts={avg} distance={dist:.4f} m")
            led_green()
            break

    sleep_ms(200)
    stop()

# ----------------------------------------------------------------
#  Straight motions
# ----------------------------------------------------------------
SPEED = 0.65

def forward():
    left.forward(SPEED * LEFT_SCALE)   # ← forward, scaled
    right.forward(SPEED * RIGHT_SCALE) # ← forward, full power

    while True:
        avg = avg_counts()
        dist = avg * DIST_PER_COUNT

        # Debug: print every 500 counts
        if avg % 500 == 0:
            print(f" L:{left.encoder_counts:6} R:{right.encoder_counts:6} avg:{avg:6} dist:{dist:.3f}m")

        # Stop condition
        if avg >= T1_COUNTS:
            _stop_sync()
            print(f"\nSTOP! avg={avg} distance={dist:.4f} m")
            led_green()
            break

    sleep_ms(200)
    stop()

def diagnose_straight(duration_ms=3000):
    """
    Run both wheels forward at the same base speed (with scaling)
    for *duration_ms* and print the encoder balance.
    """
    print("\n=== DIAGNOSTIC: straight-line balance ===")
    led_red()
    reset_encoders()

    # ---- APPLY SCALING (same as in sync loop) ----
    left.forward(SPEED * LEFT_SCALE)   # ← forward, scaled
    right.forward(SPEED * RIGHT_SCALE) # ← forward, full power

    sleep_ms(duration_ms)
    stop()

    L = left.encoder_counts
    R = right.encoder_counts
    total = R + L
    diff  = R - L
    pct   = (diff / (total / 2)) * 100 if total else 0

    print(f"\n--- BALANCE REPORT ---")
    print(f"Left  counts : {L}")
    print(f"Right counts : {R}")
    print(f"Difference   : {diff:+}  ({pct:+.2f} %)")
    if diff > 50:
        print("Left wheel is FASTER → **decrease** LEFT_SCALE")
    elif diff < -50:
        print("Right wheel is FASTER → **increase** LEFT_SCALE")
    else:
        print("Perfect balance! Keep current LEFT_SCALE.")
    led_green()
    
    
def forward_2():
    left.forward(SPEED * LEFT_SCALE)   # ← forward, scaled
    right.forward(SPEED * RIGHT_SCALE) # ← forward, full power

    while True:
        avg = avg_counts()
        dist = avg * DIST_PER_COUNT

        # Debug: print every 500 counts
        if avg % 500 == 0:
            print(f" L:{left.encoder_counts:6} R:{right.encoder_counts:6} avg:{avg:6} dist:{dist:.3f}m")

        # Stop condition
        if avg >= T2_COUNTS:
            _stop_sync()
            print(f"\nSTOP! avg={avg} distance={dist:.4f} m")
            led_green()
            break

    sleep_ms(200)
    stop()




# ----- Turn scaling -------------------------------------------------
TURN_SPEED = 0.55                  # a bit slower than straight speed
LEFT_SCALE_TURN  = LEFT_SCALE      # keep the same per-wheel scaling
RIGHT_SCALE_TURN = RIGHT_SCALE

# ----------------------------------------------------------------
# Turn sync helpers – SPIN IN PLACE (one wheel forward, one backward)
# ----------------------------------------------------------------
def _turn_sync_left(duty):
    """Spin LEFT in place: right forward, left backward."""
    global SYNC_ACTIVE, BASE_DUTY
    BASE_DUTY = duty
    SYNC_ACTIVE = True
    def _cb(_):
        if not SYNC_ACTIVE: return
        left_duty  = -BASE_DUTY * LEFT_SCALE_TURN      # backward
        right_duty =  BASE_DUTY * RIGHT_SCALE_TURN     # forward
        # Clamp to [-1.0, 1.0]
        left_duty  = max(min(left_duty,  1.0), -1.0)
        right_duty = max(min(right_duty, 1.0), -1.0)
        # Preserve direction
        if left_duty >= 0:
            left.forward(abs(left_duty))
        else:
            left.backward(abs(left_duty))
        if right_duty >= 0:
            right.forward(abs(right_duty))
        else:
            right.backward(abs(right_duty))
    SYNC_TIMER.init(period=SYNC_INTERVAL_MS, mode=Timer.PERIODIC, callback=_cb)

def _turn_sync_right(duty):
    """Spin RIGHT in place: left forward, right backward."""
    global SYNC_ACTIVE, BASE_DUTY
    BASE_DUTY = duty
    SYNC_ACTIVE = True
    def _cb(_):
        if not SYNC_ACTIVE: return
        left_duty  =  BASE_DUTY * LEFT_SCALE_TURN      # forward
        right_duty = -BASE_DUTY * RIGHT_SCALE_TURN     # backward
        left_duty  = max(min(left_duty,  1.0), -1.0)
        right_duty = max(min(right_duty, 1.0), -1.0)
        if left_duty >= 0:
            left.forward(abs(left_duty))
        else:
            left.backward(abs(left_duty))
        if right_duty >= 0:
            right.forward_duty(right_duty)
        else:
            right.backward(abs(right_duty))
    SYNC_TIMER.init(period=SYNC_INTERVAL_MS, mode=Timer.PERIODIC, callback=_cb)

# ----------------------------------------------------------------
# 90° LEFT SPIN (both wheels opposite)
# ----------------------------------------------------------------
def left_turn(debug_modulo=200):
    """Spin 90° left in place – opposite wheel directions."""
    # For spin-in-place, both wheels travel the same arc distance
    target = TURN_COUNTS_ONE_REV // 4.7
    print(f"\n=== SPIN LEFT 90° (target counts per wheel: {target}) ===")
    led_red()
    reset_encoders()
    _turn_sync_right(TURN_SPEED)
    while True:
        # Use average of absolute counts (both wheels move)
        l_cnt = abs(left.encoder_counts)
        r_cnt = abs(right.encoder_counts)
        avg = (l_cnt + r_cnt) // 2
        if avg % debug_modulo == 0:
            print(f" L:{l_cnt:6} R:{r_cnt:6} avg:{avg:6}  (spin left)")
        if avg >= target:
            _stop_sync()
            stop()                                      # force zero
            arc = avg * DIST_PER_COUNT
            print(f"\nLEFT 90° SPIN DONE – arc: {arc:.4f} m")
            led_green()
            break
    sleep_ms(200)
    stop()


def left_turn_2(debug_modulo=200):
    """Spin 90° left in place – opposite wheel directions."""
    # For spin-in-place, both wheels travel the same arc distance
    target = TURN_COUNTS_ONE_REV // 6.5
    print(f"\n=== SPIN LEFT 90° (target counts per wheel: {target}) ===")
    led_red()
    reset_encoders()
    _turn_sync_right(TURN_SPEED)
    while True:
        # Use average of absolute counts (both wheels move)
        l_cnt = abs(left.encoder_counts)
        r_cnt = abs(right.encoder_counts)
        avg = (l_cnt + r_cnt) // 2
        if avg % debug_modulo == 0:
            print(f" L:{l_cnt:6} R:{r_cnt:6} avg:{avg:6}  (spin left)")
        if avg >= target:
            _stop_sync()
            stop()                                      # force zero
            arc = avg * DIST_PER_COUNT
            print(f"\nLEFT 90° SPIN DONE – arc: {arc:.4f} m")
            led_green()
            break
    sleep_ms(200)
    stop()


# ----------------------------------------------------------------
# 270° RIGHT SPIN (both wheels opposite)
# ----------------------------------------------------------------
def right_turn(debug_modulo=200):
    """Spin 270° right in place – opposite wheel directions."""
    target = 3 * (TURN_COUNTS_ONE_REV // 4.7)
    print(f"\n=== SPIN RIGHT 270° (target counts per wheel: {target}) ===")
    led_red()
    reset_encoders()
    _turn_sync_left(TURN_SPEED)
    while True:
        l_cnt = abs(left.encoder_counts)
        r_cnt = abs(right.encoder_counts)
        avg = (l_cnt + r_cnt) // 2
        if avg % debug_modulo == 0:
            print(f" L:{l_cnt:6} R:{r_cnt:6} avg:{avg:6}  (spin right)")
        if avg >= target:
            _stop_sync()
            stop()                                      # force zero
            arc = avg * DIST_PER_COUNT
            print(f"\nRIGHT 270° SPIN DONE – arc: {arc:.4f} m")
            led_green()
            break
    sleep_ms(200)
    stop()


# ----------------------------------------------------------------
#  Main
# ----------------------------------------------------------------
if __name__ == "__main__":
    try:
        forward()
        left_turn()
        forward_2()
        right_turn()
        forward_2()
        left_turn_2()
        forward_2()
    except KeyboardInterrupt:
        _stop_sync()
        STBY.value(0)
        led_off()
        print("\nInterrupted.")
