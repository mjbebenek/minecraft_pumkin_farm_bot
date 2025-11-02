import minescript as ms  # type: ignore
import time
import math
import threading

# ================================================================
# KONFIGURACJA
# ================================================================
TARGET_BLOCK = "minecraft:pumpkin"
SCAN_SPEED_YAW = 2
SCAN_SPEED_PITCH = 5
PITCH_MAX = 45
PITCH_MIN = 10
LOOP_DELAY_SECONDS = 0.001
KEY_F7 = 296
KEY_F9 = 298

MAX_SCAN_DISTANCE = 8
APPROACH_DISTANCE = 1.5
EYE_HEIGHT = 1.62
PITCH_CORRECTION_FACTOR = 0.85
INVENTORY_CAPACITY = 41

STUCK_TIME = 2.0
MIN_DIST_CHANGE = 0.1
SWITCH_THRESHOLD = 0.4   # różnica dystansu do zmiany celu
DEBUG = False

# ================================================================
# STAN I SYNCHRONIZACJA
# ================================================================
bot_active = False
scanner_state = {"current_yaw": 0, "current_pitch": 0, "pitch_direction": "DOWN"}

scanning_allowed = threading.Event()
scanning_allowed.set()
destroy_thread_running = threading.Event()

# ================================================================
# FUNKCJE POMOCNICZE
# ================================================================
def debug(msg: str):
    if DEBUG:
        ms.echo(f"§7[DEBUG] {msg}")

def _normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def _raw_yaw_from_positions(player_pos, target_pos):
    px, _, pz = player_pos
    tx, _, tz = target_pos
    tx += 0.5
    tz += 0.5
    dx = tx - px
    dz = tz - pz
    yaw_rad = math.atan2(-dx, dz)
    return _normalize_angle(math.degrees(yaw_rad))

def _compute_yaw_to_target(player_pos, target_pos, current_yaw=None):
    desired = _raw_yaw_from_positions(player_pos, target_pos)
    if current_yaw is None:
        return desired
    cur = _normalize_angle(current_yaw)
    candidates = [desired - 360, desired, desired + 360]
    best = min(candidates, key=lambda d: abs(d - cur))
    return _normalize_angle(best)

def _get_pos_from_target(target):
    if not target:
        return None
    try:
        if hasattr(target, "position"):
            p = target.position
            return (p.x, p.y, p.z) if hasattr(p, "x") else tuple(p)
        elif isinstance(target, dict):
            return (target["x"], target["y"], target["z"])
    except Exception:
        return None
    return None

# ================================================================
# PLACEHOLDERS: DURABILITY / EKWIPUNEK
# ================================================================
def check_tool_durability():
    return False

def check_inventory_status():
    global bot_active
    try:
        inv = ms.player_inventory()
        if inv is None:
            return False
        if any(item is None for item in inv):
            total_slots = len(inv)
            filled = sum(1 for item in inv if item is not None)
            return filled >= total_slots
        if len(inv) >= INVENTORY_CAPACITY:
            for i in range(7):
                ms.execute("/wymiendynie")
            if check_inventory_status():
                bot_active = False
                stop_all_actions()
                destroy_thread_running.clear()
                scanning_allowed.set()
                ms.echo("§c[Inventory] Ekwipunnek pelny, zatrzymuje bota.")

    except Exception:
        return False

# ================================================================
# AKCJE BOTA
# ================================================================
def stop_all_actions():
    ms.player_press_forward(False)
    ms.player_press_attack(False)
    ms.player_press_backward(False)

def go_and_destroy_target():
    global bot_active

    try:
        ms.echo("§e[Skaner] Cel namierzony — podchodzę i niszczę...")
        scanning_allowed.clear()

        targeted = ms.player_get_targeted_block(max_distance=MAX_SCAN_DISTANCE)
        pos = _get_pos_from_target(targeted)
        if not pos:
            ms.echo("§c[Skaner] Nie udało się odczytać pozycji celu.")
            scanning_allowed.set()
            return
        tx, ty, tz = pos

        _, pitch_base = ms.player_orientation()
        last_dist = None
        last_move = time.time()

        # ------------------------------------------------------------
        # PODEJŚCIE DO CELU Z WYKRYWANIEM ZACIĘCIA
        # ------------------------------------------------------------
        while bot_active:
            if check_tool_durability() or check_inventory_status():
                bot_active = False
                stop_all_actions()
                break

            px, py, pz = ms.player_position()
            eye_y = py + EYE_HEIGHT
            bx, by, bz = tx + 0.5, ty + 0.5, tz + 0.5
            dx, dz, dy = bx - px, bz - pz, by - eye_y
            dist = math.hypot(dx, dz)

            now = time.time()
            if last_dist is not None:
                if abs(dist - last_dist) > MIN_DIST_CHANGE:
                    last_move = now
                elif now - last_move > STUCK_TIME:
                    ms.echo("§c[Skaner] Bot się zaciął — porzucam cel.")
                    stop_all_actions()
                    destroy_thread_running.clear()
                    scanning_allowed.set()
                    return

            last_dist = dist
            if dist <= APPROACH_DISTANCE:
                ms.player_press_forward(False)
                break

            cur_yaw, cur_pitch = ms.player_orientation()
            desired_yaw = _compute_yaw_to_target((px, py, pz), (tx, ty, tz), cur_yaw)
            pitch_target = pitch_base - math.degrees(math.atan2(dy, dist)) * PITCH_CORRECTION_FACTOR
            pitch_target = max(PITCH_MIN, min(PITCH_MAX, pitch_target))

            step = (pitch_target - cur_pitch) / 5.0
            for _ in range(5):
                cur_pitch += step
                ms.player_set_orientation(desired_yaw, cur_pitch)
                time.sleep(0.01)

            ms.player_press_forward(True)
            time.sleep(0.02)

        if not bot_active:
            stop_all_actions()
            scanning_allowed.set()
            return

        # ------------------------------------------------------------
        # ATAK Z NATYCHMIASTOWYM PRZEŁĄCZANIEM CELU
        # ------------------------------------------------------------
        ms.echo("§e[Skaner] Atakuję blok...")
        ms.player_press_attack(True)

        attack_start_time = time.time()
        ATTACK_TIMEOUT = 3.0

        while bot_active:
            if time.time() - attack_start_time > ATTACK_TIMEOUT:
                ms.echo("§c[Skaner] Timeout ataku — porzucam cel.")
                break

            px, py, pz = ms.player_position()

            # natychmiastowe przełączenie na bliższą dynię
            new_target = ms.player_get_targeted_block(max_distance=MAX_SCAN_DISTANCE)
            new_pos = _get_pos_from_target(new_target)
            if new_pos and getattr(new_target, "type", None) == TARGET_BLOCK:
                nx, ny, nz = new_pos
                new_horiz = math.hypot(nx + 0.5 - px, nz + 0.5 - pz)
                old_horiz = math.hypot(tx + 0.5 - px, tz + 0.5 - pz)
                if new_horiz + SWITCH_THRESHOLD < old_horiz:
                    ms.echo(f"§e[Skaner] Przełączenie na bliższą dynię ({new_horiz:.2f} < {old_horiz:.2f})")
                    tx, ty, tz = nx, ny, nz
                    desired_yaw = _compute_yaw_to_target((px, py, pz), (tx, ty, tz))
                    ms.player_set_orientation(desired_yaw, ms.player_orientation()[1])
                    attack_start_time = time.time()
                    continue

            # jeśli blok zniknął — przerwij
            current_check = ms.player_get_targeted_block(max_distance=2)
            if not current_check or getattr(current_check, "type", None) != TARGET_BLOCK:
                ms.echo("§a[Skaner] Cel zniszczony lub utracony.")
                break

            time.sleep(0.02)

        ms.player_press_attack(False)
        stop_all_actions()

        next_target = ms.player_get_targeted_block(max_distance=MAX_SCAN_DISTANCE)
        if next_target and getattr(next_target, "type", None) == TARGET_BLOCK:
            ms.echo("§e[Skaner] Kolejny cel wykryty — kontynuuję niszczenie.")
            threading.Thread(target=go_and_destroy_target, daemon=True).start()
            return

        yaw_now, _ = ms.player_orientation()
        new_yaw = _normalize_angle(yaw_now - 15)
        scanner_state.update({"current_yaw": new_yaw, "current_pitch": PITCH_MAX, "pitch_direction": "UP"})
        ms.player_set_orientation(new_yaw, PITCH_MAX)
        debug(f"Reset po zniszczeniu: yaw={new_yaw:.2f}, pitch={PITCH_MAX}")

    except Exception as e:
        ms.echo(f"§c[Skaner] Błąd: {e}")
    finally:
        destroy_thread_running.clear()
        scanning_allowed.set()
        ms.echo("§6[Skaner] Wznawiam skanowanie.")

# ================================================================
# SKANOWANIE
# ================================================================
def run_scan_tick():
    global scanner_state, bot_active
    yaw = scanner_state["current_yaw"]
    pitch = scanner_state["current_pitch"]
    direction = scanner_state["pitch_direction"]

    yaw += SCAN_SPEED_YAW
    if yaw > 180:
        yaw -= 360
    if direction == "DOWN":
        pitch -= SCAN_SPEED_PITCH
        if pitch <= PITCH_MIN:
            pitch, direction = PITCH_MIN, "UP"
    else:
        pitch += SCAN_SPEED_PITCH
        if pitch >= PITCH_MAX:
            pitch, direction = PITCH_MAX, "DOWN"

    ms.player_set_orientation(yaw, pitch)
    debug(f"Skanowanie: yaw={yaw:.2f}, pitch={pitch:.2f}, dir={direction}")

    target = ms.player_get_targeted_block(max_distance=MAX_SCAN_DISTANCE)
    if target and getattr(target, "type", None) == TARGET_BLOCK and bot_active:
        if not destroy_thread_running.is_set():
            ms.echo(f"§e[Skaner] Wykryto {target.type} — niszczę...")
            scanning_allowed.clear()
            destroy_thread_running.set()
            threading.Thread(target=go_and_destroy_target, daemon=True).start()

    scanner_state.update({"current_yaw": yaw, "current_pitch": pitch, "pitch_direction": direction})

def scan_loop():
    ms.echo("§6[Skaner] Wątek skanowania uruchomiony.")
    while True:
        scanning_allowed.wait()
        if not bot_active:
            time.sleep(0.1)
            continue
        try:
            run_scan_tick()
        except Exception as e:
            ms.echo(f"§c[Skaner] Błąd w run_scan_tick(): {e}")
        time.sleep(LOOP_DELAY_SECONDS)

# ================================================================
# PĘTLA GŁÓWNA
# ================================================================
def master_control_loop():
    global bot_active, scanner_state
    ms.echo("§6[Skaner] Pętla zdarzeń uruchomiona.")
    yaw, _ = ms.player_orientation()
    scanner_state.update({"current_yaw": yaw, "current_pitch": PITCH_MAX})

    with ms.EventQueue() as q:
        q.register_key_listener()
        while True:
            event = q.get()
            if event.type == ms.EventType.KEY:
                if event.key == KEY_F7 and event.action == 0:
                    bot_active = not bot_active
                    if bot_active:
                        ms.echo("§a[Skaner] AKTYWOWANY (F7).")
                        yaw, _ = ms.player_orientation()
                        scanner_state.update({"current_yaw": yaw, "current_pitch": PITCH_MAX})
                        scanning_allowed.set()
                    else:
                        ms.echo("§c[Skaner] WYŁĄCZONY (F7).")
                        stop_all_actions()
                        scanning_allowed.clear()
                elif event.key == KEY_F9 and event.action == 0:
                    ms.echo("§c[Skaner] Zamykanie programu (F9).")
                    exit(0)

# ================================================================
# START
# ================================================================
if __name__ == "__main__":
    threading.Thread(target=scan_loop, daemon=True).start()
    master_control_loop()
