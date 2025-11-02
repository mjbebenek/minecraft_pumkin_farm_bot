import minescript as ms  # type: ignore
import time
import math
import threading

# ================================================================
# KONFIGURACJA
# ================================================================
TARGET_BLOCK = "minecraft:pumpkin"
SCAN_SPEED_YAW = 1
SCAN_SPEED_PITCH = 5
PITCH_MAX = 45
PITCH_MIN = 10
LOOP_DELAY_SECONDS = 0.02
KEY_F7 = 296
KEY_F9 = 298

MAX_SCAN_DISTANCE = 4
APPROACH_DISTANCE = 1.5
EYE_HEIGHT = 1.62
PITCH_CORRECTION_FACTOR = 0.85
INVENTORY_CAPACITY = 36
STUCK_TIME = 2.0         # po ilu sekundach braku ruchu uznajemy zacięcie
MIN_DIST_CHANGE = 0.1    # minimalna zmiana odległości, by uznać ruch
YAW_SCAN_RANGE = 15      # szerokość stożka w poziomie
PITCH_SCAN_RANGE = 10    # wysokość stożka stożka
YAW_STEP = 2
PITCH_STEP = 3

# --- DEBUG ---
DEBUG = True  # ustaw False, żeby wyciszyć debugowe echo

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

# ================================================================
# PLACEHOLDERS: DURABILITY / EKWIPUNEK
# ================================================================
def check_tool_durability():
    debug("Durability check placeholder (no-op).")
    return False

def check_inventory_status():
    try:
        inv = ms.player_inventory()
        if inv is None:
            return False
        contains_none = any(item is None for item in inv)
        if contains_none:
            total_slots = len(inv)
            filled_slots = sum(1 for item in inv if item is not None)
            debug(f"Inventory: {filled_slots}/{total_slots}")
            if filled_slots >= total_slots:
                ms.echo("§c[Ekwipunek] Ekwipunek pełny! (dokładne sprawdzenie)")
                return True
            return False
        else:
            filled_slots = len(inv)
            debug(f"Inventory (non-empty only): {filled_slots}/{INVENTORY_CAPACITY}")
            if filled_slots >= INVENTORY_CAPACITY:
                ms.echo("§c[Ekwipunek] Ekwipunek prawdopodobnie pełny!")
                return True
            return False
    except Exception as e:
        ms.echo(f"§c[Ekwipunek] Błąd: {e}")
        return False

# ================================================================
# AKCJE BOTA
# ================================================================
def stop_all_actions():
    ms.player_press_forward(False)
    ms.player_press_attack(False)
    ms.player_press_backward(False)

def go_and_destroy_target():
    """
    Podejdź do celu, koryguj yaw/pitch względem środka bloku, zniszcz blok.
    Dynamiczne przełączanie na bliższe cele: jeśli podczas podejścia/ataku
    pojawi się inna poprawna dynia, bliższa od obecnego targetu o co najmniej
    CLOSE_SWITCH_THRESHOLD, to przełączamy cel.
    (Zachowuje placeholders durability/inventory i DEBUG.)
    """
    global bot_active
    # próg (metry) — jeśli pojawi się target bliższy o co najmniej tą wartość, przełączamy
    CLOSE_SWITCH_THRESHOLD = 0.6

    def _get_pos_from_target(target):
        """Zwraca tuple (x,y,z) lub None jeśli nie da się odczytać."""
        if not target:
            return None
        try:
            if hasattr(target, "position"):
                p = target.position
                if hasattr(p, "x"):
                    return (p.x, p.y, p.z)
                else:
                    return tuple(p)
            elif isinstance(target, dict):
                return (target["x"], target["y"], target["z"])
        except Exception:
            return None
        return None

    try:
        ms.echo("§e[Skaner] Cel namierzony — podchodzę i niszczę...")
        scanning_allowed.clear()

        targeted = ms.player_get_targeted_block(max_distance=MAX_SCAN_DISTANCE)
        if not targeted:
            ms.echo("§c[Skaner] Brak targetu przy starcie niszczenia.")
            scanning_allowed.set()
            return

        # pobierz początkową pozycję targetu
        tpos = _get_pos_from_target(targeted)
        if not tpos:
            ms.echo("§c[Skaner] Nieznany format pozycji targetu — przerwanie.")
            scanning_allowed.set()
            return
        tx, ty, tz = tpos

        # zapamiętaj pitch w momencie wykrycia
        _, pitch_base = ms.player_orientation()

                # Podejście do celu z detekcją zablokowania
        last_distance = None
        last_move_time = time.time()
        STUCK_TIME = 2.0         # po ilu sekundach uznajemy, że bot się zaciął
        MIN_DIST_CHANGE = 0.1    # minimalna zmiana odległości, żeby uznać że się ruszył

        while bot_active:
            if check_tool_durability() or check_inventory_status():
                bot_active = False
                stop_all_actions()
                break

            px, py, pz = ms.player_position()
            eye_y = py + EYE_HEIGHT

            bx, by, bz = tx + 0.5, ty + 0.5, tz + 0.5
            dx, dz, dy = bx - px, bz - pz, by - eye_y
            horiz_dist = math.hypot(dx, dz)

            # --- Wykrywanie zablokowania ---
            now = time.time()
            if last_distance is not None:
                # wykrycie ruchu lub zastoju
                if abs(horiz_dist - last_distance) > MIN_DIST_CHANGE:
                    last_move_time = now
                elif now - last_move_time > STUCK_TIME:
                    ms.echo("§c[Skaner] Bot utknął — próbuję znaleźć nowy cel w pobliżu.")

                    current_yaw, current_pitch = ms.player_orientation()
                    best_target = None
                    best_distance = float("inf")

                    # płynny mini-skan — stożek wokół aktualnego kierunku
                    for yaw_offset in range(-YAW_SCAN_RANGE, YAW_SCAN_RANGE + 1, YAW_STEP):
                        for pitch_offset in range(-PITCH_SCAN_RANGE, PITCH_SCAN_RANGE + 1, PITCH_STEP):
                            target_yaw = current_yaw + yaw_offset
                            target_pitch = current_pitch + pitch_offset

                            # płynne przejście (4 kroki)
                            for interp in range(1, 5):
                                intermediate_yaw = current_yaw + (target_yaw - current_yaw) * (interp / 4.0)
                                intermediate_pitch = current_pitch + (target_pitch - current_pitch) * (interp / 4.0)
                                ms.player_set_orientation(intermediate_yaw, intermediate_pitch)
                                time.sleep(0.01)

                            t = ms.player_get_targeted_block(max_distance=MAX_SCAN_DISTANCE)
                            if t and getattr(t, "type", None) == TARGET_BLOCK:
                                pos = _get_pos_from_target(t)
                                if pos:
                                    dist = math.dist(ms.player_position(), pos)
                                    if dist < best_distance:
                                        best_distance = dist
                                        best_target = pos

                            time.sleep(0.02)  # chwilka na stabilizację

                    # powrót do pozycji początkowej
                    for interp in range(1, 6):
                        yaw_interp = current_yaw + (current_yaw - ms.player_orientation()[0]) * (interp / 5.0)
                        pitch_interp = current_pitch + (current_pitch - ms.player_orientation()[1]) * (interp / 5.0)
                        ms.player_set_orientation(yaw_interp, pitch_interp)
                        time.sleep(0.01)

                    if best_target:
                        tx, ty, tz = best_target
                        ms.echo(f"§e[Skaner] Nowy cel po zacięciu: {tx:.1f}, {ty:.1f}, {tz:.1f}")
                        last_move_time = now
                        last_distance = horiz_dist
                        continue

                    # jeśli nie znaleziono nowego celu — porzucamy
                    ms.echo("§4[Skaner] Brak nowych celów — porzucam i wracam do skanowania.")
                    stop_all_actions()
                    yaw_now, pitch_now = ms.player_orientation()
                    scanner_state.update({
                        "current_yaw": _normalize_angle(yaw_now + 10),
                        "current_pitch": pitch_now,
                        "pitch_direction": "UP"
                    })
                    scanning_allowed.set()
                    destroy_thread_running.clear()
                    return


            last_distance = horiz_dist

            if horiz_dist <= APPROACH_DISTANCE:
                ms.player_press_forward(False)
                break

            # Korekcja yaw/pitch jak wcześniej
            cur_yaw, cur_pitch = ms.player_orientation()
            desired_yaw = _compute_yaw_to_target((px, py, pz), (tx, ty, tz), cur_yaw)
            pitch_target = pitch_base - math.degrees(math.atan2(dy, horiz_dist)) * PITCH_CORRECTION_FACTOR
            pitch_target = max(PITCH_MIN, min(PITCH_MAX, pitch_target))

            # płynna zmiana pitch
            step = (pitch_target - cur_pitch) / 5.0
            for _ in range(5):
                cur_pitch += step
                ms.player_set_orientation(desired_yaw, cur_pitch)
                time.sleep(0.02)

            ms.player_press_forward(True)
            time.sleep(0.05)

        if not bot_active:
            stop_all_actions()
            destroy_thread_running.clear()
            scanning_allowed.set()
            ms.echo("§c[Skaner] Podejście przerwane.")
            return

        # --- Atak (również dynamiczne przełączanie jeśli pojawi się bliższy) ---
        ms.echo("§e[Skaner] Atakuję blok...")
        ms.player_press_attack(True)

        while bot_active:
            # sprawdzamy bieżący blok w celowniku
            current = ms.player_get_targeted_block(max_distance=2)
            cur_pos = _get_pos_from_target(current)

            px, py, pz = ms.player_position()

            # jeśli aktualny look jest pumpkin, policz jego odległość
            if cur_pos and getattr(current, "type", None) == TARGET_BLOCK:
                nx, ny, nz = cur_pos
                nx_center_x, nx_center_y, nx_center_z = nx + 0.5, ny + 0.5, nz + 0.5
                new_horiz = math.hypot(nx_center_x - px, nx_center_z - pz)
                # odległość do obecnego targeta (środek)
                bx, bz = tx + 0.5, tz + 0.5
                old_horiz = math.hypot(bx - px, bz - pz)
                # jeśli aktualny widoczny jest wyraźnie bliższy -> switch
                if new_horiz + CLOSE_SWITCH_THRESHOLD < old_horiz:
                    ms.echo(f"§e[Skaner] (Atak) przełączam na bliższy target: {new_horiz:.2f} < {old_horiz:.2f}")
                    tx, ty, tz = nx, ny, nz
                    # ustaw orientację tak, by patrzeć na nowy cel
                    cur_yaw, cur_pitch = ms.player_orientation()
                    desired_yaw = _compute_yaw_to_target((px, py, pz), (tx, ty, tz), cur_yaw)
                    ms.player_set_orientation(desired_yaw, cur_pitch)

            # jeśli w celowniku nie ma targetu lub inny typ -> zakończ atak (blok znikł/został zniszczony)
            current_check = ms.player_get_targeted_block(max_distance=2)
            if not current_check or getattr(current_check, "type", None) != TARGET_BLOCK:
                ms.echo("§a[Skaner] Cel zniszczony lub utracony.")
                break

            time.sleep(LOOP_DELAY_SECONDS)

        ms.player_press_attack(False)
        stop_all_actions()

        # --- Czy jest kolejny blok linia wzroku? Kontynuuj w nowym wątku ---
        next_target = ms.player_get_targeted_block(max_distance=MAX_SCAN_DISTANCE)
        if next_target and getattr(next_target, "type", None) == TARGET_BLOCK:
            ms.echo("§e[Skaner] Kolejny cel wykryty — kontynuuję niszczenie bez skanowania!")
            threading.Thread(target=go_and_destroy_target, daemon=True).start()
            return

        # --- Reset orientacji po zniszczeniu ---
        yaw_now, _ = ms.player_orientation()
        new_yaw = _normalize_angle(yaw_now - 15)
        scanner_state.update({
            "current_yaw": new_yaw,
            "current_pitch": PITCH_MAX,
            "pitch_direction": "UP"
        })
        ms.player_set_orientation(new_yaw, PITCH_MAX)
        debug(f"Reset po zniszczeniu: yaw={new_yaw:.2f}, pitch={PITCH_MAX}")

    except Exception as e:
        ms.echo(f"§c[Skaner] Błąd w go_and_destroy_target(): {e}")
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
            ms.echo(f"§e[Skaner] TARGET! Wykryto {target.type} — niszczę...")
            scanning_allowed.clear()
            destroy_thread_running.set()
            threading.Thread(target=go_and_destroy_target, daemon=True).start()

    scanner_state.update({
        "current_yaw": yaw,
        "current_pitch": pitch,
        "pitch_direction": direction
    })

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
