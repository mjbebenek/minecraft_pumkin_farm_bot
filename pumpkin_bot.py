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

# --- DEBUG ---
DEBUG = True  # ustaw na False, żeby wyciszyć debugowe echo

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
    """
    PLACEHOLDER: kontrola trwałości narzędzia (main_hand).
    Zaimplementuj parsowanie NBT lub inne sprawdzenie, gdy API to umożliwi.
    Zwraca True jeśli trwałość wymaga zatrzymania bota.
    """
    debug("Durability check placeholder (no-op).")
    return False

def check_inventory_status():
    """
    PLACEHOLDER: wykrywanie pełnego ekwipunku.
    - obsługuje zarówno listę z None jak i listę zawierającą tylko niepuste sloty.
    Zwraca True jeśli wymagane jest zatrzymanie bota.
    """
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
    (W tej wersji - BEZ timeoutów. Prosty flow: podejdź → atakuj dopóki blok istnieje.)
    """
    global bot_active
    try:
        ms.echo("§e[Skaner] Cel namierzony — podchodzę i niszczę...")
        scanning_allowed.clear()

        targeted = ms.player_get_targeted_block(max_distance=MAX_SCAN_DISTANCE)
        if not targeted:
            ms.echo("§c[Skaner] Brak targetu przy starcie niszczenia.")
            scanning_allowed.set()
            return

        # Pobierz pozycję celu (kompatybilnie z różnymi formatami)
        try:
            if hasattr(targeted, "position"):
                tpos = targeted.position
                if hasattr(tpos, "x"):
                    tx, ty, tz = tpos.x, tpos.y, tpos.z
                else:
                    tx, ty, tz = tuple(tpos)
            elif isinstance(targeted, dict):
                tx, ty, tz = targeted["x"], targeted["y"], targeted["z"]
            else:
                raise ValueError("Nieznany format targetu")
        except Exception as e:
            ms.echo(f"§c[Skaner] Nie udało się odczytać pozycji bloku: {e}")
            scanning_allowed.set()
            return

        # zapamiętaj pitch w momencie wykrycia
        _, pitch_base = ms.player_orientation()

        # --- Podejście do celu ---
        while bot_active:
            # opcjonalne kontrole (placeholders)
            if check_tool_durability():
                ms.echo("§c[Skaner] Durability wymaga interwencji — wstrzymuję bota.")
                bot_active = False
                stop_all_actions()
                break
            if check_inventory_status():
                ms.echo("§c[Skaner] Inventory wymaga interwencji — wstrzymuję bota.")
                bot_active = False
                stop_all_actions()
                break

            px, py, pz = ms.player_position()
            eye_y = py + EYE_HEIGHT

            bx, by, bz = tx + 0.5, ty + 0.5, tz + 0.5
            dx, dz, dy = bx - px, bz - pz, by - eye_y
            horiz_dist = math.hypot(dx, dz)

            if horiz_dist <= APPROACH_DISTANCE:
                ms.player_press_forward(False)
                break

            cur_yaw, cur_pitch = ms.player_orientation()
            desired_yaw = _compute_yaw_to_target((px, py, pz), (tx, ty, tz), cur_yaw)

            pitch_target = pitch_base - math.degrees(math.atan2(dy, horiz_dist)) * PITCH_CORRECTION_FACTOR
            pitch_target = max(PITCH_MIN, min(PITCH_MAX, pitch_target))

            # płynna zmiana pitch (5 kroków)
            step = (pitch_target - cur_pitch) / 5.0
            for _ in range(5):
                cur_pitch += step
                ms.player_set_orientation(desired_yaw, cur_pitch)
                time.sleep(0.02)

            ms.player_press_forward(True)
            debug(f"Podejście: dist={horiz_dist:.2f}, yaw={desired_yaw:.2f}, pitch_target={pitch_target:.2f}")
            time.sleep(0.05)

        if not bot_active:
            stop_all_actions()
            destroy_thread_running.clear()
            scanning_allowed.set()
            ms.echo("§c[Skaner] Podejście przerwane.")
            return

        # --- Niszczenie (PROSTE: atakuj dopóki blok istnieje) ---
        ms.echo("§e[Skaner] Atakuję blok...")
        ms.player_press_attack(True)

        while bot_active:
            t = ms.player_get_targeted_block(max_distance=2)
            if not t or getattr(t, "type", None) != TARGET_BLOCK:
                # blok zniknął / zniszczono / stracono
                ms.echo("§a[Skaner] Cel zniszczony lub utracony.")
                break
            # opcjonalnie można tu dodać małe opóźnienie lub monitoring
            time.sleep(LOOP_DELAY_SECONDS)

        ms.player_press_attack(False)
        stop_all_actions()

        # --- Jeśli kolejny blok w tej samej linii, kontynuuj w nowym wątku ---
        next_target = ms.player_get_targeted_block(max_distance=MAX_SCAN_DISTANCE)
        if next_target and getattr(next_target, "type", None) == TARGET_BLOCK:
            ms.echo("§e[Skaner] Kolejny cel wykryty — kontynuuję niszczenie bez skanowania!")
            threading.Thread(target=go_and_destroy_target, daemon=True).start()
            return  # ważne: return -> nowy wątek przejmuje działanie

        # --- Restart skanowania: yaw-15, pitch -> PITCH_MAX, kierunek UP ---
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
