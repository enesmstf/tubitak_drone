import re

with open('/home/enes/Belgeler/tubitak_drone/ground_station_core_v2.py', 'r') as f:
    content = f.read()

old_logic = """            if self.relative_alt_m is None or self.relative_alt_m < AIRBORNE_RELATIVE_ALT_THRESHOLD_M:
                return False, "havada değil (irtifa eşiği altında)"
            return True, "hazır\""""

new_logic = """            if BENCH_TEST_MODE:
                return True, "hazır (TEST MODU)"
            if self.relative_alt_m is None or self.relative_alt_m < AIRBORNE_RELATIVE_ALT_THRESHOLD_M:
                return False, "havada değil (irtifa eşiği altında)"
            return True, "hazır\""""

content = content.replace(old_logic, new_logic)

with open('/home/enes/Belgeler/tubitak_drone/ground_station_core_v2.py', 'w') as f:
    f.write(content)
