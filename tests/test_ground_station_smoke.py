import os
import signal
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from ground_station_core import GroundStation


class GroundStationSmokeTest(unittest.TestCase):
    def test_simulation_run_starts_and_stops(self):
        gs = GroundStation(
            lora_port='dummy',
            pixhawk_ports={1: 'p1', 2: 'p2', 3: 'p3', 4: 'p4'},
            simulation=True,
            platform_label='Test'
        )

        def stop_after_short_time():
            time.sleep(1.2)
            gs.shutdown()

        import threading
        threading.Thread(target=stop_after_short_time, daemon=True).start()

        try:
            gs.run()
        except SystemExit:
            self.fail('Ground station should not exit during normal operation')
        except KeyboardInterrupt:
            self.fail('Ground station should not raise KeyboardInterrupt in test mode')


if __name__ == '__main__':
    unittest.main()
