from __future__ import annotations

import time

from lerobot_teleoperator_omega_continuum import OmegaContinuum, OmegaContinuumConfig


def main() -> None:
    omega = OmegaContinuum(OmegaContinuumConfig(id="omega_continuum"))
    omega.connect()
    try:
        while True:
            print(omega.get_action())
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        omega.disconnect()


if __name__ == "__main__":
    main()
