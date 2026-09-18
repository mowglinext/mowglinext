import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).with_name("package_release.py")
SPEC = importlib.util.spec_from_file_location("package_release", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class PackageReleaseTest(unittest.TestCase):
    def test_usb_dfu_metadata_is_capability_and_mcu_driven(self):
        by_board = {entry["board"]: entry for entry in MODULE.PERMUTATIONS}
        f401 = by_board["BOARD_YARDFORCE500B"]
        f103 = by_board["BOARD_YARDFORCE500"]

        self.assertEqual(f401["env"], "Yardforce500B")
        self.assertEqual(f401["mcu"], "STM32F401VC")
        self.assertIs(f401["usb_dfu"], True)
        self.assertEqual(f103["mcu"], "STM32F103VC")
        self.assertIs(f103["usb_dfu"], False)
        for entry in (f401, f103):
            self.assertEqual(entry["flash_address"], "0x08000000")
            self.assertEqual(entry["flash_size"], 262144)


if __name__ == "__main__":
    unittest.main()
