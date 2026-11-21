import unittest
from keyboard_drive import KeyboardDrive

class KeyboardTest(unittest.TestCase):
    def test_wasd(self):
        for key, expected in [('w', (.25, .25)), ('s', (-.25, -.25)),
                              ('a', (-.25, .25)), ('d', (.25, -.25))]:
            k = KeyboardDrive()
            k.event(0x300, ord(key))
            self.assertEqual(k.state(True), expected)
            k.event(0x301, ord(key))
            self.assertEqual(k.state(True), (0, 0))

    def test_guard_requires_fresh_press(self):
        k = KeyboardDrive()
        k.event(0x300, ord('w'))
        generation = k.generation
        self.assertEqual(k.state(False), (0, 0))
        k.event(0x300, ord('w'), repeat=True)
        self.assertEqual(k.state(True), (0, 0))
        self.assertEqual(k.generation, generation)
        k.event(0x301, ord('w'))
        k.event(0x300, ord('w'))
        self.assertEqual(k.state(True), (.25, .25))
        self.assertGreater(k.generation, generation)

    def test_focus_stop_minimize_exit(self):
        for event in (7, 13, 14):
            k = KeyboardDrive()
            k.event(0x300, ord('w'))
            k.event(0x200, window_event=event)
            self.assertEqual(k.state(True), (0, 0))
        for key in (32, 27):
            k.event(0x300, ord('w'))
            k.event(0x300, key)
            self.assertEqual(k.state(True), (0, 0))

if __name__ == '__main__':
    unittest.main()
