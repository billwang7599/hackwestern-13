import unittest
from headless_keyboard import HeadlessKeyboard

class HeadlessKeysTest(unittest.TestCase):
    def frame(self,k,t,mask,permit=True):
        k.latest=(int(t*1e6),1,mask)
        return k.sample(t,permit)
    def test_requires_neutral_and_release(self):
        k=HeadlessKeyboard()
        self.frame(k,10,1)
        self.assertEqual(k.drive.state(True),(0,0))
        self.frame(k,10.1,0); self.frame(k,10.2,1)
        self.assertEqual(k.drive.state(True),(.25,.25))
        self.frame(k,10.3,0)
        self.assertEqual(k.drive.state(True),(0,0))
    def test_stale_does_not_restart_held_key(self):
        k=HeadlessKeyboard()
        self.frame(k,10,0); self.frame(k,10.1,1)
        k.sample(10.5,True)
        self.frame(k,10.6,1)
        self.assertEqual(k.drive.state(True),(0,0))
    def test_guard_and_max_hold_require_release(self):
        for blocked in (True,False):
            k=HeadlessKeyboard()
            self.frame(k,10,0); self.frame(k,10.1,2)
            self.frame(k,12.2 if not blocked else 10.2,2,not blocked)
            self.frame(k,12.3,2)
            self.assertEqual(k.drive.state(True),(0,0))
            self.frame(k,12.4,0); self.frame(k,12.5,2)
            self.assertEqual(k.drive.state(True),(-.25,-.25))
    def test_conflict_stop_and_phone(self):
        k=HeadlessKeyboard()
        self.frame(k,10,0); self.frame(k,10.1,1); self.frame(k,10.2,5)
        self.assertEqual(k.drive.state(True),(0,0))
        self.frame(k,10.3,16)
        self.assertTrue(k.drive.claimed)
        self.frame(k,10.4,32)
        self.assertFalse(k.drive.claimed)
    def test_input_echo_and_phone_takeover(self):
        k=HeadlessKeyboard()
        self.frame(k,10,0); self.frame(k,10.1,1)
        k.take_phone_control()
        self.frame(k,10.2,1)
        self.assertFalse(k.drive.claimed)
        state=k.snapshot(10.2)
        self.assertEqual(state['keys'],['W'])
        self.assertEqual(state['last_press'],['W'])
        self.assertEqual(state['owner'],'phone')
        self.assertEqual(state['event_count'],1)
        self.frame(k,10.3,0)
        self.assertEqual(k.snapshot(10.3)['last_press'],['W'])
        self.assertEqual(k.snapshot(10.3)['last_keys'],['RELEASE'])

    def test_disconnect_and_future_timestamp(self):
        k=HeadlessKeyboard()
        self.frame(k,10,0); self.frame(k,10.1,1)
        k.latest=(0,0,0)
        self.assertFalse(k.sample(10.2,True))
        self.assertEqual(k.drive.state(True),(0,0))
        k.latest=(20000000,1,1)
        self.assertFalse(k.sample(10.3,True))

if __name__=='__main__': unittest.main()
