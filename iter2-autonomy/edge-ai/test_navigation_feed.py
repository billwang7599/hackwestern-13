"""Native YOLO box conversion and guard->planner copy, with fake sensors only."""
import json
from pathlib import Path
import socket
import subprocess
import tempfile
import unittest
from test_guard import packet

HERE=Path(__file__).resolve().parent

class NavigationTests(unittest.TestCase):
    def test_pixel_boxes_and_empty_frames(self):
        source=r'''
#include "navigation_feed.hpp"
#include <iostream>
#include <vector>
struct D { float left,top,right,bottom,confidence; int label; };
int main() {
    const char* labels[]={"chair","bottle"};
    std::vector<D> d={{10,20,310,220,.9,0},{0,0,50,100,.7,1},{0,0,900,100,.8,0}};
    std::cout << navigation_detections(640,480,d,labels);
    std::cout << navigation_detections(640,480,std::vector<D>{},labels);
}
'''
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp); (root/'test.cpp').write_text(source)
            subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-I',str(HERE),str(root/'test.cpp'),'-o',str(root/'test')],check=True)
            result=subprocess.check_output([str(root/'test')],text=True).splitlines()
        first,empty=map(json.loads,result)
        self.assertEqual((first['w'],first['h']),(640,480))
        self.assertEqual(first['dets'][0]['box'],[10,20,310,220])
        self.assertEqual(len(first['dets']),2)
        self.assertEqual(empty['dets'],[])

    def test_guard_forwards_only_valid_wire(self):
        subprocess.run(['sh',str(HERE/'build-guard.sh')],check=True)
        with socket.socket() as phone, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as nav:
            phone.bind(('127.0.0.1',0));phone.listen();phone.settimeout(3)
            nav.bind(('127.0.0.1',0));nav.settimeout(.4)
            proc=subprocess.Popen([str(HERE/'build/lidar_guard'),'--port',str(phone.getsockname()[1]),
                '--nav-port',str(nav.getsockname()[1]),'--seconds','3'],stderr=subprocess.DEVNULL,stdout=subprocess.DEVNULL)
            try:
                with phone.accept()[0] as wire:
                    wire.settimeout(1)
                    good=packet(seq=1)
                    wire.sendall(good)
                    self.assertEqual(wire.recv(1),b'\x01')
                    self.assertEqual(nav.recv(60000),good[4:])
                    bad=bytearray(packet(seq=2));bad[4:8]=b'BAD!'
                    wire.sendall(bad)
                    with self.assertRaises(socket.timeout):nav.recv(60000)
            finally:
                proc.terminate();proc.wait(timeout=3)

if __name__=='__main__':unittest.main()
