import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'camera'))
from protocol import Parser,Stable,encode,crc16,validate_request
from detectors import choose_unique,cluster_rings,material
import settings

class Tests(unittest.TestCase):
    def test_crc_and_corruption(self):
        self.assertEqual(crc16(b'123456789'),0x29b1)
        packet=encode(77,values=[0,0,320,240,160,120,0,0])
        for index in range(30):
            broken=bytearray(packet);broken[index]^=64
            parser=Parser();self.assertEqual(parser.feed(broken),[])
            self.assertEqual(parser.feed(packet)[0]['token'],77)

    def test_fragment_concat_noise(self):
        packet=encode(42)
        parser=Parser();self.assertEqual(parser.feed(b'noise\xa5'),[])
        self.assertEqual(parser.feed(packet[:10]),[])
        result=parser.feed(packet[10:]+packet)
        self.assertEqual([p['token'] for p in result],[42,42])

    def test_request(self):
        request=Parser().feed(encode(1,values=[0,0,320,240,160,120,0,0]))[0]
        self.assertTrue(validate_request(request))
        request['values'][2]=321;self.assertFalse(validate_request(request))

    def test_stability(self):
        s=Stable()
        for t in [0,40,80,120]:self.assertFalse(s.update((10,10),t))
        self.assertTrue(s.update((10,10),200))
        self.assertFalse(s.update(None,220))
        self.assertFalse(s.update((10,10),240))
        for n in range(10):self.assertFalse(s.update((n,10),300+40*n))

    def test_identity_ambiguity(self):
        self.assertIsNone(choose_unique([(10,10),(12,10)],(10,10),20))
        self.assertEqual(choose_unique([(10,10),(100,100)],(10,10),20),(10,10))

    def test_cluster_mean_sort(self):
        circles=[(30,60,r) for r in (10,20,30,40,50)]+[(50,60,r) for r in (10,20,30)]
        self.assertEqual([c[0] for c in cluster_rings(circles)],[30,50])
        self.assertEqual(cluster_rings([(10,10,30),(10,10,31)]),[])

    def test_material_filter_before_selection(self):
        class Blob:
            def __init__(self,w,h):self.width=w;self.height=h
            def w(self):return self.width
            def h(self):return self.height
            def x(self):return 100
            def y(self):return 90
            def cx(self):return 160
            def cy(self):return 120
            def pixels(self):return self.width*self.height
        class Image:
            def find_blobs(self,*a,**kw):return [Blob(300,200),Blob(50,50)]
        request=Parser().feed(encode(1,values=[0,0,320,240,160,120,0,0]))[0]
        old=settings.SIZE_BOUNDS[1];settings.SIZE_BOUNDS[1]=(20,80,20,80)
        try:self.assertEqual(material(Image(),request)[:2],(160,120))
        finally:settings.SIZE_BOUNDS[1]=old

if __name__=='__main__':unittest.main()
