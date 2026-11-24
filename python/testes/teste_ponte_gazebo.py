"""Verifica o protocolo JSON/PWM e a rotação da odometria sem SITL."""
import math
from pathlib import Path
import struct
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from comunicacao.ponte_gazebo_json import decodificar_servo, velocidades_motores, rotacionar, quat_para_euler


class PonteTeste(unittest.TestCase):
    def test_pacotes_16_e_32(self):
        for magic,n in [(18458,16),(29569,32)]:
            pwm=[1000,1500,2000,0]+[1000]*(n-4)
            pacote=struct.pack('<HHI'+'H'*n,magic,100,12,*pwm)
            self.assertEqual(list(decodificar_servo(pacote)),pwm)
        self.assertIsNone(decodificar_servo(b''))
        self.assertIsNone(decodificar_servo(struct.pack('<HHI16H',29569,100,0,*([1000]*16))))

    def test_limites_motor(self):
        self.assertEqual(velocidades_motores([0,1000,1500,2000]),[0,0,550,1100])
        self.assertEqual(velocidades_motores([500,2500,1000,2000]),[0,1100,0,1100])

    def test_velocidade_corpo_para_mundo(self):
        q=[math.sqrt(.5),0,0,math.sqrt(.5)]
        v=rotacionar(q,[1,0,0])
        self.assertAlmostEqual(v[0],0)
        self.assertAlmostEqual(v[1],1)
        self.assertAlmostEqual(v[2],0)
        self.assertEqual(rotacionar([1,0,0,0],[1,2,3]),[1,2,3])

    def test_quaternion_para_euler(self):
        self.assertEqual(quat_para_euler(1,0,0,0),[0,0,0])
        r,p,y=quat_para_euler(math.sqrt(.5),0,0,math.sqrt(.5))
        self.assertAlmostEqual(r,0)
        self.assertAlmostEqual(p,0)
        self.assertAlmostEqual(y,math.pi/2)


if __name__=='__main__':
    unittest.main(verbosity=2)
