"""新旧保存ログの型と、実GUI受信経路のADC整数精度を検証する。実機接続なし。"""
import csv
import io
import math
from pathlib import Path
import struct
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools'))
from log_protocol import IR_TRAILER, saved_log_frame, saved_log_magic, decode_saved_values


def labels_v3():
    return ['cnt'] + ['field'+str(i) for i in range(49)] + list(IR_TRAILER)


def labels_v4():
    labels=labels_v3()
    labels[1:5]=['ideal.velo','ego.velo','ideal.accel','ego.accel']
    labels[36:40]=['irq.total_us','irq.logging_us','irq.max_us','irq.period_us']
    return labels


def labels_v5():
    labels = labels_v4()
    labels[13], labels[15] = 'estimate.heading_rad', 'estimate.sigma_heading_rad'
    return labels


def packet_v3(magic=0xA55B):
    return struct.pack('<HH49e4h2H', magic, 123, *[i/4 for i in range(49)], -32768, -7, 2049, 32767, 2209, 7)


class SavedLogTests(unittest.TestCase):
    def test_saved_adc_and_packed_words_are_exact_and_size_stays_114(self):
        for labels,expected in ((labels_v3(),0xA55B),(labels_v4(),0xA55C),(labels_v5(),0xA55D)):
            frame=saved_log_frame(labels)
            self.assertEqual(frame.size,114)
            self.assertEqual(saved_log_magic(labels),expected)
            self.assertEqual(saved_log_magic(tuple(labels)),expected)
            magic,index,*words=frame.unpack(packet_v3(expected))
            values=decode_saved_values(labels,words)
            self.assertEqual((magic,index),(expected,123))
            self.assertEqual(values[:49],[i/4 for i in range(49)])
            self.assertEqual(values[49:],[-32768,-7,2049,32767,2209,7])

    def test_legacy_51_and_55_words_keep_binary16_semantics(self):
        for count in (51,55):
            labels = ['cnt']+['field']*count
            words = struct.unpack('<'+str(count)+'H',struct.pack('<'+str(count)+'e',*[2048.]*count))
            self.assertEqual(saved_log_magic(labels),0xA55A)
            self.assertEqual(decode_saved_values(labels,words), [2048.]*count)
        with self.assertRaises(ValueError): decode_saved_values(labels_v3(), [0]*54)
        with self.assertRaises(ValueError): saved_log_magic(None)

    def test_heading_valid_zero_and_invalid_nan_are_distinct_in_saved_v5(self):
        labels = labels_v5()
        values = [0.0]*49 + [-32768, -7, 2049, 32767, 2209, 7]
        values[12], values[14] = 0.0, 0.0078125
        for invalid in (False, True):
            if invalid: values[12] = values[14] = float('nan')
            wire = struct.pack('<HH49e4h2H', 0xA55D, 1, *values)
            _, _, *words = saved_log_frame(labels).unpack(wire)
            decoded = decode_saved_values(labels, words)
            if invalid: self.assertTrue(math.isnan(decoded[12]) and math.isnan(decoded[14]))
            else: self.assertEqual((decoded[12], decoded[14]), (0.0, 0.0078125))
            self.assertEqual(decoded[49:], values[49:])

    def test_main_gui_saves_new_and_old_frames_using_header_selected_types(self):
        import myshell_gui as gui
        for version in (2,3,4,5):
            with tempfile.TemporaryDirectory() as directory:
                labels=labels_v5() if version==5 else labels_v4() if version==4 else labels_v3() if version==3 else ['cnt']+['field'+str(i) for i in range(55)]
                start=packet_v3(0xA55D if version==5 else 0xA55C if version==4 else 0xA55B) if version>=3 else struct.pack('<HH55e',0xA55A,123,*[i/4 for i in range(55)])
                data = io.BytesIO(start+packet_v3(0xFFFF))
                lines = iter(['START','HEADER,'+','.join(labels),'BINARY'])
                commands=[]
                instance=SimpleNamespace(serial_lock=threading.Lock(),command_timeout=1.,
                    _append=lambda text:None,_prepare_shell=lambda:None,_write_command=commands.append,
                    _readline_text=lambda:next(lines),_read_exact=data.read)
                with patch.object(gui,'__file__',str(Path(directory)/'myshell_gui.py')):
                    gui.MyshellGui._log_binary_worker(instance)
                path=next((Path(directory)/'logs').glob('*.csv'))
                with path.open(encoding='utf-8',newline='') as stream: rows=list(csv.DictReader(stream))
                self.assertEqual(len(rows),1)
                self.assertEqual(rows[0]['cnt'],'123')
                if version>=3:
                    self.assertEqual(rows[0]['sen_fl.value'],'2049')
                    self.assertEqual(rows[0]['sen_l.value'],'-32768')
                    self.assertEqual(rows[0]['ir.range_states'],'2209')
                else: self.assertEqual(rows[0]['field54'],'13.5')


if __name__=='__main__': unittest.main()
