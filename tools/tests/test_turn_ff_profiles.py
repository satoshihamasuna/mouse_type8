"""FFレポートのプロファイル読取が共有定義と従来ヘッダーで一致することを確認する。"""
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools/analysis'))
from turn_ff_report import parse_profiles


class TurnFfProfilesTests(unittest.TestCase):
    def test_header_and_shared_definition_select_the_same_profile_and_ignore_other_sections(self):
        legacy = '''const static t_turn_param_table table={.5f,-37.5f,44,35,-90,Turn_R,0};
const static t_ff_gain gain={.8f,.1f,.09f,.002f,.0008f,.0009f,.08f,0};
const static t_param param_R90_500={&table,&sp,&om,&gain};
'''
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)/'Param_A';folder.mkdir()
            header=folder/'turn_500.h';header.write_text(legacy,encoding='utf-8')
            expected=parse_profiles(header);self.assertEqual(len(expected),1)
            header.write_text('extern const t_turn_param_table table;\n',encoding='utf-8')
            shared='// ===== Params/Param_A/turn_500.h =====\n'+legacy.replace('const static','const')
            shared+='// ===== Params/Param_A/turn_700.h =====\n'+legacy.replace('param_R90_500','param_R90_700')
            (folder/'run_param_A.cpp').write_text(shared,encoding='utf-8')
            self.assertEqual(parse_profiles(header),expected)
            (folder/'run_param_A.cpp').write_text('// missing section\n',encoding='utf-8')
            with self.assertRaises(ValueError):parse_profiles(header)
            # The paired definition takes precedence over an old aggregate file.
            (folder/'turn_500.cpp').write_text(legacy.replace('const static','const'),encoding='utf-8')
            self.assertEqual(parse_profiles(header),expected)
            nested=folder/'turn_500';nested.mkdir()
            (nested/'turn_500.h').write_text('extern const t_turn_param_table table;\n',encoding='utf-8')
            (nested/'turn_500.cpp').write_text(legacy.replace('const static','const'),encoding='utf-8')
            self.assertEqual(parse_profiles(nested/'turn_500.h'),expected)

    def test_real_mouse_a_turn500_has_fourteen_profiles_with_current_geometry(self):
        profiles=parse_profiles(ROOT/'Params/Param_A/turn_500/turn_500.h')
        self.assertEqual(len(profiles),14)
        self.assertEqual(len({p['motion_name'] for p in profiles}),14)
        right=next(p for p in profiles if p['motion_name']=='long_r90')
        self.assertAlmostEqual(right['r_min_mm'],-37.5)
        self.assertAlmostEqual(right['velocity_m_s'],.5)


if __name__=='__main__':
    unittest.main()
