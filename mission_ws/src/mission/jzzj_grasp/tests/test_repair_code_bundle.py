"""Corrupt-source repair backs up originals and never alters private field data."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest.mock import patch
from test_stage_c import ROOT

spec=importlib.util.spec_from_file_location('repair_bundle',ROOT/'tools/stage_c/repair_code_bundle.py')
repair=importlib.util.module_from_spec(spec);spec.loader.exec_module(repair)


class RepairTests(unittest.TestCase):
    def archive(self,path,files):
        manifest={n:repair.digest(v) for n,v in files.items()}
        with tarfile.open(path,'w:gz') as tar:
            for name,data in dict(files,**{'MANIFEST.json':json.dumps(manifest).encode()}).items():
                m=tarfile.TarInfo(name);m.size=len(data);tar.addfile(m,io.BytesIO(data))

    def test_bundle_rejects_nul_and_non_code_members(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(repair,'ALLOWED',{'tools/stage_c/example.py'}):
            archive=Path(tmp)/'bundle.tgz'
            self.archive(archive,{'tools/stage_c/example.py':b'print(1)\n'})
            self.assertEqual(repair.load_bundle(archive)['tools/stage_c/example.py'][0],b'print(1)\n')
            self.archive(archive,{'tools/stage_c/example.py':b'\0'*100})
            with self.assertRaisesRegex(ValueError,'corrupt payload'):repair.load_bundle(archive)
            self.archive(archive,{'.local/private.json':b'{}'})
            with self.assertRaisesRegex(ValueError,'allowlist'):repair.load_bundle(archive)

    def test_repair_keeps_corrupt_backup_private_data_and_session_locks(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'tools/stage_c/a.py';source.parent.mkdir(parents=True);source.write_bytes(b'\0'*12566)
            private=root/'.local/fixed_grasp';private.mkdir(parents=True)
            session=private/'handoff_state.json';session.write_text('{"state":"FAULT"}')
            lock=root/'hardware.lock';lock.write_text('original owner')
            backup=root/'backup';backup.mkdir()
            payload={'tools/stage_c/a.py':(b'print(1)\n',0o644)}
            repair.restore(root,payload,backup)
            self.assertEqual(source.read_bytes(),b'print(1)\n')
            with tarfile.open(backup/'sources-before.tar') as tar:self.assertEqual(tar.extractfile('tools/stage_c/a.py').read(),b'\0'*12566)
            self.assertEqual(session.read_text(),'{"state":"FAULT"}');self.assertEqual(lock.read_text(),'original owner')

    def test_write_failure_restores_originals_and_removes_only_new_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'a.py').write_text('original');backup=root/'backup';backup.mkdir()
            original=repair.write_atomic;failed=[False]
            def fail_once(path,data,mode):
                if path.name=='b.py' and not failed[0]:failed[0]=True;raise IOError('simulated write failure')
                return original(path,data,mode)
            with patch.object(repair,'write_atomic',side_effect=fail_once):
                with self.assertRaisesRegex(IOError,'simulated'):repair.restore(root,{'a.py':(b'updated',0o644),'b.py':(b'new',0o644)},backup)
            self.assertEqual((root/'a.py').read_text(),'original');self.assertFalse((root/'b.py').exists())

    def test_verify_detects_corruption_without_modifying_it(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'a.py').write_bytes(b'\0')
            with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(repair.verify(root,{'a.py':(b'pass\n',0o644)}),['a.py'])
            self.assertEqual((root/'a.py').read_bytes(),b'\0')

    def test_build_cache_is_moved_and_devel_is_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);backup=root/'backup';backup.mkdir()
            for ws in ('catkin_ws','mission_ws'):
                (root/ws/'build').mkdir(parents=True);(root/ws/'build/build.make').write_bytes(b'\0')
                (root/ws/'devel').mkdir();(root/ws/'devel/setup.bash').write_text('original')
            repair.quarantine_builds(root,backup)
            for ws in ('catkin_ws','mission_ws'):
                self.assertFalse((root/ws/'build').exists())
                self.assertEqual((backup/('build-cache-'+ws)/'build.make').read_bytes(),b'\0')
                self.assertEqual((root/ws/'devel/setup.bash').read_text(),'original')

    def test_target_rejects_escape(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):repair.target(Path(tmp),'../outside.py')


if __name__=='__main__':unittest.main()
