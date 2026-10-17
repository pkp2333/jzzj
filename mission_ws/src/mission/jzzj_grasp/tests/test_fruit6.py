"""No fresh inference. Synthetic tests always run; external saved assets are opt-in."""
import copy
import contextlib
import io
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[5]
PKG=ROOT/'mission_ws/src/mission/jzzj_grasp'
sys.path.insert(0,str(PKG/'src'))
from jzzj_grasp.fruit6 import (load_manifest,validate_config,check_resources,sha256,decode_saved_outputs,nms_xyxy,FrameGate)
MANIFEST=PKG/'resources/fruit6_manifest.json'
LABELS=PKG/'resources/fruit6_labels.txt'
CONFIG=PKG/'params/detector_fruit6.yaml'

def module(name,filename):
    spec=importlib.util.spec_from_file_location(name,str(PKG/'scripts'/filename));m=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m);return m
CLI=module('fruit6_cli_under_test','check_fruit6.py')
RUNNER=module('fruit6_runner_under_test','run_verified_fruit6.py')

class ResourceTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.base=Path(self.tmp.name)
        self.model=self.base/'yolov5s.rknn';self.model.write_bytes(b'synthetic model fixture only')
        self.m=load_manifest(MANIFEST);self.m['model']['sha256']=sha256(self.model)
        self.manifest=self.base/'manifest.json';self.manifest.write_text(json.dumps(self.m))
    def tearDown(self):self.tmp.cleanup()
    def test_packaged_labels(self):
        self.assertEqual(LABELS.read_text().splitlines(),['6']+['fruit-'+str(i) for i in range(1,7)])
        self.assertEqual(sha256(LABELS),self.m['labels']['sha256'])
    def test_valid_synthetic_model_and_labels(self):
        report=check_resources(self.base,LABELS,self.manifest)
        self.assertEqual(report['runtime_compatibility'],'NOT_VALIDATED_ON_DEVICE')
    def test_missing_model(self):
        self.model.unlink()
        with self.assertRaisesRegex(ValueError,'missing'):check_resources(self.base,LABELS,self.manifest)
    def test_wrong_model_hash(self):
        self.model.write_bytes(b'other model')
        with self.assertRaisesRegex(ValueError,'SHA256'):check_resources(self.base,LABELS,self.manifest)
    def test_14_class_names_rejected(self):
        p=self.base/'classes.txt';p.write_text('\n'.join('class'+str(i) for i in range(14)))
        with self.assertRaisesRegex(ValueError,'label count'):check_resources(self.base,p,self.manifest)
    def test_wrong_label_order(self):
        p=self.base/'labels.txt';p.write_text('6\n'+'\n'.join(self.m['labels']['names'][::-1]))
        with self.assertRaisesRegex(ValueError,'order'):check_resources(self.base,p,self.manifest)
    def test_anchor_shape_and_count(self):
        self.assertEqual(len(self.m['anchors']),18)
        self.assertEqual(self.m['anchors'][:6],[10,13,16,30,33,23])
        validate_config(json.loads(CONFIG.read_text()),self.m)
    def test_bad_config(self):
        for key,value in [('class_num',14),('anchors',[]),('expected_input_width',320),('conf_thresh',.5)]:
            c=json.loads(CONFIG.read_text());c[key]=value
            with self.assertRaises(ValueError):validate_config(c,self.m)
    def test_manifest_wrong_output_shape(self):
        self.m['model']['outputs'][0]['shape'][1]=57;self.manifest.write_text(json.dumps(self.m))
        with self.assertRaisesRegex(ValueError,'tensor'):load_manifest(self.manifest)
    def test_all_assets_fail_if_reference_missing(self):
        with self.assertRaisesRegex(ValueError,'reference'):check_resources(self.base,LABELS,self.manifest,True)
    def test_pinned_model_hash(self):
        self.assertEqual(load_manifest(MANIFEST)['model']['sha256'],'e3e96bf0e5aa62937319a62859cd2b8038e194c44ad6b731a827edac74192192')

class DecoderTests(unittest.TestCase):
    def empty(self):return [np.zeros((1,33,g,g),np.float32) for g in (80,40,20)]
    def test_no_detections(self):
        b,c,s=decode_saved_outputs(self.empty());self.assertEqual(b.shape,(0,4));self.assertEqual(len(c),0)
    def test_single_box_and_source_scale(self):
        heads=self.empty();heads[0][0,:4,10,20]=.5;heads[0][0,4,10,20]=.9;heads[0][0,7,10,20]=.8
        boxes,classes,scores=decode_saved_outputs(heads)
        self.assertEqual(classes.tolist(),[2]);self.assertAlmostEqual(scores[0],.72,places=5)
        # x=164,y=84 at model size, anchor 10x13; source height 480 scales y by .75.
        np.testing.assert_allclose(boxes[0],[159,77.5*.75,169,90.5*.75],atol=1e-4)
    def test_wrong_output_count_or_shape(self):
        a=self.empty()
        with self.assertRaises(ValueError):decode_saved_outputs(a[:2])
        a[0]=np.zeros((1,57,80,80),np.float32)
        with self.assertRaisesRegex(ValueError,'shape'):decode_saved_outputs(a)
    def test_wrong_dtype_nan_or_logits(self):
        for value in (np.zeros((1,33,80,80),np.int8),np.full((1,33,80,80),np.nan,np.float32),np.full((1,33,80,80),2.,np.float32)):
            a=self.empty();a[0]=value
            with self.assertRaises(ValueError):decode_saved_outputs(a)
    def test_per_class_nms_does_not_merge_different_classes(self):
        a=self.empty();a[0][0,:4,5,5]=.5;a[0][0,4,5,5]=1;a[0][0,5,5,5]=1
        a[0][0,11:15,5,5]=.5;a[0][0,15,5,5]=1;a[0][0,17,5,5]=1
        b,c,s=decode_saved_outputs(a);self.assertEqual(c.tolist(),[0,1])
    def test_nms_duplicate(self):
        self.assertEqual(nms_xyxy(np.array([[1,1,5,5],[1,1,5,5]]),np.array([.9,.8]),.45),[0])
    def test_invalid_source_dimensions(self):
        with self.assertRaises(ValueError):decode_saved_outputs(self.empty(),(640,0))

class FrameTests(unittest.TestCase):
    def test_duplicate_frame_rejected(self):
        g=FrameGate();self.assertTrue(g.accept(100,100));self.assertFalse(g.accept(100,100.1))
    def test_stale_future_zero_nan_rejected(self):
        g=FrameGate()
        for stamp in (0,99,101,float('nan')):self.assertFalse(g.accept(stamp,100))
    def test_empty_new_frame_still_advances(self):
        g=FrameGate();self.assertTrue(g.accept(100,100));self.assertTrue(g.accept(100.1,100.1))
        self.assertFalse(g.accept(100,100.1))

class LaunchAndRunnerTests(unittest.TestCase):
    def test_probe_launch_no_motion_nodes(self):
        root=ET.parse(PKG/'launch/model_probe_fruit6.launch').getroot()
        scripts={n.get('type') for n in root.iter('node')}
        self.assertNotIn('stage_c_server.py',scripts);self.assertNotIn('sim_context.py',scripts)
        self.assertNotIn('arm_trajectory_node.py',scripts)
        args={a.get('name'):a for a in root.findall('arg')}
        self.assertNotIn('default',args['model_dir'].attrib)
        self.assertEqual(args['start_camera'].get('default'),'false')
        self.assertEqual(args['enable_rgbd_observer'].get('default'),'false')
    def test_sample_launch_quarantines_test_image(self):
        root=ET.parse(PKG/'launch/model_probe_fruit6_sample.launch').getroot()
        args={a.get('name'):a.get('value') for a in root.find('include').findall('arg')}
        self.assertEqual(args['start_camera'],'false')
        self.assertEqual(args['enable_rgbd_observer'],'false')
        self.assertEqual(args['rgb_topic'],'/stage_c/model_probe/sample_rgb')
        self.assertEqual(args['detections_topic'],'/stage_c/model_probe/sample_detections')
        self.assertEqual(root.find('node').get('type'),'publish_fruit6_sample.py')

    def test_flat_config_and_launch_paths(self):
        c=json.loads(CONFIG.read_text());self.assertNotIn('yolov5_rknn_rc',c)
        root=ET.parse(PKG/'launch/model_probe_fruit6.launch').getroot();group=root.find("group[@ns='yolov5_rknn_rc']")
        params={p.get('name'):p.get('value') for p in group.findall('param')}
        self.assertEqual(params['model_url'],'$(arg model_dir)/yolov5s.rknn')
        self.assertIn('$(find jzzj_grasp)/resources/',params['labels_url'])
    def test_rgbd_config_keeps_hardware_disabled(self):
        c=json.loads((PKG/'params/stage_c_fruit6.yaml').read_text())
        self.assertEqual(c['perception']['allowed_labels'],load_manifest(MANIFEST)['labels']['names'])
        for key in ('real_motion_enabled','robot_calibration_confirmed','workspace_clear_confirmed','model_validated'):
            self.assertFalse(c['execution'][key])
        self.assertFalse(c['perception']['calibration_confirmed'])
    def test_runner_executes_existing_detector_and_preserves_remaps(self):
        lib=types.ModuleType('roslib');packages=types.ModuleType('roslib.packages')
        packages.find_node=lambda *args:['/existing/yolov5_rknn_stream'];lib.packages=packages
        report={'runtime_compatibility':'NOT_VALIDATED_ON_DEVICE'}
        with contextlib.redirect_stdout(io.StringIO()),patch.dict(sys.modules,{'roslib':lib,'roslib.packages':packages}),patch.object(RUNNER,'check_resources',return_value=report),patch.object(RUNNER.os,'execv') as execute:
            RUNNER.main(['--model-dir',str(PKG.resolve()),'--manifest',str(MANIFEST),'--labels',str(LABELS),'--config',str(CONFIG),'__name:=probe','__ns:=/yolov5_rknn_rc'])
            execute.assert_called_once_with('/existing/yolov5_rknn_stream',['/existing/yolov5_rknn_stream','__name:=probe','__ns:=/yolov5_rknn_rc'])
    def test_hash_failure_never_executes_detector(self):
        with contextlib.redirect_stderr(io.StringIO()),patch.object(RUNNER,'check_resources',side_effect=ValueError('hash wrong')),patch.object(RUNNER.os,'execv') as execute:
            code=RUNNER.main(['--model-dir',str(PKG.resolve()),'--manifest',str(MANIFEST),'--labels',str(LABELS),'--config',str(CONFIG)])
            self.assertEqual(code,2);execute.assert_not_called()


class OverlayCallbackTests(unittest.TestCase):
    """Actual overlay callback against memory-only ROS/CV doubles, no GUI or hardware."""
    @classmethod
    def setUpClass(cls):
        fake_ros=types.ModuleType('rospy');cls.now=[100.]
        class Stamp:
            def __init__(self,v):self.value=v
            def to_sec(self):return self.value
            def __eq__(self,other):return isinstance(other,Stamp) and self.value==other.value
        cls.Stamp=Stamp
        fake_ros.Time=types.SimpleNamespace(now=lambda:Stamp(cls.now[0]));fake_ros.logwarn_throttle=lambda *a:None
        cv=types.ModuleType('cv2');cv.FONT_HERSHEY_SIMPLEX=0
        cv.rectangle=lambda *a:None;cv.putText=lambda *a:None
        bridge=types.ModuleType('cv_bridge');bridge.CvBridge=object
        sensor=types.ModuleType('sensor_msgs.msg');sensor.Image=object
        user=types.ModuleType('user_msgs.msg');user.ObjDets=object
        with patch.dict(sys.modules,{'rospy':fake_ros,'cv2':cv,'cv_bridge':bridge,'sensor_msgs.msg':sensor,'user_msgs.msg':user}):
            cls.overlay_module=module('overlay_under_test','model_probe_overlay.py')
    def setUp(self):
        from collections import deque
        import threading,time
        self.now[0]=100.
        self.o=self.overlay_module.Overlay.__new__(self.overlay_module.Overlay)
        self.o.lock=threading.RLock();self.o.rgb=deque(maxlen=90);self.o.pending=deque(maxlen=20)
        self.o.gate=FrameGate();self.o.last_frame=None;self.o.last_output=None;self.o.last_received=None
        self.o.started=time.monotonic();self.o.labels=['fruit-'+str(i) for i in range(1,7)]
        self.published=[];self.status=[]
        self.o.image_pub=types.SimpleNamespace(publish=self.published.append)
        self.o.status=lambda level,message:self.status.append((level,message))
        self.o.bridge=types.SimpleNamespace(imgmsg_to_cv2=lambda m,encoding:m.data.copy(),
            cv2_to_imgmsg=lambda a,encoding:types.SimpleNamespace(data=a,encoding=encoding))
    def messages(self,stamp=100.,detections=None):
        header=types.SimpleNamespace(stamp=self.Stamp(stamp),frame_id='camera')
        image=types.SimpleNamespace(header=header,data=np.zeros((480,640,3),np.uint8))
        detections=[] if detections is None else detections
        det=types.SimpleNamespace(header=copy.deepcopy(header),dets=detections,num=len(detections))
        return image,det
    def test_empty_fresh_detection_publishes_clean_frame(self):
        image,det=self.messages();self.o.on_rgb(image);self.o.on_detection(det);self.o.tick(None)
        self.assertEqual(len(self.published),1);self.assertEqual(self.status[-1][0],0)
        self.assertIn('0 detections',self.status[-1][1])
        self.assertEqual(self.published[0].header.stamp,image.header.stamp)
    def test_duplicate_does_not_republish_success(self):
        image,det=self.messages();self.o.on_rgb(image);self.o.on_detection(det);self.o.tick(None)
        self.o.on_detection(det);self.now[0]=100.1;self.o.tick(None)
        self.assertEqual(len(self.published),1)
    def test_stale_clears_boxes_with_warning(self):
        import time
        image,det=self.messages();self.o.on_rgb(image);self.o.on_detection(det);self.o.tick(None)
        self.now[0]=101.;self.o.last_received=time.monotonic()-1.;self.o.tick(None)
        self.assertEqual(len(self.published),2);self.assertEqual(self.status[-1][0],1)
        self.assertIn('stale',self.status[-1][1])
    def test_wrong_neutral_label_is_error(self):
        item=types.SimpleNamespace(class_id=0,label='rotten_lemon')
        image,det=self.messages(detections=[item]);self.o.on_rgb(image);self.o.on_detection(det);self.o.tick(None)
        self.assertEqual(self.status[-1][0],2);self.assertIn('label mismatch',self.status[-1][1])
    def test_detection_can_arrive_before_rgb(self):
        image,det=self.messages();self.o.on_detection(det);self.o.tick(None)
        self.assertEqual(self.published,[])
        self.o.on_rgb(image);self.o.tick(None);self.assertEqual(len(self.published),1)
    def test_wrong_frame_cannot_match(self):
        image,det=self.messages();det.header.frame_id='other'
        self.o.on_rgb(image);self.o.on_detection(det);self.o.tick(None)
        self.assertEqual(self.published,[])

@unittest.skipUnless(os.environ.get('STAGE_C_FRUIT6_ASSETS'),'external reference assets not supplied (set STAGE_C_FRUIT6_ASSETS)')
class ExternalAssetsTests(unittest.TestCase):
    def test_actual_model_hash_and_all_source_artifacts(self):
        result=check_resources(os.environ['STAGE_C_FRUIT6_ASSETS'],LABELS,MANIFEST,True)
        self.assertEqual(len(result['checked_assets']),9)
    def test_saved_outputs_and_source_coordinates(self):
        base=Path(os.environ['STAGE_C_FRUIT6_ASSETS']);m=load_manifest(MANIFEST);ref=m['offline_reference']
        a=[np.load(base/('onnx_yolov5_'+str(i)+'.npy'),allow_pickle=False) for i in range(3)]
        boxes,classes,scores=decode_saved_outputs(a,ref['source_image_size_wh'],anchors=m['anchors'])
        self.assertEqual(classes.tolist(),ref['classes'])
        np.testing.assert_allclose(scores,ref['scores'],atol=.00015,rtol=0)
        expected=np.array(ref['boxes_model_xyxy'])*[1.,.75,1.,.75]
        np.testing.assert_allclose(boxes,expected,atol=.12,rtol=0)
    def test_cli_read_only_replay(self):
        with contextlib.redirect_stdout(io.StringIO()):
            code=CLI.main(['--model-dir',os.environ['STAGE_C_FRUIT6_ASSETS'],'--manifest',str(MANIFEST),'--labels',str(LABELS),'--replay'])
        self.assertEqual(code,0)

if __name__=='__main__':unittest.main()
