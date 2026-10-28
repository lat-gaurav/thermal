"""Frame identity checks without opening the live camera or Cube link.

Run: python3 -m unittest discover -s tests -v
"""
import ast
import io
import json
from pathlib import Path
import shutil
import subprocess
import threading
import time
import textwrap
import unittest

import numpy as np

from flight.camera import FlightCamera
from flight.preview import Preview, _INDEX, _make_handler


class Request:
    def __init__(self, data):
        self.input = io.BytesIO(data)
        self.output = bytearray()

    def makefile(self, *args):
        return self.input

    def sendall(self, data):
        self.output.extend(data)


def request(preview, path, body=None):
    data = b'' if body is None else json.dumps(body).encode()
    method = 'GET' if body is None else 'POST'
    wire = (f'{method} {path} HTTP/1.1\r\nHost: localhost\r\n'
            f'Connection: close\r\nContent-Length: {len(data)}\r\n\r\n').encode() + data
    sock = Request(wire)
    _make_handler(preview)(sock, ('127.0.0.1', 1), None)
    return bytes(sock.output)


class ClickFramesTest(unittest.TestCase):
    def test_pipeline_computes_capture_gap_and_consumes_once(self):
        source = Path('tools/flight_pipeline.py').read_text()
        start = source.index('            manual_click = (preview.take_click(with_receipt=True)')
        end = source.index('            if manual_click is not None:', start)
        code = compile(textwrap.dedent(source[start:end]), '<click pickup>', 'exec')
        preview = Preview(scale=0.5)
        preview.set_click(10, 20, 120)
        state = dict(preview=preview, init_name='manual_click', capture_frame=130)
        exec(code, state)
        self.assertEqual(state['manual_point'], (20., 40.))
        self.assertEqual((state['click_frame'], state['click_gap']), (120, 10))
        exec(code, state)
        self.assertIsNone(state['manual_point'])
        self.assertEqual(state['click_gap'], '')
        preview.set_click(10, 20, 130)
        state['init_name'] = 'cue_nearest'
        exec(code, state)
        self.assertIsNone(state['manual_point'])
        self.assertEqual(preview.take_click(), (20., 40., 130))

    def test_camera_index_counts_processing_skips(self):
        camera = FlightCamera()
        class Capture:
            def read(self):
                if camera.grabbed == 2:
                    camera.running = False
                return True, np.zeros((4, 4), np.uint8)
        camera.cap = Capture()
        camera.running = True
        camera._loop()
        _, frame, skipped, index = camera.read(with_index=True)
        self.assertEqual((skipped, index), (2, 3))
        self.assertEqual(camera.read(with_index=True), (None, None, 0, None))
        # Legacy three-value API remains available to disk_soak.
        self.assertEqual(camera.read(), (None, None, 0))

    def test_encoder_pairs_index_with_encoded_image_not_new_pending_frame(self):
        preview = Preview(max_fps=0)
        entered, release = threading.Event(), threading.Event()
        def render(frame, overlay):
            entered.set()
            release.wait(2)
            return b'image-41'
        preview._render = render
        preview._run = True
        preview.offer(None, {}, capture_index=41)
        encoder = threading.Thread(target=preview._encode_loop)
        encoder.start()
        try:
            self.assertTrue(entered.wait(2))
            preview.offer(None, {}, capture_index=99)
            # Prevent encoding the next pending frame, while finishing this one.
            preview._run = False
            release.set()
            encoder.join(2)
            _, jpeg, index = preview.latest(-1, with_index=True)
            self.assertEqual((jpeg, index), (b'image-41', 41))
            response = request(preview, '/frame.jpg')
            self.assertIn(b'X-Capture-Index: 41\r\n', response)
            self.assertTrue(response.endswith(b'image-41'))
        finally:
            preview._run = False
            release.set()
            encoder.join(2)

    def test_click_round_trip_and_restart_rejection(self):
        preview = Preview(scale=0.5)
        body = dict(x=30, y=40, frame_index=41, session=preview.session)
        self.assertIn(b'200 OK', request(preview, '/api/click', body))
        self.assertEqual(preview.take_click(), (60., 80., 41))
        preview.set_click(10, 20, 42)  # a newer click is independent
        self.assertEqual(preview.take_click(), (20., 40., 42))
        self.assertIsNone(preview.take_click())
        body['session'] = 'before-restart'
        self.assertIn(b'400', request(preview, '/api/click', body))
        self.assertIsNone(preview.take_click())
        preview._click = (1, 2, time.monotonic()-1, 41, time.monotonic()-2)
        self.assertIsNone(preview.take_click())

    def test_csv_columns_match_active_row(self):
        tree = ast.parse(Path('tools/flight_pipeline.py').read_text())
        header = next(n.value for n in tree.body
                      if isinstance(n, ast.Assign) and any(
                          isinstance(t, ast.Name) and t.id == 'CSV_HEADER' for t in n.targets))
        columns = ast.literal_eval(header)
        rows = [n.args[0] for n in ast.walk(tree) if isinstance(n, ast.Call)
                and isinstance(n.func, ast.Attribute) and n.func.attr == 'offer'
                and isinstance(n.func.value, ast.Name) and n.func.value.id == 'csv_f'
                and n.args and isinstance(n.args[0], ast.List)]
        self.assertEqual(len(rows), 1)
        self.assertEqual(len(columns), len(rows[0].elts))
        i = columns.index('capture_frame')
        self.assertEqual(columns[i:i+4],
                         ['capture_frame', 'click_frame', 'click_used_frame', 'click_gap'])

    @unittest.skipUnless(shutil.which('node'), 'node is needed for browser JS check')
    def test_browser_click_uses_displayed_image_during_next_decode(self):
        script = _INDEX.decode().split('<script>', 1)[1].split('</script>', 1)[0]
        # Syntax-check the entire page, including unchanged controls.
        subprocess.run(['node', '--check'], input=script, text=True, check=True)
        display = script.split('// Runtime controls:', 1)[0]
        harness = r'''
const assert = require('assert');
let click, drawn, decode, frame=41, pending=0, maximum=0, posts=[];
const timers=[];
const canvas={width:640,height:512,
 getContext:()=>({drawImage:b=>{drawn=b.frame;}}),
 getBoundingClientRect:()=>({left:0,top:0,width:320,height:256}),
 addEventListener:(name,fn)=>{click=fn;}};
const msg={textContent:''};
global.document={getElementById:id=>id==='img'?canvas:msg};
global.setTimeout=fn=>{timers.push(fn);};
global.fetch=async (url,opts)=>{
 if(url==='/api/click'){posts.push(JSON.parse(opts.body));return {ok:true};}
 pending++;maximum=Math.max(maximum,pending);
 return {ok:true,headers:{get:k=>k==='X-Capture-Index'?String(frame):'session'},
 blob:async()=>({frame})};
};
global.createImageBitmap=blob=>new Promise(resolve=>{
 decode=()=>{pending--;resolve({frame:blob.frame,width:640,height:512,close(){}});};
});
async function flush(){for(let i=0;i<12;i++)await Promise.resolve();}
'''
        checks = r'''
(async()=>{
 await flush();
 await click({clientX:100,clientY:100});assert.equal(posts.length,0);
 decode();await flush();assert.equal(drawn,41);
 await click({clientX:100,clientY:100});
 assert.deepEqual(posts[0],{x:200,y:200,frame_index:41,session:'session'});
 frame=99;timers.shift()();await flush();
 await click({clientX:100,clientY:100});assert.equal(posts[1].frame_index,41);
 decode();await flush();
 await click({clientX:100,clientY:100});assert.equal(posts[2].frame_index,99);
 assert.equal(maximum,1);
})().catch(e=>{console.error(e);process.exitCode=1;});
'''
        subprocess.run(['node'], input=harness+display+checks, text=True, check=True)


if __name__ == '__main__':
    unittest.main()
