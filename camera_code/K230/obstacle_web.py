"""Small HTTP helpers and page for the standalone K230 obstacle preview."""
import json
import os
import time

DEFAULT_CONFIG = {"threshold": [8, 25, -14, 10, -8, 16], "min_pixels": 5000}


def validate_config(value):
    if not isinstance(value, dict) or set(value) != {"threshold", "min_pixels"}:
        raise ValueError("Expected threshold and min_pixels")
    threshold = value["threshold"]
    if not isinstance(threshold, list) or len(threshold) != 6:
        raise ValueError("LAB threshold requires six integers")
    for index, number in enumerate(threshold):
        low, high = (0, 100) if index < 2 else (-128, 127)
        if type(number) is not int or not low <= number <= high:
            raise ValueError("LAB value outside range")
    if any(threshold[index] > threshold[index + 1] for index in (0, 2, 4)):
        raise ValueError("Minimum must not exceed maximum")
    pixels = value["min_pixels"]
    if type(pixels) is not int or not 1 <= pixels <= 640 * 480:
        raise ValueError("min_pixels must be 1..307200")
    return {"threshold": threshold[:], "min_pixels": pixels}


def read_request(client):
    # CanMV blocking recv can wait for the full requested size, so read incrementally.
    client.setblocking(False)
    data = bytearray()
    started = time.ticks_ms()
    body_start = None
    body_length = 0
    while True:
        if time.ticks_diff(time.ticks_ms(), started) >= 2000:
            raise OSError(110)
        try:
            chunk = client.recv(512)
        except OSError as error:
            if error.args[0] not in (11, 35, 10035, 110):
                raise
            chunk = None
        if chunk == b"":
            raise ValueError("Incomplete request")
        if chunk:
            data.extend(chunk)
            if len(data) > 4096:
                raise ValueError("Request too large")
            if body_start is None:
                end = data.find(b"\r\n\r\n")
                if end >= 0:
                    headers = bytes(data[:end]).decode().split("\r\n")
                    first = headers[0].split()
                    if len(first) != 3:
                        raise ValueError("Invalid request line")
                    for header in headers[1:]:
                        name, _, value = header.partition(":")
                        if name.lower() == "content-length":
                            body_length = int(value.strip())
                    if not 0 <= body_length <= 512:
                        raise ValueError("Body too large")
                    body_start = end + 4
            if body_start is not None and len(data) >= body_start + body_length:
                return first[0], first[1].split("?", 1)[0], bytes(data[body_start:body_start + body_length])
        os.exitpoint()
        time.sleep_ms(1)


def send_all(client, data):
    view = memoryview(data)
    offset = 0
    progress = time.ticks_ms()
    while offset < len(view):
        if time.ticks_diff(time.ticks_ms(), progress) >= 2000:
            raise OSError(110)
        try:
            sent = client.send(view[offset:offset + 4096])
        except OSError as error:
            if error.args[0] not in (11, 35, 10035, 110):
                raise
            sent = None
        if sent == 0:
            raise OSError("Client disconnected")
        if sent:
            offset += sent
            progress = time.ticks_ms()
        else:
            os.exitpoint()
            time.sleep_ms(1)


def respond(client, status, content_type, body, detection=None):
    header = "HTTP/1.1 {}\r\nContent-Type: {}\r\nContent-Length: {}\r\n".format(
        status, content_type, len(body))
    if detection is not None:
        header += "X-Detection: {}\r\n".format(json.dumps(detection))
    header += "Cache-Control: no-store\r\nConnection: close\r\n\r\n"
    send_all(client, header.encode())
    send_all(client, body)


PAGE = r"""<!doctype html>
<html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>K230 黑色物块调试</title>
<style>
body{font-family:system-ui,sans-serif;margin:24px;background:#f4f5f7;color:#202630}
main{max-width:1080px;margin:auto}h1{font-size:24px}section{background:white;padding:20px;border-radius:10px;margin:16px 0}
img{display:block;width:100%;max-width:640px;aspect-ratio:4/3;background:#202630;object-fit:contain}
.layout{display:flex;gap:24px;flex-wrap:wrap}.controls{flex:1;min-width:240px}
label{display:inline-flex;flex-direction:column;gap:6px;margin:8px 12px 8px 0}input{width:90px;padding:8px;font:inherit}
button{padding:10px 18px;margin:10px 8px 0 0;font:inherit;cursor:pointer}pre{white-space:pre-wrap;word-break:break-all}
#status{min-height:44px}small{color:#596575}
</style><main><h1>K230 黑色物块调试</h1>
<p>仅相机检测，不控制车辆。只识别画面下方 2/3（y=160～479），选择其中最大的有效黑色色块。</p>
<section class="layout"><div><img id="frame" alt="等待 K230 画面"><p id="status">正在连接…</p><small id="rate"></small></div>
<div class="controls"><form id="form"><div id="fields"></div>
<label>最小黑色像素数<input id="pixels" type="number" min="1" max="307200" required></label><br>
<button id="apply" type="submit">应用参数</button></form><button id="pause" type="button">暂停画面</button>
<p id="message" role="status"></p><small>参数仅在本次运行有效，重启后恢复默认。应用参数会重新进行连续帧确认。</small>
<pre id="config"></pre></div></section></main>
<script>
const el=id=>document.getElementById(id),names=['L 最小','L 最大','A 最小','A 最大','B 最小','B 最大'];
names.forEach((name,i)=>{const label=document.createElement('label');label.textContent=name;
const input=document.createElement('input');input.id='t'+i;input.type='number';input.required=true;
input.min=i<2?0:-128;input.max=i<2?100:127;label.append(input);el('fields').append(label)});
let paused=false,objectURL=null,initialized=false;
async function request(path,options={}){const response=await fetch(path,{...options,cache:'no-store',signal:AbortSignal.timeout(6000)});
if(!response.ok)throw Error(await response.text());return response}
function showConfig(config){config.threshold.forEach((v,i)=>el('t'+i).value=v);el('pixels').value=config.min_pixels;
el('config').textContent='BLACK_THRESHOLD = ['+JSON.stringify(config.threshold)+']\nMIN_PIXELS = '+config.min_pixels}
async function refresh(){if(paused){setTimeout(refresh,200);return}const started=performance.now();
try{if(!initialized){showConfig(await(await request('/config')).json());initialized=true}
const response=await request('/frame.jpg?time='+Date.now()),result=JSON.parse(response.headers.get('X-Detection'));
const next=URL.createObjectURL(await response.blob());
await new Promise((resolve,reject)=>{el('frame').onload=resolve;el('frame').onerror=()=>{URL.revokeObjectURL(next);reject(Error('图像无法解码'))};el('frame').src=next});
if(objectURL)URL.revokeObjectURL(objectURL);objectURL=next;
el('status').textContent=result.found?(result.confirmed?'已连续确认':'候选物块')+'：中心 ('+result.u+', '+result.v+')，底部行 '+result.bottom+'，黑色像素 '+result.pixels:'未检测到有效黑色色块';
el('rate').textContent='本帧请求耗时 '+Math.round(performance.now()-started)+' ms';
}catch(error){el('status').textContent='连接或图像失败：'+(error.message||'图像无法解码')}
setTimeout(refresh,200)}
el('form').onsubmit=async event=>{event.preventDefault();el('apply').disabled=true;
try{const config={threshold:names.map((_,i)=>Number(el('t'+i).value)),min_pixels:Number(el('pixels').value)};
showConfig(await(await request('/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(config)})).json());
el('message').textContent='参数已应用'}catch(error){el('message').textContent='应用失败：'+error.message}
finally{el('apply').disabled=false}};
el('pause').onclick=()=>{paused=!paused;el('pause').textContent=paused?'继续画面':'暂停画面';if(paused)el('status').textContent='画面已暂停'};
refresh();
</script></html>""".encode("utf-8")
