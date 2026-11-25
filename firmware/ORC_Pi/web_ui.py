"""
web_ui.py — trang web giám sát chạy trên chính Raspberry Pi (không cần cài thêm gì).

Mở trên điện thoại / máy tính cùng mạng Wi-Fi:   http://<tên-máy>.local:8080   hoặc   http://<IP của Pi>:8080
Hiển thị: toạ độ x, y, yaw, bộ đếm chuột, tình trạng IMU / chuột, số lỗi, quỹ đạo xe và log (các dòng print của orc_pi.py).
Chỉ ĐỌC dữ liệu, không điều khiển robot. Không có mật khẩu: chỉ dùng trong mạng nội bộ của bạn.
"""
import collections
import json
import math
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

LOG_LINES = collections.deque(maxlen=500)        # (số thứ tự, giờ, nội dung)
_seq = [0]
_lock = threading.Lock()
T0 = time.monotonic()


class _Tee:
    """Chép mọi dòng print vào bộ nhớ đệm để trang web đọc; vẫn in ra terminal / journalctl như cũ."""

    def __init__(self, real):
        self.real = real
        self.buf = ""

    def write(self, s):
        self.real.write(s)
        self.buf += s
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            line = line.rstrip()
            # dòng trạng thái x=... in 2 lần/giây đã có trên trang (thẻ số liệu), không cần đưa vào log
            if line and not line.startswith("x="):
                with _lock:
                    _seq[0] += 1
                    LOG_LINES.append((_seq[0], time.strftime("%H:%M:%S"), line))
        return len(s)

    def flush(self):
        self.real.flush()

    def __getattr__(self, name):
        return getattr(self.real, name)


def install_log_tee():
    if not isinstance(sys.stdout, _Tee):
        sys.stdout = _Tee(sys.stdout)


def _state(bot):
    imu = bot.imu
    return {
        "x": round(bot.x, 1), "y": round(bot.y, 1), "yaw": round(math.degrees(bot.yaw), 2),
        "vx": round(bot.vx, 1), "vy": round(bot.vy, 1),
        "raw": [bot.raw_x, bot.raw_y], "gyro": round(bot.gyro_deg, 1),
        "imu_ok": bool(bot.imu_ok) and (not hasattr(imu, "age") or (imu.age() is not None and imu.age() < 1.0)), "mouse": bool(bot.mouse.connected), "cal": bool(bot.calibrating),
        "imu_name": imu.NAME, "imu_hz": round(getattr(bot, "imu_hz", 0.0)),
        "imu_temp": round(getattr(bot, "imu_temp", float("nan")), 1) if not math.isnan(getattr(bot, "imu_temp", float("nan"))) else None,
        "glitch": imu.glitch, "err": imu.err, "resets": imu.resets,
        "bias": [round(v, 3) for v in bot.bias],
        "esp_age": (round(time.monotonic() - bot.esp_last, 1) if getattr(bot, "esp_last", 0) else None), "esp_pings": getattr(bot, "esp_pings", 0),
        "tx": bot.seq, "uptime": round(time.monotonic() - T0),
    }


INDEX_HTML = r"""<!doctype html>
<html lang="vi"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ORC Pi — giám sát</title>
<style>
:root{--bg:#0f1318;--card:#181e26;--line:#2a323d;--tx:#e6ebf2;--mu:#8b97a8;--ok:#3ecf8e;--bad:#ff6b6b;--warn:#f5b84a;--ac:#6cb2ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font:14px/1.45 system-ui,Segoe UI,Roboto,sans-serif}
header{display:flex;flex-wrap:wrap;gap:10px;align-items:center;padding:12px 16px;border-bottom:1px solid var(--line)}
h1{font-size:16px;margin:0 8px 0 0}.badge{padding:3px 10px;border-radius:99px;border:1px solid var(--line);color:var(--mu);font-size:12px}
.ok{color:var(--ok);border-color:var(--ok)}.bad{color:var(--bad);border-color:var(--bad)}.warn{color:var(--warn);border-color:var(--warn)}
main{display:grid;gap:12px;padding:12px 16px;grid-template-columns:repeat(auto-fit,minmax(150px,1fr))}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:10px 12px}
.card small{color:var(--mu);display:block;font-size:11px;text-transform:uppercase;letter-spacing:.04em}
.card b{font-size:22px;font-variant-numeric:tabular-nums;font-weight:600}.card span{color:var(--mu);font-size:12px}
.wide{grid-column:1/-1}canvas{width:100%;height:240px;background:#0b0f14;border-radius:8px;display:block}
#log{height:300px;overflow:auto;background:#0b0f14;border-radius:8px;padding:8px 10px;font:12px/1.5 ui-monospace,Consolas,monospace;white-space:pre-wrap}
#log div.esp{color:#7fd6e8}#log.f-pi div.esp,#log.f-esp div.pi{display:none}#log div.e{color:var(--bad)}#log div.w{color:var(--warn)}#log i{color:var(--mu);font-style:normal;margin-right:8px}
button{background:var(--card);color:var(--tx);border:1px solid var(--line);border-radius:8px;padding:5px 12px;cursor:pointer}
button:hover{border-color:var(--ac)}.row{display:flex;gap:8px;align-items:center;justify-content:space-between;margin-bottom:8px}
</style></head><body>
<header><h1>ORC Pi</h1>
<span id="b_link" class="badge">đang kết nối…</span><span id="b_imu" class="badge">IMU</span>
<span id="b_mouse" class="badge">chuột</span><span id="b_esp" class="badge">ESP32#1</span><span id="b_cal" class="badge" hidden>đang cân gyro — giữ yên xe</span>
<span class="badge" id="b_up"></span></header>
<main>
<div class="card"><small>X (mm)</small><b id="x">–</b></div>
<div class="card"><small>Y (mm)</small><b id="y">–</b></div>
<div class="card"><small>Yaw (độ)</small><b id="yaw">–</b></div>
<div class="card"><small>Vận tốc (mm/s)</small><b id="v">–</b></div>
<div class="card"><small>Chuột raw x, y</small><b id="raw">–</b></div>
<div class="card"><small>Gyro cộng dồn (độ)</small><b id="gyro">–</b></div>
<div class="card"><small id="imu_t">IMU</small><b id="imu">–</b><span id="imu2"></span></div>
<div class="card"><small>Lỗi cảm biến</small><b id="err">–</b><span>nhiễu / lỗi I2C / reset</span></div>
<div class="card"><small>Bias gyro (dps)</small><b id="bias" style="font-size:15px">–</b></div>
<div class="card wide"><div class="row"><small style="margin:0">Quỹ đạo (mm)</small><button onclick="trail=[]">Xoá quỹ đạo</button></div><canvas id="cv"></canvas></div>
<div class="card wide"><div class="row"><small style="margin:0">Log</small><span><button onclick="flt('')">Tất cả</button> <button onclick="flt('f-pi')">Pi</button> <button onclick="flt('f-esp')">ESP32#1</button> <button onclick="dl()">Tải log</button> <button onclick="clr()">Xoá</button></span></div><div id="log"></div></div>
</main>
<script>
const $=id=>document.getElementById(id);let last=0,trail=[],lines=[],fails=0,cur=null;
function badge(el,txt,cls){el.textContent=txt;el.className='badge '+cls}
function fmt(s){const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return (h?h+'g ':'')+m+'p '+s%60+'s'}
function draw(){const c=$('cv'),r=c.getBoundingClientRect(),d=devicePixelRatio||1;c.width=r.width*d;c.height=r.height*d;const g=c.getContext('2d');g.scale(d,d);
 const W=r.width,H=r.height;g.clearRect(0,0,W,H);if(!trail.length)return;
 let x0=Math.min(...trail.map(p=>p[0])),x1=Math.max(...trail.map(p=>p[0])),y0=Math.min(...trail.map(p=>p[1])),y1=Math.max(...trail.map(p=>p[1]));
 const pad=24,span=Math.max(x1-x0,y1-y0,200),s=Math.min((W-2*pad)/span,(H-2*pad)/span);
 const cx=(x0+x1)/2,cy=(y0+y1)/2,X=x=>W/2+(x-cx)*s,Y=y=>H/2-(y-cy)*s;
 g.strokeStyle='#2a323d';g.lineWidth=1;g.beginPath();g.moveTo(X(0),0);g.lineTo(X(0),H);g.moveTo(0,Y(0));g.lineTo(W,Y(0));g.stroke();
 g.strokeStyle='#6cb2ff';g.lineWidth=2;g.beginPath();trail.forEach((p,i)=>i?g.lineTo(X(p[0]),Y(p[1])):g.moveTo(X(p[0]),Y(p[1])));g.stroke();
 if(cur){const px=X(cur.x),py=Y(cur.y),a=cur.yaw*Math.PI/180;g.fillStyle='#3ecf8e';g.beginPath();g.arc(px,py,6,0,7);g.fill();
  g.strokeStyle='#3ecf8e';g.beginPath();g.moveTo(px,py);g.lineTo(px+Math.cos(a)*18,py-Math.sin(a)*18);g.stroke();}}
function addLog(seq,t,txt){const d=document.createElement('div');const u=txt.toLowerCase();
 d.className=txt.startsWith('[ESP32#1]')?'esp':'pi';
 if(/lỗi|thất bại|mất|!!|reset|bị động/.test(u))d.className+=' e';else if(/cảnh báo/.test(u))d.className+=' w';
 d.innerHTML='<i>'+t+'</i>';d.appendChild(document.createTextNode(txt));const L=$('log'),b=L.scrollTop+L.clientHeight>=L.scrollHeight-30;
 L.appendChild(d);while(L.childNodes.length>500)L.removeChild(L.firstChild);if(b)L.scrollTop=L.scrollHeight;lines.push(t+' '+txt)}
function flt(c){$('log').className=c;$('log').scrollTop=$('log').scrollHeight}
function dl(){const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([lines.join('\n')],{type:'text/plain'}));a.download='orc_pi_log.txt';a.click()}
function clr(){$('log').innerHTML='';lines=[]}
async function tick(){try{const r=await fetch('/api?since='+last,{cache:'no-store'});const j=await r.json();fails=0;const s=j.state;cur=s;
 badge($('b_link'),'đang nhận dữ liệu','ok');badge($('b_imu'),s.imu_name+(s.imu_ok?' OK':' LỖI'),s.imu_ok?'ok':'bad');
 badge($('b_mouse'),s.mouse?'chuột OK':'chưa thấy chuột',s.mouse?'ok':'bad');const ea=s.esp_age;badge($('b_esp'),ea===null?'ESP32#1: chưa nhận gì':(ea<3?'ESP32#1 → Pi OK':'ESP32#1 im lặng '+ea+'s'),ea===null?'warn':(ea<3?'ok':'bad'));
 $('b_cal').hidden=!s.cal;$('b_up').textContent='chạy '+fmt(s.uptime);
 $('x').textContent=s.x;$('y').textContent=s.y;$('yaw').textContent=s.yaw;$('v').textContent=Math.hypot(s.vx,s.vy).toFixed(0);
 $('raw').textContent=s.raw[0]+', '+s.raw[1];$('gyro').textContent=s.gyro;$('imu_t').textContent=s.imu_name;
 $('imu').textContent=s.imu_hz+' Hz';$('imu2').textContent=s.imu_temp===null?'':s.imu_temp+' °C';
 $('err').textContent=s.glitch+' / '+s.err+' / '+s.resets;$('bias').textContent=s.bias.join('  ');
 const l=trail[trail.length-1];if(!l||Math.hypot(s.x-l[0],s.y-l[1])>5){trail.push([s.x,s.y]);if(trail.length>1500)trail.shift()}
 j.log.forEach(e=>{addLog(e[0],e[1],e[2]);last=Math.max(last,e[0])});draw()}
 catch(e){fails++;if(fails>2)badge($('b_link'),'mất kết nối tới Pi','bad')}setTimeout(tick,500)}
tick();addEventListener('resize',draw);
</script></body></html>"""


def _make_handler(bot):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            if u.path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", INDEX_HTML.encode("utf-8"))
            elif u.path == "/api":
                try:
                    since = int(parse_qs(u.query).get("since", ["0"])[0])
                except ValueError:
                    since = 0
                with _lock:
                    new = [list(e) for e in LOG_LINES if e[0] > since]
                try:
                    body = json.dumps({"state": _state(bot), "log": new}, ensure_ascii=False).encode("utf-8")
                except Exception as e:                 # không để lỗi web làm sập robot
                    body = json.dumps({"error": str(e)}).encode("utf-8")
                self._send(200, "application/json; charset=utf-8", body)
            else:
                self._send(404, "text/plain; charset=utf-8", b"not found")

    return Handler


def start_web(bot, port=8080):
    """Chạy máy chủ web ở luồng nền. Lỗi (ví dụ cổng bận) chỉ in cảnh báo, robot vẫn chạy bình thường."""
    try:
        srv = ThreadingHTTPServer(("0.0.0.0", port), _make_handler(bot))
    except OSError as e:
        print(f"[WEB] CẢNH BÁO: không mở được cổng {port}: {e}")
        return None
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        host = socket.gethostname()
        ip = ""
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("10.255.255.255", 1))
            ip = s.getsockname()[0]
        finally:
            s.close()
    except OSError:
        host, ip = socket.gethostname(), ""
    print(f"[WEB] Trang giám sát: http://{host}.local:{port}" + (f"  hoặc  http://{ip}:{port}" if ip else ""))
    return srv
