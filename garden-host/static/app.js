// garden-host: live updates (Server-Sent Events) and live listening
// (WebSocket PCM into an AudioWorklet). Everything else is plain HTML.
'use strict';

// ----- live updates -----

const players = new Map(); // mic key -> player

function startEvents() {
  const page = document.body.dataset.page;
  const es = new EventSource('/events' + (page === 'add' ? '?scan=1' : ''));
  let broken = false;
  es.onerror = () => { broken = true; };
  // The server restarted or we missed updates: the page re-renders.
  es.onopen = () => { if (broken) location.reload(); };
  es.addEventListener('reload', () => location.reload());
  es.addEventListener('micset', () => { if (page === 'dashboard') location.reload(); });
  es.addEventListener('html', (e) => {
    const t = document.createElement('template');
    t.innerHTML = e.data;
    const el = t.content.firstElementChild;
    const old = el && el.id && document.getElementById(el.id);
    if (!old) return;
    old.replaceWith(el);
    for (const box of el.querySelectorAll('[data-listen]')) box.checked = players.has(box.dataset.listen);
  });
  es.addEventListener('meter', (e) => {
    const m = JSON.parse(e.data);
    for (const bar of document.querySelectorAll(`[data-level="${m.key}"]`)) bar.style.width = m.pct + '%';
    for (const t of document.querySelectorAll(`[data-level-text="${m.key}"]`)) t.textContent = m.text;
  });
  es.addEventListener('rssi', (e) => {
    const m = JSON.parse(e.data);
    for (const t of document.querySelectorAll(`[data-rssi="${m.key}"]`)) t.textContent = m.rssi;
  });
}

// ----- live listening -----

// A jitter buffer that plays mono PCM arriving at `rate` Hz, resampled
// linearly to the output rate (a no-op when they match, the usual case: the
// context is created at the stream's rate and the browser converts to the
// output device).
//
// One implementation serves both playback paths. AudioWorklet (below) is
// only defined in a secure context, i.e. https or localhost; a host app
// opened over plain http on a LAN address needs the ScriptProcessor
// fallback instead.
const PLAYER_CORE = `
class GardenPlayerCore {
  constructor(outRate) {
    this.buf = new Float32Array(1 << 17);
    this.mask = this.buf.length - 1;
    this.r = 0; this.w = 0; this.pos = 0;
    this.outRate = outRate;
    this.rate = outRate;
    this.playing = false;
  }
  setRate(rate) {
    if (rate > 0) this.rate = rate;
  }
  push(d) {
    for (let i = 0; i < d.length; i++) this.buf[(this.w + i) & this.mask] = d[i];
    this.w += d.length;
    // Cap latency: drop the oldest audio beyond 300 ms.
    const max = Math.round(this.rate * 0.3);
    if (this.w - this.r > max) { this.r = this.w - Math.round(this.rate * 0.08); this.pos = 0; }
  }
  render(out) {
    const step = this.rate / this.outRate;
    // Start (and restart after an underrun) with 80 ms buffered.
    if (!this.playing && this.w - this.r >= this.rate * 0.08) this.playing = true;
    for (let i = 0; i < out.length; i++) {
      if (!this.playing || this.w - this.r < 2) { this.playing = false; out[i] = 0; continue; }
      const a = this.buf[this.r & this.mask], b = this.buf[(this.r + 1) & this.mask];
      out[i] = a + (b - a) * this.pos;
      this.pos += step;
      const whole = Math.floor(this.pos);
      this.r += whole; this.pos -= whole;
    }
  }
}
`;

const WORKLET = `${PLAYER_CORE}
class GardenPlayer extends AudioWorkletProcessor {
  constructor() {
    super();
    this.core = new GardenPlayerCore(sampleRate);
    this.port.onmessage = (e) => {
      const d = e.data;
      if (typeof d === 'number') this.core.setRate(d);
      else this.core.push(d);
    };
  }
  process(inputs, outputs) {
    this.core.render(outputs[0][0]);
    return true;
  }
}
registerProcessor('garden-player', GardenPlayer);
`;

let workletUrl = null;

// The fallback path: same core, driven by a deprecated but universally
// supported ScriptProcessorNode(no secure context needed).
function makeScriptPlayer(ctx, rate) {
  const Core = new Function(`${PLAYER_CORE}\nreturn GardenPlayerCore;`)();
  const core = new Core(ctx.sampleRate);
  core.setRate(rate);
  const node = ctx.createScriptProcessor(2048, 0, 1);
  node.onaudioprocess = (e) => core.render(e.outputBuffer.getChannelData(0));
  return { node, push: (f) => core.push(f), setRate: (r) => core.setRate(r) };
}

// Live updates replace the mic cards, so this note lives outside them.
let noteText = '';
function audioNote(text) {
  text = text || '';
  if (text === noteText) return; // called for every audio frame
  noteText = text;
  const el = document.getElementById('audio-warning');
  if (!el) return;
  el.querySelector('span').textContent = text;
  el.hidden = !text;
}

async function startListening(key, box) {
  // Created in the click handler so autoplay rules allow it, at the rate the
  // stream is expected to use.
  const rate = Number(box.dataset.rate) || 24000;
  let ctx;
  try { ctx = new AudioContext({ sampleRate: rate, latencyHint: 'interactive' }); } catch { ctx = new AudioContext({ latencyHint: 'interactive' }); }
  const player = { ctx, ws: null, node: null };
  players.set(key, player);
  // AudioWorklet needs a secure context (https or localhost); otherwise use
  // the ScriptProcessor path with the same DSP.
  let feed;
  if (ctx.audioWorklet) {
    workletUrl ??= URL.createObjectURL(new Blob([WORKLET], { type: 'text/javascript' }));
    await ctx.audioWorklet.addModule(workletUrl);
    if (players.get(key) !== player) return; // stopped meanwhile
    const node = new AudioWorkletNode(ctx, 'garden-player', { outputChannelCount: [1] });
    node.port.postMessage(rate);
    feed = { push: (f) => node.port.postMessage(f, [f.buffer]), setRate: (r) => node.port.postMessage(r) };
    player.node = node;
  } else {
    const sp = makeScriptPlayer(ctx, rate);
    feed = sp;
    player.node = sp.node;
  }
  player.node.connect(ctx.destination);
  // Firefox and Safari may still start the context suspended; the click that
  // got here counts as a gesture, so resuming is allowed.
  if (ctx.state !== 'running') await ctx.resume().catch(() => {});
  if (ctx.state !== 'running') {
    audioNote('The browser is blocking audio. Allow sound for this site, then tick Listen again.');
  }
  ctx.onstatechange = () => {
    if (players.get(key) === player && ctx.state !== 'running') {
      audioNote('The browser paused the audio. Tick Listen again.');
    }
  };
  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  const ws = new WebSocket(`${proto}//${location.host}/mics/${key}/listen`);
  ws.binaryType = 'arraybuffer';
  player.ws = ws;
  ws.onmessage = (e) => {
    if (typeof e.data === 'string') {
      const msg = JSON.parse(e.data);
      if (msg.rate) feed.setRate(msg.rate);
      return;
    }
    audioNote(null);
    const s16 = new Int16Array(e.data);
    const f32 = new Float32Array(s16.length);
    for (let i = 0; i < s16.length; i++) f32[i] = s16[i] / 32768;
    feed.push(f32);
  };
  ws.onclose = () => {
    if (players.get(key) === player) {
      audioNote('Live audio stopped: the receiver stopped sending this mic.');
      stopListening(key);
    }
  };
}

function stopListening(key) {
  const p = players.get(key);
  if (!p) return;
  players.delete(key);
  if (p.ws) { p.ws.onclose = null; p.ws.close(); }
  p.ctx.close();
  for (const box of document.querySelectorAll(`[data-listen="${key}"]`)) box.checked = false;
}

document.addEventListener('change', (e) => {
  const box = e.target;
  const key = box.dataset && box.dataset.listen;
  if (!key) return;
  if (box.checked) {
    startListening(key, box).catch((err) => {
      console.error(err);
      audioNote(`Could not start listening: ${err.message || err}`);
      stopListening(key);
    });
  } else {
    stopListening(key);
  }
});

startEvents();
