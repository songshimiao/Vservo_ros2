(function frontend(){
'use strict';
const $ = id => document.getElementById(id);
const roles = ['eih', 'eth'];
const kinds = ['intrinsic', 'handeye'];
const labels = {eih: 'EIH · 305', eth: 'ETH · 335', intrinsic: '内参', handeye: '手眼'};
const boardProfiles = {a4: 'A4 · 266 × 190 mm · 38 mm 格', compact: '紧凑板 · 175 × 125 mm · 25 mm 格',
  legacy: '旧板 · 105 × 75 mm · 15 mm 格'};
const checks = [
  '已实测格长、标记尺寸与打印比例', '已核对相机序列号、USB3 供电和线缆应力释放',
  '当前相机在本阶段采样姿态中能稳定看到标定板', '机器人使用法兰 tool0，未混用带偏置的 TCP',
  '机器人桥为 READ_ONLY，法兰 TF 连续', '按本阶段固定板，拍摄前停稳至少 1 秒'
];
let view = 'overview';
let state = null;
let token = '';
let selected = {};
try { selected = JSON.parse(localStorage.getItem('vservo-calibration') || '{}') || {}; } catch {}
let wizard = {role: 'eih', purpose: 'train', session: null, confirmed: new Set(), result: null, boardProfile: 'a4'};
let gallery = null;
let busy = false;
let speedEdited = false;
let refreshedPage = false;
const commandValues = new Map();
const urls = new WeakMap();
const rendered = new WeakMap();
const esc = value => String(value ?? '').replace(/[&<>"']/g, char =>
  ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'}[char]));
const badge = (text, tone = 'neutral') => `<span class="badge ${tone}">${esc(text)}</span>`;
const fmt = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : '—';
const localTime = value => Number.isFinite(value) ? new Date(value * 1000).toLocaleString('zh-CN', {hour12: false}) : '—';
const pre = data => `<pre>${esc(JSON.stringify(data, null, 2))}</pre>`;
const path = (name, params = {}) => `/api/${name}?${new URLSearchParams(params)}`;

function boardLabel(board) {
  if (!board) return '规格未记录';
  const mm = value => (value * 1000).toLocaleString('zh-CN', {maximumFractionDigits: 3});
  return `${board.squares_x} × ${board.squares_y} 格 · 格长 ${mm(board.square_m)} mm · 标记边长 ${mm(board.marker_m)} mm`;
}

function renderHTML(el, html) {
  if (typeof el === 'string') el = $(el);
  if (rendered.get(el) !== html) {
    el.innerHTML = html;
    rendered.set(el, html);
  }
}
async function api(url, body) {
  const options = {cache: 'no-store', signal: AbortSignal.timeout(body === undefined ? 5000 : 47000)};
  if (body !== undefined) {
    options.method = 'POST';
    options.headers = {'Content-Type': 'application/json', 'X-Vservo-Console': token};
    options.body = JSON.stringify(body);
  }
  const response = await fetch(url, options);
  const data = await response.json();
  if (!response.ok) throw Error(data.error || `HTTP ${response.status}`);
  return data;
}
function notice(text, tone = 'good') {
  $('toast').textContent = text;
  $('toast').className = `toast ${tone}`;
  $('toast').hidden = false;
}
function commandCards(commands, namespace) {
  return Object.entries(commands || {}).map(([label, cmd], i) => {
    const key = `${namespace}-${i}`;
    commandValues.set(key, cmd);
    return `<div class="command"><div class="row"><strong>${esc(label)}</strong><button data-copy="${key}">复制</button></div><pre>${esc(cmd)}</pre></div>`;
  }).join('');
}
function cameraMeta(camera) {
  if (!camera) return '等待 ROS 图像和 metadata';
  const meta = camera.metadata || {};
  return `${badge(camera.online ? '在线' : '离线', camera.online ? 'good' : 'bad')}
    ${esc(meta.camera?.serial || '未知序列号')} · ${esc(camera.size?.join(' × ') || '未知尺寸')} · ${fmt(camera.hz, 1)} Hz<br>
    最近帧 ${localTime(camera.stamp)} · 年龄 ${fmt(camera.age_s)} s<br>
    ${esc(meta.source || '等待 metadata')} · ${meta.validated === true ? '已验证内参已加载' : '未加载已验证内参'}`;
}
function fileText(file) {
  if (!file) return '等待文件检查';
  if (file.compatible === true) return '已完成 · 可跳过';
  if (file.valid && file.compatible === null) return '文件通过 · 等待相机';
  return file.exists ? '需核对或重标' : '未标定';
}
function fileTone(file) {
  return file?.compatible === true ? 'good' : file?.valid ? 'warn' : 'bad';
}
function keyFor(role, kind) { return `${role}_${kind}`; }
function setSelected(role, kind, value) {
  selected[keyFor(role, kind)] = value;
  localStorage.setItem('vservo-calibration', JSON.stringify(selected));
  void refreshState();
}
function workflow() {
  const stages = [
    ['双路内参', 'intrinsic', 'train', roles],
    ['独立内参验证', 'intrinsic', 'check', roles],
    ['EIH 手眼标定', 'handeye', 'train', ['eih']],
    ['EIH 独立手眼验证', 'handeye', 'check', ['eih']],
    ['ETH 手眼标定', 'handeye', 'train', ['eth']],
    ['ETH 独立手眼验证', 'handeye', 'check', ['eth']]
  ];
  let blocked = false;
  const rows = stages.map(([label, kind, purpose, cameraRoles]) => {
    const issues = cameraRoles.filter(role => {
      const file = state.files[role][kind];
      return purpose === 'check' ? !file.verification || file.compatible !== true : file.compatible !== true;
    });
    const done = issues.length === 0;
    const tone = done ? 'good' : blocked ? 'neutral' : 'warn';
    const status = done ? '已完成 / 可跳过' : blocked ? '待前序步骤' : '当前步骤';
    if (!done) blocked = true;
    const role = issues[0] || cameraRoles[0];
    const detail = done ? '结果与在线相机匹配' : issues.map(r => {
      const f = state.files[r][kind];
      return `${r.toUpperCase()} ${purpose === 'check' && f.compatible === true ? '缺少已通过的独立报告' : f.reason}`;
    }).join('；');
    return `<div class="step"><strong>${label} ${badge(status, tone)}</strong><div class="muted">${esc(detail)}</div><button data-view="${kind}" data-role="${role}" data-purpose="${purpose}">${done ? '查看 / 重做' : '进入步骤'}</button></div>`;
  });
  const robot = state.values.robot;
  const monitorReady = !!state.values.localization && (['READ_ONLY', 'DISARMED', 'ARMED'].includes(robot) || state.values.follow?.enabled);
  rows.push(`<div class="step"><strong>双画面伺服 ${badge(monitorReady ? '监控在线' : '等待启动', monitorReady ? 'good' : 'warn')}</strong><div class="muted">${blocked ? '检查前序标定及验证。' : '在监控页启动双画面并观察单标记定位。'}</div><button data-view="servo">打开监控</button></div>`);
  renderHTML('workflow', rows.join(''));
}
function renderOverview() {
  renderHTML('overview-cameras', roles.map(r =>
    `<article class="panel"><h3>${labels[r]}</h3><div class="camera-meta">${cameraMeta(state.cameras[r])}</div></article>`).join(''));
  workflow();
  renderHTML('files', roles.flatMap(role => kinds.map(kind => {
    const file = state.files[role][kind];
    const candidates = state.candidates[role][kind];
    const options = candidates.map(c =>
      `<option value="${esc(c.path)}" ${c.path === file.path ? 'selected' : ''}>${esc(c.path)} · ${esc(fileText(c))}</option>`).join('');
    const comparison = candidates.filter(c => c.exists).map(c =>
      `<p><code>${esc(c.path)}</code> · ${esc(c.serial || '未知序列号')} · ${localTime(c.modified)}</p>${pre(c.metrics || {状态: c.reason})}`).join('');
    return `<div class="file-group"><label>${labels[role]} ${labels[kind]} ${badge(fileText(file), fileTone(file))}
      <select data-select-role="${role}" data-select-kind="${kind}">${options}</select></label>
      <p>${esc(file.reason)} · ${file.verification ? `独立报告 ${esc(file.verification)}` : '等待独立验证'}</p>
      ${comparison ? `<details><summary>查看标定对比</summary><div>${comparison}</div></details>` : ''}</div>`;
  })).join(''));
}
function buildWizard(kind) {
  const root = $(kind).querySelector('.wizard');
  root.innerHTML = `<div class="panel">
    <div class="actions wizard-controls"></div><div class="wizard-file"></div>
    ${kind === 'handeye' ? `<fieldset class="checklist"><legend>本会话装配检查</legend>${checks.map((text, i) => `<label><input type="checkbox" data-check="${i}"> ${text}</label>`).join('')}</fieldset>` : ''}
    </div><div class="grid">
      <div class="panel"><h3 class="wizard-title"></h3><div class="viewport" data-camera="eih" data-variant="raw"><img alt="标定相机预览" hidden><span>等待图像</span></div><div class="camera-meta wizard-camera"></div></div>
      <div class="panel"><h3>实时采集条件</h3><div class="wizard-preview"></div><div class="wizard-actions"></div><div class="wizard-result" role="status"></div></div>
    </div><details class="panel"><summary>会话备用命令</summary><div class="wizard-commands"></div></details>`;
}
function renderWizard(kind) {
  const root = $(kind).querySelector('.wizard');
  const find = selector => root.querySelector(selector);
  const {role, purpose, session, confirmed, result} = wizard;
  const file = state?.files?.[role]?.[kind];
  const camera = state?.cameras?.[role];
  const preview = state?.values?.[`${role}_preview`];
  const hasPreview = preview?.mode === kind && preview.role === role && state.calibration_nodes[role] === 1;
  const locked = kind === 'handeye' && confirmed.size !== checks.length;
  const service = state?.services?.[role] || {};
  const validResult = result?.output && result?.result?.valid;
  const outputPath = session && `sessions/${session.name}/${role}_${kind === 'intrinsic' ? 'intrinsics' : 'handeye'}.json`;
  const match = !!(session && hasPreview && preview.session_dir?.endsWith(`/${session.name}`) &&
    preview.output?.endsWith(`/${outputPath}`));
  const boardMismatch = match && session.plan.board &&
    Object.entries(session.plan.board).some(([key, value]) => preview.board?.[key] !== value);
  const fileOutput = state?.candidates?.[role]?.[kind]?.find(c => c.path === outputPath);
  const solved = validResult || !!fileOutput?.exists;
  const blockers = [];
  if (!session) blockers.push('先创建或打开 Web 会话');
  else if (!match || !service.capture) blockers.push('启动本会话的采集节点并等待连接');
  if (boardMismatch) blockers.push('节点板规格与会话不匹配，重启本会话采集');
  if (locked) blockers.push('勾选全部装配检查项');
  if (kind === 'handeye' && state?.values?.robot !== 'READ_ONLY') blockers.push('等待只读机器人桥');
  if (kind === 'handeye' && state?.values?.[`${roles.find(r => r !== role)}_preview`]?.mode === 'handeye') {
    blockers.push('等待另一台相机的手眼采集节点停止');
  }
  if (match && !preview.ready) blockers.push(preview.reason || '等待有效采样帧');
  const captureReady = !blockers.length;
  renderHTML(find('.wizard-controls'), `${roles.map(r => `<button data-action="role" data-role="${r}" aria-pressed="${r === role}">${labels[r]}</button>`).join('')}
    <button data-action="purpose" data-purpose="train" aria-pressed="${purpose === 'train'}">采集与求解</button>
    <button data-action="purpose" data-purpose="check" aria-pressed="${purpose === 'check'}">独立验证</button>
    <button class="primary" data-action="new" ${!state || busy || (purpose === 'check' && !file?.valid) ? 'disabled' : ''}>${purpose === 'check' ? '创建独立验证会话' : file?.valid ? '重新标定 · 新建会话' : '新建标定会话'}</button>`);
  renderHTML(find('.wizard-file'), `<p>${badge(fileText(file), fileTone(file))} ${esc(file?.reason || '等待状态')}
    ${file?.path ? `<code>${esc(file.path)}</code>` : ''}</p>${file?.compatible === true ?
    `<button data-action="next" data-kind="${kind}">${purpose === 'train' ? '跳过已标定 · 进入独立验证' : file.verification ? '跳过已验证步骤 · 返回总览' : '返回总览'}</button>` : ''}
    ${purpose === 'check' ? `<p>验证会话沿用所选结果的板规格，采满至少 5 ${kind === 'intrinsic' ? '张' : '组'}后运行验证。${kind === 'handeye' ? '验证应保持训练时板的固定关系。' : ''}</p>` :
      `<label>新会话标定板 <select id="board-profile">${Object.entries(boardProfiles).map(([id, label]) =>
        `<option value="${id}" ${id === wizard.boardProfile ? 'selected' : ''}>${label}</option>`).join('')}</select></label>
      <p><a href="/charuco-a4.pdf" download>下载 A4 板</a> · <a href="/charuco-compact.pdf" download>下载紧凑板</a></p>`}
    ${session || (purpose === 'check' && file?.board) ? `<p>${session ? '当前会话' : '所选结果'}板规格 ${esc(boardLabel(session?.board || file?.board))}</p>` : ''}`);
  find('.wizard-title').textContent = `${labels[role]} · ${hasPreview ? 'ChArUco 角点预览' : '原始图像'}`;
  const viewport = find('.viewport');
  const variant = hasPreview && preview.stamp_ns ? 'overlay' : 'raw';
  if (viewport.dataset.camera !== role || viewport.dataset.variant !== variant) clearImage(viewport, '等待图像');
  viewport.dataset.camera = role;
  viewport.dataset.variant = variant;
  renderHTML(find('.wizard-camera'), cameraMeta(camera));
  for (const input of root.querySelectorAll('[data-check]')) input.checked = confirmed.has(Number(input.dataset.check));
  const focusHint = preview?.sharpness === undefined ? '—' : `${fmt(preview.sharpness, 1)}${preview.sharpness < 25 ? '（疑似模糊）' : ''}`;
  const lightHint = preview?.bright_fraction === undefined ? '—' : `${fmt(100*preview.bright_fraction, 1)}%${preview.bright_fraction > .3 ? '（检查反光或曝光）' : ''}`;
  renderHTML(find('.wizard-preview'), `<p>当前会话 <code>${esc(session?.name || '未创建')}</code></p>
    <p>${badge(captureReady && !busy ? '可采集' : '待改善', captureReady && !busy ? 'good' : 'warn')} ${esc(busy ? '等待操作返回' : blockers.join(' · '))}</p>
    <p>${badge(service.capture ? '采集服务可用' : '采集服务未发现', service.capture ? 'good' : 'warn')} ${badge(service.solve ? '求解服务可用' : '求解服务未发现', service.solve ? 'good' : 'warn')}</p>
    ${hasPreview ? `<p>${preview.corners} 个角点 · ${preview.sample_count} 个已保存样本${match ? '' : '（其他会话）'}</p>
      <p>预览帧 ${localTime(preview.stamp_ns / 1e9)} · 年龄 ${fmt((state.ros_now - preview.stamp_ns/1e9))} s · 清晰度 ${focusHint} · 高亮像素 ${lightHint}。两项仅作观察提示。</p>
      ${kind === 'handeye' ? `<p>法兰静止 ${preview.still === null ? '等待 TF 或有效检测' : preview.still ? '通过' : '未通过'} · ${fmt(preview.flange_mm)} mm / ${fmt(preview.flange_deg)}° · 最新 TF ${state.tf.ok ? '有效' : '不可用'}</p>` : ''}
      ` : '<p>采集节点连接后显示角点叠加和质量信息。</p>'}`);
  const actions = find('.wizard-actions');
  renderHTML(actions, `<div class="actions">
    <button data-action="start-calibration">启动本会话采集</button>
    <button class="primary" data-action="capture">保存有效${kind === 'intrinsic' ? '图片' : '样本'}</button>
    ${purpose === 'train' ? `<button data-action="solve">求解${labels[kind]}</button>` : '<button data-action="verify">运行独立验证</button>'}
    ${solved && (validResult || fileOutput?.valid) ? `<button data-action="adopt" data-file="${esc(outputPath)}">在本页选用新结果</button>` : ''}
    ${session ? `<button data-action="samples">查看样本</button>` : ''}</div>`);
  actions.querySelector('[data-action="start-calibration"]').disabled = !session || busy;
  actions.querySelector('[data-action="capture"]').disabled = !captureReady || busy;
  const solve = actions.querySelector('[data-action="solve"]');
  if (solve) solve.disabled = !match || boardMismatch || busy || locked || !service.solve || solved;
  const verify = actions.querySelector('[data-action="verify"]');
  if (verify) verify.disabled = !session || busy || (match ? preview.sample_count : session.samples.length) < 5;
  const explanation = result?.success === false ? `<p>检查角点数量、图像时效、重复帧、已加载内参、TF 与停稳情况。求解失败时检查样本数及姿态覆盖。</p>` : '';
  renderHTML(find('.wizard-result'), busy ? '<p class="notice warn">等待 ROS 服务返回…</p>' : result ?
    `<div class="notice ${result.success ? 'good' : 'bad'}"><strong>${result.success ? '成功' : '失败'}</strong> · ${esc(result.message)}</div>
      ${explanation}${result.result ? pre({输出: result.output, 状态: result.result.reason, ...result.result.metrics}) : ''}
      ${result.report ? `<a href="${esc(path('report', {name: result.report, download: 1}))}">下载验证报告</a>${pre(result.metrics)}` : ''}` : '');
  renderHTML(find('.wizard-commands'), session ? commandCards(session.commands, kind) :
    `<p>创建会话后生成独立路径。${purpose === 'check' ? '验证脚本读取会话清单、所选标定文件并写入独立报告。' : '启动采集节点后可在此调用采集和求解。'}</p>`);
}
function renderServo() {
  const values = state?.values || {};
  for (const role of roles) renderHTML(`servo-${role}`, cameraMeta(state?.cameras?.[role]));
  const robot = values.robot || '等待机器人状态';
  const follow = values.follow;
  const timing = follow?.fault_timing ?? follow?.control_timing;
  $('servo-start-hint').textContent = state?.allow_motion ?
    '启动监控后自动进入持续伺服，双路丢失目标会复位。' : '当前为只读监控。';
  const running = !!state?.processes?.servo?.running;
  const canStart = state?.arm_available && !follow?.enabled;
  const speed = $('servo-speed');
  if (!speedEdited) {
    speed.value = state?.speed_percent ?? 100;
    const profile = state?.motion_profile;
    $('servo-linear').value = profile?.linear_mm_s ?? 100;
    $('servo-angular').value = profile?.angular_deg_s ?? 30;
    $('servo-accel').value = profile?.joint_accel_rad_s2 ?? 0.5;
  }
  $('servo-speed-draft').textContent = `${speed.value}%`;
  const applied = values.speed?.percent;
  $('servo-speed-current').textContent = Number.isFinite(applied) ?
    `桥接已生效 ${fmt(applied, 0)}% · ${fmt(values.speed.effective_linear_mm_s, 1)} mm/s · ${fmt(values.speed.effective_angular_deg_s, 1)}°/s · 关节上限 ${fmt(values.speed.max_joint_speed_rad_s, 3)} rad/s · 加速度 ${fmt(values.speed.max_joint_accel_rad_s2, 2)} rad/s²` :
    `下次启动 ${fmt(state?.speed_percent ?? 100, 0)}%`;
  $('servo').querySelector('[data-action="speed"]').disabled = !state || busy ||
    (running && !state.speed_available);
  const actions = $('servo-actions');
  renderHTML(actions, `<button data-action="start-monitor">启动或重载监控</button>
    <button data-action="stop-monitor">结束监控</button>
    ${state?.allow_motion ? '<button class="primary" data-action="start-motion">恢复持续伺服</button><button data-action="stop-motion">停止伺服</button>' : '<span>只读模式</span>'}`);
  actions.querySelector('[data-action="start-monitor"]').disabled = !state || busy;
  actions.querySelector('[data-action="stop-monitor"]').disabled = !running || busy;
  const start = actions.querySelector('[data-action="start-motion"]');
  if (start) start.disabled = !running || busy || !canStart;
  const stop = actions.querySelector('[data-action="stop-motion"]');
  if (stop) stop.disabled = !running || busy || !state.arm_available;
  const modes = {FOLLOWING_ETH: 'ETH 主相机跟随', FOLLOWING_EIH: 'EIH 主相机跟随', RETURNING_HOME: '双路丢失 · 正在复位',
    BRAKING_FOR_HOME: '双路丢失 · 正在减速', PREPARING_HOME: '双路丢失 · 正在检查复位路径',
    WAITING_TARGET: '持续伺服 · 等待目标', WAITING_COMMAND: '持续伺服 · 等待有效指令',
    WAITING_LOCALIZATION: '持续伺服 · 等待定位节点', DISARMED: '伺服已停止', READ_ONLY: '只读监控'};
  renderHTML('servo-mode', esc(robot.startsWith('FAULT') ? '伺服故障' : modes[robot] || robot));
  $('servo-mode').className = `badge ${robot.startsWith('FAULT') ? 'bad' :
    ['BRAKING_FOR_HOME', 'PREPARING_HOME', 'RETURNING_HOME'].includes(robot) ? 'warn' : follow?.enabled ? 'good' : 'neutral'}`;
  const mode = values.localization || '等待定位状态';
  const agreement = values.agreement;
  const difference = agreement?.translation_mm;
  const consistent = Number.isFinite(difference) && difference <= agreement.agreement_mm &&
    agreement.rotation_deg <= agreement.agreement_deg;
  const source = agreement?.source === 'eth' ? 'ETH 主相机' : agreement?.source === 'eih' ? 'EIH 主相机' : '';
  const reason = {
    waiting_eth: '等待 ETH 获取目标', eth_tracking: 'ETH 引导接近目标',
    waiting_eih: 'ETH 数据已超时，等待 EIH 捕获',
    eih_confirming: `EIH 近距确认 ${agreement?.handoff_count ?? 0} / ${agreement?.handoff_confirmations ?? 3}`,
    eih_tracking: 'EIH 主相机持续跟随', eih_lost: 'EIH 丢失或超距，短暂停留等待重新捕获',
    eth_primary: 'ETH 目标有效，优先跟随 ETH', eih_fallback: 'ETH 丢失，使用 EIH 目标继续跟随',
    eth_leading: 'ETH 当前主导，持续可见时保持主导权', eih_leading: 'EIH 当前主导，持续可见时保持主导权',
    both_lost: '双路没有有效目标，持续伺服将执行复位并等待重新捕获',
    waiting_tf: '相机已检测到目标，等待图像时刻的定位变换，保持等待并暂停复位',
    pair_mismatch: '同步双路位姿超差，暂停输出'
  }[agreement?.reason];
  const pairLabel = Number.isFinite(difference) ? (consistent ? '双路核对一致' : 'ETH 辅助核对存在偏差') :
    agreement?.pair_status === 'skew' ? '两路时间差超限' : '无同步双路观测';
  const pose = data => data ? pre({坐标系: data.frame, 位置_m: data.position_m, 四元数_xyzw: data.quaternion_xyzw}) : '<p>等待有效位姿或位姿已超时。</p>';
  renderHTML('servo-state', `<div class="panel"><h3>定位来源与跟随</h3>
    ${badge(mode, ['COARSE', 'FINE'].includes(mode) ? 'good' : mode === 'INCONSISTENT' ? 'bad' : 'warn')}
    ${source ? badge(source, 'good') : ''}
    ${badge(pairLabel, Number.isFinite(difference) ? consistent ? 'good' : 'warn' : 'neutral')}
    <p>${agreement?.selection_policy === 'first_seen' ? '先看到的相机先主导，主相机丢失后由仍有目标的相机接管' :
      agreement?.selection_policy === 'eth_primary' ? 'ETH 优先，ETH 丢失时 EIH 接管，双路偏差仅供观察' :
      agreement?.pair_enforced ? '启用了双路一致性阻断' : 'ETH 提供接近引导与辅助观测，EIH 接管后独立跟随'}</p>
    ${follow?.fault ? `<p class="notice bad">${esc(follow.fault)}</p>` : ''}
    ${Number.isFinite(follow?.localization_age_s) ? `<p>定位心跳年龄 ${fmt(follow.localization_age_s*1000, 0)} ms / 超时门槛 ${fmt(follow.localization_timeout_s*1000, 0)} ms</p>` : ''}
    ${Number.isFinite(timing?.period_ms) ? `<p>指令间隔 ${fmt(timing.command_gap_ms, 1)} ms / 门槛 ${fmt(timing.timeout_ms, 0)} ms · 控制周期 ${fmt(timing.period_ms, 1)} ms · 循环耗时 ${fmt(timing.work_ms, 1)} ms · ServoJ 调用 ${fmt(timing.servoj_ms, 1)} ms · 使能 ${fmt(timing.enable_ms, 1)} ms${follow?.fault_timing ? '（故障时记录）' : ''}</p>` : ''}
    ${follow?.reason ? `<p>${esc(follow.reason)}</p>` : ''}
    ${follow?.enabled ? `<p>双路丢失复位角度 ${esc((follow.home_joint_deg || [90, -110, 110, -180, -90, 0]).map(v => `${fmt(v, 0)}°`).join('、'))}。复位完成后等待并自动恢复跟随。</p>` : ''}
    ${reason ? `<p>${esc(reason)}</p>` : ''}
    ${roles.map(role => {
      const camera = agreement?.cameras?.[role];
      if (!camera) return '';
      const status = camera.localized ? '目标定位可用' : camera.visible ? '已检测到目标，等待定位变换' : '未收到有效目标';
      return `<p>${role.toUpperCase()}：${esc(status)} · TF ${camera.tf_ready ? '可用' : '缺失'}${camera.error ? `<br>${esc(camera.error)}` : ''}</p>`;
    }).join('')}
    <p>辅助位置差 ${fmt(difference)} mm（参考 ${fmt(agreement?.agreement_mm)} mm） · 姿态差 ${fmt(agreement?.rotation_deg)}°（参考 ${fmt(agreement?.agreement_deg)}°） · 305 距离 ${fmt(agreement?.eih_distance_m)} m</p>
    <p>配对时间差 ${fmt(agreement?.pair_skew_s, 3)} / ${fmt(agreement?.max_skew_s, 3)} s · 观测时效门槛 ${fmt(agreement?.max_age_s)} s</p>
    <p>305 检测 ${esc(values.eih_detector || '等待')} · 335 检测 ${esc(values.eth_detector || '等待')}</p>
    <p>伺服 ${esc(values.servo || '等待状态')} · 机器人 ${esc(robot)}</p></div>
    <div class="grid"><div class="panel"><h3>定位目标</h3>${pose(values.target)}</div><div class="panel"><h3>下一步法兰目标</h3>${pose(values.next_flange)}</div></div>`);
}
function render() {
  if (state && view === 'overview') renderOverview();
  $('overview').querySelector('[data-action="start-cameras"]').disabled = !state || busy || !!state.processes?.servo?.running;
  if (kinds.includes(view)) renderWizard(view);
  if (view === 'servo') renderServo();
  if (view === 'diagnostics') renderHTML('process-logs', Object.entries(state?.processes || {}).map(([name, proc]) =>
    `<button data-action="process-log" data-name="${esc(name)}">${esc(name)} · ${proc.running ? '已启动' : `退出 ${proc.exit_code}`} · 查看日志</button>`).join(''));
  renderHTML('commands', state ? commandCards({...state.commands,
    '设置当前 ROS 域': `export ROS_DOMAIN_ID=${state.domain.match(/^\d+$/) ? state.domain : '99'}`}, 'shared') : '<p>等待后端连接。</p>');
}
let polling = false;
async function refreshState() {
  if (polling) return;
  polling = true;
  try {
    const query = {...selected};
    const data = await api(path('state', query));
    if (JSON.stringify(query) !== JSON.stringify(selected)) return;
    state = data;
    token = state.token;
    $('connection').textContent = '后端已连接';
    $('connection').className = 'badge good';
    $('domain').textContent = `ROS 域 ${state.domain}`;
    if (!refreshedPage) {
      refreshedPage = true;
      void api(path('page'), {}).catch(() => {});
    }
  } catch (err) {
    if (err.message.includes('标定文件不属于')) {
      selected = {};
      localStorage.removeItem('vservo-calibration');
    }
    state = null;
    if (wizard.result?.success) wizard.result = null;
    $('connection').textContent = '后端连接中断';
    $('connection').className = 'badge bad';
    if (view === 'overview') {
      for (const id of ['workflow', 'files', 'overview-cameras']) renderHTML(id, '<p>等待后端重新连接。</p>');
    }
  } finally {
    polling = false;
    render();
  }
}
function clearImage(box, reason) {
  const img = box.querySelector('img');
  if (urls.has(img)) {
    URL.revokeObjectURL(urls.get(img));
    urls.delete(img);
  }
  img.removeAttribute('src');
  img.hidden = true;
  const placeholder = box.querySelector('span');
  placeholder.textContent = reason;
  placeholder.hidden = false;
}
async function loadImage(box) {
  const {camera, variant} = box.dataset;
  try {
    if (!state?.cameras?.[camera]?.online) throw Error('相机离线或图像超时');
    const response = await fetch(path('image', {role: camera, variant}), {cache: 'no-store', signal: AbortSignal.timeout(2000)});
    if (!response.ok) throw Error('等待新图像');
    const blob = await response.blob();
    if (box.closest('[hidden]') || box.dataset.camera !== camera || box.dataset.variant !== variant) return;
    const img = box.querySelector('img');
    const url = URL.createObjectURL(blob);
    const previous = urls.get(img);
    urls.set(img, url);
    img.src = url;
    img.hidden = false;
    box.querySelector('span').hidden = true;
    if (previous) URL.revokeObjectURL(previous);
  } catch (err) {
    if (!box.closest('[hidden]') && box.dataset.camera === camera && box.dataset.variant === variant) clearImage(box, err.message);
  }
}
async function imageLoop() {
  await Promise.all([...$(view).querySelectorAll('.viewport')].map(loadImage));
  setTimeout(imageLoop, 250);
}
function show(next, role, purpose) {
  if (kinds.includes(next) && (view !== next || role || purpose)) {
    wizard = {role: role || 'eih', purpose: purpose || 'train', session: null, confirmed: new Set(), result: null,
      boardProfile: role === 'eth' && next === 'handeye' ? 'compact' : 'a4'};
  }
  for (const element of document.querySelectorAll('.view')) {
    element.hidden = element.id !== next;
    if (element.hidden) for (const box of element.querySelectorAll('.viewport')) clearImage(box, '等待图像');
  }
  for (const button of document.querySelectorAll('nav [data-view]')) {
    if (button.dataset.view === next) button.setAttribute('aria-current', 'page');
    else button.removeAttribute('aria-current');
  }
  view = next;
  if (next === 'sessions') void refreshSessions().catch(e => notice(e.message, 'bad'));
  if (next === 'reports') void refreshReports().catch(e => notice(e.message, 'bad'));
  if (next === 'diagnostics') void refreshLogs().catch(e => notice(e.message, 'bad'));
  render();
}
async function refreshSessions(focus) {
  const names = await api(path('sessions'));
  const list = $('session-list');
  const old = focus || list.value;
  list.replaceChildren(...names.map(name => new Option(name, name)));
  if (names.includes(old)) list.value = old;
  await loadSession(list.value);
}
async function loadSession(name) {
  gallery = name ? await api(path('session', {name})) : null;
  if (!gallery) {
    renderHTML('session-detail', '<p>暂无会话。</p>');
    return;
  }
  renderHTML('session-detail', `<p>${esc(gallery.role || gallery.plan?.role || '—')} · ${esc(gallery.mode || gallery.plan?.kind || '—')} · ${gallery.samples.length} 个样本</p>
    ${gallery.plan ? '<button data-action="resume">打开会话操作页</button>' : ''}
    <div class="gallery">${gallery.samples.slice(-120).map(s =>
      `<div class="sample"><img loading="lazy" alt="${esc(s.filename)}" src="${esc(path('sample', {name, filename: s.filename}))}">
      <p>${esc(s.filename)} · ${s.corners} 角点 · RMS ${fmt(s.reprojection_px)} px</p>
      ${s.resample ? `<p class="warn notice">${esc(s.resample)}</p>` : ''}
      <button data-action="mark" data-file="${esc(s.filename)}" data-marked="${!!s.resample}">${s.resample ? '取消标记' : '标记重采'}</button></div>`).join('')}</div>`);
}
async function refreshReports() {
  const names = await api(path('reports'));
  const list = $('report-list');
  const old = list.value;
  list.replaceChildren(...names.map(name => new Option(name, name)));
  if (names.includes(old)) list.value = old;
  await loadReport(list.value);
}
async function loadReport(name) {
  const report = name ? await api(path('report', {name})) : null;
  renderHTML('report-detail', report ?
    `<p>${badge(report.data?.passed === true ? '通过' : report.data?.passed === false ? '未通过' : '无通过结论',
      report.data?.passed === true ? 'good' : 'warn')} 退出码 ${esc(report.exit_code ?? (report.inferred_code === null ? '未记录' : `${report.inferred_code}（按报告推断）`))}</p>
      ${report.data ? `<a href="${esc(path('report', {name, download: 1}))}" download="${esc(name.split('/').pop())}">下载 JSON</a>${pre(report.data)}` : '<p>脚本没有生成报告，请查看终端错误输出。</p>'}` :
    '<p>暂无报告。使用会话命令运行独立验证，或按操作手册运行定位精度脚本。</p>');
}
async function refreshLogs() {
  const rows = await api(path('logs'));
  renderHTML('logs', rows.map(r => `<div class="check-row"><code>${esc(r.time)} · ${esc(r.event)}</code><div>${esc(r.message)}</div></div>`).join(''));
}
async function copyText(text) {
  try {
    await navigator.clipboard.writeText(text);
  } catch {
    const area = document.createElement('textarea');
    area.value = text;
    document.body.appendChild(area);
    area.select();
    const copied = document.execCommand('copy');
    area.remove();
    if (!copied) throw Error('浏览器未授权剪贴板，请手动复制命令。');
  }
  notice('命令已复制');
}
async function handleAction(button) {
  const action = button.dataset.action;
  const {role, purpose, session} = wizard;
  if (action === 'role' || action === 'purpose') {
    show(view, button.dataset.role || role, button.dataset.purpose || purpose);
  } else if (action === 'next') {
    if (purpose === 'check') show('overview');
    else show(view, role, 'check');
  } else if (action === 'new') {
    const active = wizard;
    const board_profile = active.boardProfile;
    const created = await api(path('session'), {role, kind: view, purpose, board_profile, selected: selected[keyFor(role, view)] || ''});
    active.session = created;
    active.result = null;
    active.confirmed.clear();
    notice(`已创建会话 ${created.name}`);
    render();
  } else if (['capture', 'solve', 'verify', 'start-calibration', 'start-cameras', 'start-monitor',
              'stop-monitor', 'start-motion', 'stop-motion', 'speed'].includes(action)) {
    if (busy) return;
    const active = wizard;
    const sessionAction = ['capture', 'solve', 'verify', 'start-calibration'].includes(action);
    busy = true;
    if (sessionAction) active.result = null;
    render();
    try {
      if (action === 'speed' && ['servo-linear', 'servo-angular', 'servo-accel'].some(id => !$(id).reportValidity()))
        throw Error('请填写范围内的速度和加速度');
      const result = await api(path(action), {name: session?.name, selected,
        percent: action === 'speed' ? Number($('servo-speed').value) : undefined,
        linear_mm_s: action === 'speed' ? Number($('servo-linear').value) : undefined,
        angular_deg_s: action === 'speed' ? Number($('servo-angular').value) : undefined,
        joint_accel_rad_s2: action === 'speed' ? Number($('servo-accel').value) : undefined,
        confirmed: action === 'start-motion' || active.confirmed.size === checks.length});
      if (action === 'speed' && result.success) {
        state.speed_percent = result.percent;
        state.motion_profile = result.motion_profile;
        speedEdited = false;
      }
      if (sessionAction) active.result = result;
      notice(result.message, result.success ? 'good' : 'bad');
    } catch (err) {
      if (sessionAction) active.result = {success: false, message: err.message};
      throw err;
    } finally {
      busy = false;
      render();
      void refreshState();
    }
  } else if (action === 'adopt') {
    setSelected(role, view, button.dataset.file);
    notice('已选择新结果。重新启动采集或监控后生效。');
  } else if (action === 'samples') {
    show('sessions');
    await refreshSessions(session.name);
  } else if (action === 'resume') {
    const saved = gallery;
    show(saved.plan.kind, saved.plan.role, saved.plan.purpose);
    wizard.session = saved;
    render();
  } else if (action === 'mark') {
    const reason = button.dataset.marked === 'true' ? '' : window.prompt('重采原因', '角点质量不足或姿态需重采');
    if (reason === null) return;
    await api(path('resample'), {name: gallery.name, filename: button.dataset.file, reason});
    await loadSession(gallery.name);
  } else if (action === 'diagnose') {
    show('diagnostics');
    const rows = await api(path('diagnostics'), {selected});
    renderHTML('diagnostic-result', rows.map(r => `<div class="check-row"><strong>${esc(r.name)}</strong> ${badge(r.ok ? '通过' : '待排查', r.ok ? 'good' : 'warn')}<p>${esc(r.detail)}</p>${r.ok ? '' : `<p>${esc(r.hint)}</p>`}</div>`).join(''));
    await refreshLogs();
  } else if (action === 'process-log') {
    renderHTML('process-output', `<pre>${esc(await api(path('process-log', {name: button.dataset.name})))}</pre>`);
  } else if (action === 'logs') await refreshLogs();
  else if (action === 'refresh-sessions') await refreshSessions();
  else if (action === 'refresh-reports') await refreshReports();
}
document.addEventListener('click', async event => {
  const button = event.target.closest('button');
  if (!button) return;
  try {
    if (button.dataset.copy) await copyText(commandValues.get(button.dataset.copy));
    else if (button.dataset.view) show(button.dataset.view, button.dataset.role, button.dataset.purpose);
    else if (button.dataset.action) await handleAction(button);
  } catch (err) {
    notice(err.message, 'bad');
  }
});
document.addEventListener('change', event => {
  const input = event.target;
  if (input.id === 'board-profile') wizard.boardProfile = input.value;
  const {selectRole, selectKind, check} = input.dataset;
  if (selectRole) setSelected(selectRole, selectKind, input.value);
  if (check !== undefined) {
    if (input.checked) wizard.confirmed.add(Number(check));
    else wizard.confirmed.delete(Number(check));
    render();
  }
  if (input.id === 'session-list') void loadSession(input.value).catch(err => notice(err.message, 'bad'));
  if (input.id === 'report-list') void loadReport(input.value).catch(err => notice(err.message, 'bad'));
});
document.addEventListener('input', event => {
  if (['servo-speed', 'servo-linear', 'servo-angular', 'servo-accel'].includes(event.target.id)) {
    speedEdited = true;
    $('servo-speed-draft').textContent = `${$('servo-speed').value}%`;
  }
});
for (const kind of kinds) buildWizard(kind);
void refreshState();
void imageLoop();
setInterval(refreshState, 1000);
})();
