# 双相机选择器，只维护观测和状态，不依赖 ROS。输入均为 base_T_target。
# 默认 first_seen：先连续看到目标的一路主导；主相机失效后才交接。
# COARSE 表示 ETH 主导，FINE 表示 EIH 主导；这两个名称不保证实际测量精度。
"""ETH acquisition, EIH refinement, hysteresis, loss and consistency checks."""
import numpy as np
from .geometry import error, validate


# 每路样本保存（图像时间戳、基座系目标矩阵、相机到目标距离），并记录连续可见起点。
class Selector:
    # 校验观测时效、配对时间差和历史 handoff 策略的滞回/确认参数。
    def __init__(self, max_age=0.3, pair_skew=0.10, agreement_m=0.04,
                 agreement_deg=12.0, enter_m=0.45, leave_m=0.55, confirmations=3,
                 handoff_timeout=1.0, require_pair_agreement=False, target_loss_timeout=2.0,
                 reacquire_delay=0.5, selection_policy='first_seen'):
        if selection_policy not in ('first_seen', 'eth_primary', 'handoff'):
            raise ValueError('Unknown camera selection policy')
        self.selection_policy = selection_policy
        if not np.isfinite(handoff_timeout) or handoff_timeout < max_age:
            raise ValueError('handoff_timeout must be finite and at least max_age')
        if (not np.isfinite([max_age, pair_skew, enter_m, leave_m, target_loss_timeout, reacquire_delay]).all()
                or max_age <= 0 or pair_skew < 0 or not 0 < enter_m < leave_m
                or type(confirmations) is not int or confirmations < 1
                or target_loss_timeout < handoff_timeout or not 0 <= reacquire_delay < target_loss_timeout):
            raise ValueError('Invalid freshness, handoff distances or reacquisition settings')
        self.max_age, self.pair_skew = max_age, pair_skew
        self.agreement_m, self.agreement_rad = agreement_m, np.deg2rad(agreement_deg)
        self.enter_m, self.leave_m, self.confirmations = enter_m, leave_m, confirmations
        self.handoff_timeout = handoff_timeout
        self.require_pair_agreement = require_pair_agreement
        self.target_loss_timeout, self.reacquire_delay = target_loss_timeout, reacquire_delay
        self.samples = {}
        self.visible_since = {}
        self.active_source = None
        self.fine, self.count = False, 0
        self.last_confirmed_stamp = None
        self.last_coarse_stamp = None
        self.last_target_stamp = None
        self.fine_lost_at = None

    # 接收比已存样本更新的观测；长时间中断后重新记录首次可见时刻。
    def update(self, role, stamp, T, distance):
        if role not in ('eih', 'eth') or not np.isfinite([stamp, distance]).all() or distance <= 0:
            raise ValueError('Invalid role / observation')
        if role in self.samples and stamp <= self.samples[role][0]:
            return
        pose = validate(T).copy()
        if role not in self.visible_since or stamp-self.samples[role][0] > self.max_age:
            self.visible_since[role] = stamp
            if self.active_source == role:
                self.active_source = None
        self.samples[role] = (stamp, pose, distance)

    # 过滤过期或未来观测，计算双路一致性诊断，然后按所选策略返回（状态、样本）。
    def choose(self, now):
        if self.last_target_stamp is not None and not 0 <= now-self.last_target_stamp <= self.target_loss_timeout:
            self.reset_handoff()
        live = {k: v for k, v in self.samples.items() if 0 <= now-v[0] <= self.max_age}
        eih, eth = live.get('eih'), live.get('eth')
        skew = abs(eih[0]-eth[0]) if eih and eth else None
        diag = dict(eih_distance_m=eih[2] if eih else None, eth_age_s=now-eth[0] if eth else None,
                    eih_age_s=now-eih[0] if eih else None, pair_skew_s=skew,
                    pair_status='skew' if skew is not None else 'missing',
                    translation_mm=None, rotation_deg=None, max_age_s=self.max_age,
                    max_skew_s=self.pair_skew, agreement_mm=self.agreement_m*1000,
                    agreement_deg=float(np.degrees(self.agreement_rad)), fine_enter_m=self.enter_m,
                    fine_leave_m=self.leave_m, handoff_timeout_s=self.handoff_timeout,
                    handoff_confirmations=self.confirmations, pair_enforced=self.require_pair_agreement,
                    target_loss_timeout_s=self.target_loss_timeout, reacquire_delay_s=self.reacquire_delay,
                    selection_policy=self.selection_policy)
        # 仅对时间足够接近的两路观测比较基座系位姿，避免把运动造成的差异误作标定偏差。
        if skew is not None and skew <= self.pair_skew:
            t, a = error(eih[1], eth[1])
            diag.update(translation_mm=t*1000, rotation_deg=float(np.degrees(a)), pair_status='agreed')
            if t > self.agreement_m or a > self.agreement_rad:
                diag['pair_status'] = 'inconsistent'
                if self.require_pair_agreement and self.selection_policy == 'handoff':
                    self.reset_handoff()
                    return self.result(diag, 'INCONSISTENT', 'pair_mismatch')
        # 默认策略保持当前有效主相机；双路偏差只显示，不在这里否决目标或反复抢占。
        if self.selection_policy == 'first_seen':
            diag['pair_enforced'] = False
            if self.active_source not in live:
                self.active_source = min(live, key=lambda role: self.visible_since.get(role, live[role][0])) if live else None
            if self.active_source is None:
                self.visible_since.clear()
                self.fine = False
                return self.result(diag, 'HOLD', 'both_lost')
            self.fine = self.active_source == 'eih'
            return self.result(diag, 'FINE' if self.fine else 'COARSE',
                               f'{self.active_source}_leading', live[self.active_source])
        # 历史策略：ETH 有效时始终优先，只有 ETH 不可用才使用 EIH。
        if self.selection_policy == 'eth_primary':
            diag['pair_enforced'] = False
            if eth:
                self.fine = False
                return self.result(diag, 'COARSE', 'eth_primary', eth)
            if eih:
                self.fine = True
                return self.result(diag, 'FINE', 'eih_fallback', eih)
            self.fine = False
            return self.result(diag, 'HOLD', 'both_lost')
        # 以下为历史 handoff 策略：近距离进入 EIH，较远阈值才退出，形成距离滞回。
        if self.fine:
            if eih and eih[2] <= self.leave_m:
                return self.result(diag, 'FINE', 'eih_tracking', eih)
            self.fine, self.count, self.last_confirmed_stamp = False, 0, None
            self.fine_lost_at = now
            return self.result(diag, 'HOLD', 'eih_lost')

        coarse_age = now-self.last_coarse_stamp if self.last_coarse_stamp is not None else None
        recent_coarse = coarse_age is not None and 0 <= coarse_age <= self.handoff_timeout
        recovering_eih = self.fine_lost_at is not None and self.last_target_stamp is not None
        if (recent_coarse or recovering_eih) and eih and eih[2] <= self.enter_m:
            # 确认计数按不同图像帧增加，不能靠同一帧被多个定时周期重复读取来凑数。
            if eih[0] != self.last_confirmed_stamp:
                self.count += 1
                self.last_confirmed_stamp = eih[0]
            if self.count >= self.confirmations:
                self.fine = True
                self.fine_lost_at = None
                return self.result(diag, 'FINE', 'eih_tracking', eih)
        else:
            self.count = 0
            self.last_confirmed_stamp = None
        if self.fine_lost_at is not None and now-self.fine_lost_at < self.reacquire_delay:
            return self.result(diag, 'HOLD', 'eih_confirming' if self.count else 'eih_lost')
        if eth:
            self.last_coarse_stamp = eth[0]
            self.fine_lost_at = None
            return self.result(diag, 'COARSE', 'eih_confirming' if self.count else 'eth_tracking', eth)
        if recent_coarse:
            return self.result(diag, 'HOLD', 'eih_confirming' if self.count else 'waiting_eih')
        self.last_coarse_stamp = None
        return self.result(diag, 'HOLD', 'waiting_eth')

    # 处理带时间戳的丢失事件：只移除不晚于该事件的样本，不能删除更新观测。
    def invalidate(self, role, stamp):
        sample = self.samples.get(role)
        if sample is not None and sample[0] <= stamp:
            self.samples.pop(role)
            self.visible_since.pop(role, None)
            if self.active_source == role:
                self.active_source = None

    # 清空主导权与滞回历史；样本仍由 choose 的时效检查决定是否可用。
    def reset_handoff(self):
        self.fine, self.count = False, 0
        self.last_confirmed_stamp = self.last_coarse_stamp = None
        self.last_target_stamp = self.fine_lost_at = None
        self.active_source = None

    # 统一记录原因、来源和交接计数；仅在实际选中样本时更新最近目标时间。
    def result(self, diagnostics, state, reason, sample=None):
        if sample is not None:
            self.last_target_stamp = sample[0]
        self.diagnostics = dict(diagnostics, reason=reason, handoff_count=self.count,
                                source={'COARSE': 'eth', 'FINE': 'eih'}.get(state))
        return state, sample
