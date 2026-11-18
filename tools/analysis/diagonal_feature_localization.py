"""斜め柱の型別観測による姿勢の遅延観測EKF（オフライン試作）。

状態は公称斜め軸の進行位置s[mm]・左位置e[mm]・左向き角psi[rad]。
必要に応じて特徴点基準の共通ずれb[mm]を不確かな校正量として持つ。
柱の極小、飽和通過区間、校正済み前センサを区別し、角度を0にしない。
画像は校正/独立評価の教師に限る。ファームウェアへは接続していない。
"""
from dataclasses import dataclass, field
import math
import numpy as np


@dataclass(frozen=True)
class FeatureModel:
    angle_rad: float = math.radians(55)
    offsets_mm: dict = field(default_factory=dict)
    noise_mm: dict = field(default_factory=dict)
    common_offset_std_mm: float = 0.
    phase_reference_calibrated: bool = False

    def key(self, side, surface): return str(side)+':'+surface

    def offset(self, side, surface):
        return float(self.offsets_mm.get(self.key(side,surface),self.offsets_mm.get(str(side),20.)))

    def noise(self, side, surface):
        return float(self.noise_mm.get(self.key(side,surface),6.))


@dataclass(frozen=True)
class PillarObservation:
    label: str
    side: int
    corner_s_mm: float
    corner_e_mm: float
    feature_ms: float
    confirmation_ms: float
    surface: str = 'unknown'
    width_mm: float = 0.
    saturated: bool = False
    gyro_radps: float = 0.
    flags: int = 0
    time_uncertainty_s: float = .0025

    def __post_init__(self):
        values=(self.corner_s_mm,self.corner_e_mm,self.feature_ms,self.confirmation_ms,
                self.width_mm,self.gyro_radps,self.time_uncertainty_s)
        if (self.side not in (1,2) or not all(math.isfinite(x) for x in values)
                or self.confirmation_ms<self.feature_ms or self.width_mm<0 or self.time_uncertainty_s<0):
            raise ValueError('Invalid pillar observation')


@dataclass(frozen=True)
class FrontObservation:
    label: str
    feature_ms: float
    confirmation_ms: float
    distance_mm: float
    range_state: int
    normal_s: float
    normal_e: float
    wall_offset_mm: float
    sensor_s_mm: float
    sensor_e_mm: float
    beam_rad: float
    sigma_mm: float = 5.
    geometry_calibrated: bool = False

    def __post_init__(self):
        values=(self.feature_ms,self.confirmation_ms,self.distance_mm,self.normal_s,
                self.normal_e,self.wall_offset_mm,self.sensor_s_mm,self.sensor_e_mm,
                self.beam_rad,self.sigma_mm)
        if (not all(math.isfinite(x) for x in values) or self.confirmation_ms<self.feature_ms
                or self.distance_mm<0 or self.sigma_mm<=0
                or not math.isclose(math.hypot(self.normal_s,self.normal_e),1.,abs_tol=1e-6)):
            raise ValueError('Invalid front observation')


def pillar_value_jacobian(state, observation, model):
    """F - cot(alpha)*D - A = 0。Dは各センサ側を正とする。"""
    s,e,psi=map(float,state[:3]);o=observation
    c,t=math.cos(psi),math.sin(psi);sign=1 if o.side==1 else -1
    cot=1/math.tan(model.angle_rad);ds,de=o.corner_s_mm-s,o.corner_e_mm-e
    forward=c*ds+t*de;side=sign*(-t*ds+c*de)
    hs=-c-cot*sign*t;he=-t+cot*sign*c
    value=forward-cot*side-model.offset(o.side,o.surface)
    h=np.array([hs,he,he*ds-hs*de])
    if len(state)==4:value-=float(state[3]);h=np.r_[h,-1.]
    return value,h,forward,side


def front_value_jacobian(state,o):
    """対応付け済み壁面への光線交点距離。壁面IDの確定/取付校正は別工程。"""
    s,e,psi=map(float,state[:3]);c,t=math.cos(psi),math.sin(psi)
    origin=np.array([s+c*o.sensor_s_mm-t*o.sensor_e_mm,e+t*o.sensor_s_mm+c*o.sensor_e_mm])
    derivative=np.array([-t*o.sensor_s_mm-c*o.sensor_e_mm,c*o.sensor_s_mm-t*o.sensor_e_mm])
    n=np.array([o.normal_s,o.normal_e]);beta=psi+o.beam_rad
    direction=np.array([math.cos(beta),math.sin(beta)])
    den=float(n@direction)
    if abs(den)<.35: return None
    numerator=o.wall_offset_mm-float(n@origin)
    distance=numerator/den
    if distance<0:return None
    den_derivative=float(n@np.array([-math.sin(beta),math.cos(beta)]))
    h=np.array([-n[0]/den,-n[1]/den,(-float(n@derivative)*den-numerator*den_derivative)/(den*den)])
    if len(state)==4:h=np.r_[h,0.]
    return distance-o.distance_mm,h


def interval_innovation(residual, half_width):
    """区間内は観測なし。区間外でも中央値へ引っ張らず、最寄り境界まで。"""
    if abs(residual)<=half_width:return 0.
    return -math.copysign(abs(residual)-half_width,residual)


def observation_policy(o,model,state):
    if isinstance(o,FrontObservation):
        if not o.geometry_calibrated:return 'front_uncalibrated',None,None
        if o.range_state!=0:return 'front_not_metric',None,None
        result=front_value_jacobian(state,o)
        if result is None:return 'front_grazing_or_behind',None,None
        if not math.isfinite(o.sigma_mm) or o.sigma_mm<=0:return 'front_invalid_noise',None,None
        return 'point',o.sigma_mm**2,0.
    # Rotation alone is handled by motion/time uncertainty. Unresolved coordinate
    # changes and invalid pose snapshots still must not become point constraints.
    if o.flags&(2|4|64):return 'unresolved_frame_or_pose',None,None
    if o.saturated and o.width_mm<=0:return 'saturation_without_bounded_interval',None,None
    value,h,forward,side=pillar_value_jacobian(state,o,model)
    timing=abs(o.gyro_radps)*o.time_uncertainty_s*math.hypot(forward,side)
    variance=model.noise(o.side,o.surface)**2+timing**2
    if o.flags&8:variance+=15.**2
    if o.saturated:
        half=o.width_mm*.5+model.noise(o.side,o.surface)
        return 'interval',variance+(o.width_mm*.5)**2,half
    return 'point',variance+o.width_mm**2/12,0.


def measurement_update(state,covariance,o,model,nis_limit=9.):
    x=np.asarray(state,float).copy();p=np.asarray(covariance,float).copy()
    policy,variance,half=observation_policy(o,model,x)
    record=dict(label=o.label,kind='front' if isinstance(o,FrontObservation) else 'pillar',policy=policy,
                accepted=False,confirmation_ms=o.confirmation_ms,feature_ms=o.feature_ms)
    if variance is None:return x,p,record
    if isinstance(o,FrontObservation):value,h=front_value_jacobian(x,o)
    else:value,h,_,_=pillar_value_jacobian(x,o,model)
    if isinstance(o,FrontObservation):record['progress_anchor']=abs(h[0])>=.2
    innovation=interval_innovation(value,half)
    if policy=='interval' and innovation==0:
        record.update(policy='interval_contains_prediction',innovation_mm=0.)
        return x,p,record
    total=float(h@p@h+variance);nis=innovation**2/total
    record.update(innovation_mm=innovation,nis=nis,measurement_variance_mm2=variance,interval_half_width_mm=half)
    if nis>nis_limit:record['policy']='nis_rejected';return x,p,record
    gain=p@h/total;x+=gain*innovation
    a=np.eye(len(x))-np.outer(gain,h);p=a@p@a.T+np.outer(gain,gain)*variance
    p=(p+p.T)*.5
    record.update(accepted=True,delta_s_mm=float(gain[0]*innovation),delta_e_mm=float(gain[1]*innovation),
                  delta_heading_deg=math.degrees(float(gain[2]*innovation)))
    return x,p,record


def propagate(state,covariance,history,start_ms,end_ms):
    if end_ms<start_ms-1e-6:raise ValueError('Backward propagation')
    x=np.asarray(state,float).copy();p=np.asarray(covariance,float).copy()
    times=history['time_ms']
    knots=np.r_[start_ms,times[(times>start_ms)&(times<end_ms)],end_ms]
    for a,b in zip(knots,knots[1:]):
        ds=float(np.interp(b,times,history['distance_mm'])-np.interp(a,times,history['distance_mm']))
        da=float(np.interp(b,times,history['gamma_rad'])-np.interp(a,times,history['gamma_rad']))
        mid=x[2]+da*.5;c,t=math.cos(mid),math.sin(mid)
        f=np.eye(len(x));f[0,2]=-ds*t;f[1,2]=ds*c
        q=np.zeros_like(p);q[:3,:3]=np.diag([.1**2*ds,.15**2*ds,math.radians(.03)**2*ds])
        x[:3]+=np.array([ds*c,ds*t,da]);p=f@p@f.T+q
    return x,p


def transform_axis(state,covariance,axis_turn_rad,origin_in_old_axis=(0.,0.)):
    """新軸へ平均と共分散を同時に変換。横位置・角度を0には戻さない。"""
    c,s=math.cos(axis_turn_rad),math.sin(axis_turn_rad)
    rotation=np.array([[c,s],[-s,c]])
    x=np.asarray(state,float).copy();x[:2]=rotation@(x[:2]-np.asarray(origin_in_old_axis,float));x[2]-=axis_turn_rad
    t=np.eye(len(x));t[:2,:2]=rotation
    return x,t@np.asarray(covariance,float)@t.T


def replay_confirmed(observations,history,model,prior,prior_time_ms,query_ms,prior_covariance=None,available_at_ms=None):
    """確認済みだけで特徴点時刻へ巻き戻して再生。未来の観測を使わない。"""
    if query_ms<prior_time_ms:return None
    if history['time_ms'][0]>prior_time_ms or history['time_ms'][-1]<query_ms:
        raise ValueError('Replay outside motion history')
    x=np.asarray(prior,float).copy()
    p=np.diag([60.**2,15.**2,math.radians(8.)**2]) if prior_covariance is None else np.asarray(prior_covariance,float).copy()
    if model.common_offset_std_mm>0 and len(x)==3:
        x=np.r_[x,0.];augmented=np.zeros((4,4));augmented[:3,:3]=p
        augmented[3,3]=model.common_offset_std_mm**2;p=augmented
    available_ms=query_ms if available_at_ms is None else available_at_ms
    if available_ms<query_ms:raise ValueError('Query is later than information availability')
    current=prior_time_ms;records=[];available=[o for o in observations if o.confirmation_ms<=available_ms and o.feature_ms<=query_ms]
    for o in sorted(available,key=lambda v:v.feature_ms):
        if o.feature_ms<prior_time_ms:continue
        x,p=propagate(x,p,history,current,o.feature_ms);current=o.feature_ms
        x,p,r=measurement_update(x,p,o,model);records.append(r)
    x,p=propagate(x,p,history,current,query_ms)
    accepted=sum(r['accepted'] for r in records)
    std=np.sqrt(np.maximum(0,np.diag(p)))
    ready=(accepted>=3 and std[1]<=6 and math.degrees(std[2])<=3
           and abs(x[1])<=40 and abs(x[2])<=math.radians(30))
    has_front_anchor=any(r['kind']=='front' and r['accepted'] and r.get('progress_anchor',False) for r in records)
    phase_ready=ready and std[0]<=12 and (model.phase_reference_calibrated or has_front_anchor)
    return dict(state=x.tolist(),covariance=p.tolist(),std_s_mm=float(std[0]),std_e_mm=float(std[1]),
        std_heading_deg=math.degrees(float(std[2])),ready=bool(ready),lateral_heading_ready=bool(ready),
        longitudinal_ready=bool(phase_ready),common_bias_estimate_mm=float(x[3]) if len(x)==4 else None,accepted=accepted,
        considered=len(records),maximum_confirmation_ms=max((o.confirmation_ms for o in available),default=None),updates=records)


def calibrate_feature_model(rows,angle_rad=math.radians(55),typed=True,shrink_count=2.):
    """走行単位の外部検証で使う校正。rowは訓練走行の画像教師だけ。"""
    offsets={};noise={};audit={};cot=1/math.tan(angle_rad)
    for side in (1,2):
        rr=[r for r in rows if r['side']==side]
        if not rr:continue
        values=np.array([r['forward_mm']-cot*r['side_mm'] for r in rr]);base=float(np.median(values));offsets[str(side)]=base
        keys=sorted({r['surface'] for r in rr}) if typed else ['shared']
        for surface in keys:
            vv=np.array([r['forward_mm']-cot*r['side_mm'] for r in rr if not typed or r['surface']==surface])
            median=float(np.median(vv));offset=(len(vv)*median+shrink_count*base)/(len(vv)+shrink_count) if typed else base
            key=str(side)+':'+surface;offsets[key]=offset
            spread=1.4826*float(np.median(abs(vv-median)))
            noise[key]=max(2.5,spread,6. if len(vv)<3 else 0.)
            audit[key]=dict(count=len(vv),offset_mm=offset,sigma_mm=noise[key],unshrunk_median_mm=median)
    return FeatureModel(angle_rad,offsets,noise),audit


def associate_landmark(state,covariance,observation,landmarks,model,last_progress=-math.inf,margin=2.):
    """左右・順序・予測残差で対応付ける。欠けた本数の3分割などは行わない。"""
    ranked=[]
    for p in landmarks:
        if p['side']!=observation.side or p['progress_mm']<=last_progress:continue
        candidate=PillarObservation(p['id'],p['side'],p['s_mm'],p['e_mm'],observation.feature_ms,
            observation.confirmation_ms,p['surface'],observation.width_mm,observation.saturated,
            observation.gyro_radps,observation.flags,observation.time_uncertainty_s)
        policy,r,half=observation_policy(candidate,model,state)
        if r is None:continue
        value,h,_,_=pillar_value_jacobian(state,candidate,model)
        innovation=interval_innovation(value,half)
        nis=innovation**2/float(h@covariance@h+r)
        if nis<=9:ranked.append((nis,p['id'],candidate,p['progress_mm']))
    ranked.sort(key=lambda v:v[0])
    if not ranked:return dict(accepted=False,reason='no_consistent_landmark')
    if len(ranked)>1 and ranked[1][0]-ranked[0][0]<margin:
        return dict(accepted=False,reason='ambiguous_landmark',scores=[x[0] for x in ranked])
    best=ranked[0]
    return dict(accepted=True,reason='ready',observation=best[2],progress_mm=best[3],nis=best[0])


class CorrectionSlew:
    """推定器とは別に、制御へ渡す補正を前進距離と時間の両方で制限する。"""
    def __init__(self):self.e_mm=0.;self.heading_rad=0.

    def update(self,target_e_mm,target_heading_rad,dt_s,travel_mm):
        if not all(math.isfinite(v) for v in (target_e_mm,target_heading_rad,dt_s,travel_mm)) or dt_s<0 or travel_mm<0:
            raise ValueError('Invalid slew input')
        e_limit=min(40*dt_s,.1*travel_mm)
        heading_limit=min(math.radians(10)*dt_s,math.radians(.025)*travel_mm)
        self.e_mm+=float(np.clip(target_e_mm-self.e_mm,-e_limit,e_limit))
        self.heading_rad+=float(np.clip(target_heading_rad-self.heading_rad,-heading_limit,heading_limit))
        return self.e_mm,self.heading_rad
