"""機体候補内の前後キーポイントを推定する小型CPU CNN。単独で盤面較正は行わない。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
_torch = None


def torch_runtime():
    global _torch
    if _torch is None:
        try:
            import torch
        except ModuleNotFoundError:
            location = ROOT/'.tools'/f'vision_python{sys.version_info.major}{sys.version_info.minor}'
            if location.is_dir():
                sys.path.insert(0,str(location))
            try:
                import torch
            except ImportError as error:
                raise RuntimeError('CNNにはCPU版PyTorchが必要です。tools/requirements-vision-cnn.txtを確認してください') from error
        torch.set_num_threads(1)
        _torch = torch
    return _torch


def build_network():
    torch = torch_runtime()
    nn = torch.nn

    class KeypointNet(nn.Module):
        def __init__(self):
            super().__init__()
            self.encoder = nn.Sequential(nn.Conv2d(4,12,3,padding=1),nn.SiLU(),
                nn.Conv2d(12,16,3,stride=2,padding=1),nn.SiLU(),
                nn.Conv2d(16,24,3,stride=2,padding=1),nn.SiLU(),
                nn.Conv2d(24,24,3,padding=1),nn.SiLU())
            self.heatmap = nn.Sequential(nn.Conv2d(28,12,3,padding=1),nn.SiLU(),nn.Conv2d(12,2,1))
            self.visibility = nn.Sequential(nn.Flatten(),nn.Linear(24*16*16,2))

        def forward(self,x):
            encoded = self.encoder(x)
            large = torch.nn.functional.interpolate(encoded,size=(64,64),mode='bilinear',align_corners=False)
            return self.heatmap(torch.cat((large,x),dim=1)),self.visibility(encoded)

    return KeypointNet()


def patch_features(patch):
    from body_appearance_model import patch_to_features
    feature = patch_to_features(patch)/np.array([255.,64.,64.,255.],np.float32)
    return np.moveaxis(feature,-1,0).astype(np.float32)


class BodyKeypointCNN:
    def __init__(self,network,metadata):
        self.network = network.eval()
        self.metadata = metadata

    @classmethod
    def load(cls,path):
        torch = torch_runtime()
        saved = torch.load(Path(path),map_location='cpu',weights_only=True)
        if saved.get('schema_version')!=1 or saved.get('backend')!='mouse_tiny_heatmap_cnn':
            raise ValueError('未対応の機体CNNモデルです')
        network = build_network()
        network.load_state_dict(saved['state_dict'],strict=True)
        return cls(network,json.loads(saved['metadata_json']))

    def save(self,path):
        torch = torch_runtime()
        path = Path(path)
        if path.exists() or path.with_suffix('.json').exists():
            raise FileExistsError('学習済みCNNを上書きしません: '+str(path))
        path.parent.mkdir(parents=True,exist_ok=True)
        torch.save(dict(schema_version=1,backend='mouse_tiny_heatmap_cnn',state_dict=self.network.state_dict(),
            metadata_json=json.dumps(self.metadata,ensure_ascii=False)),path)
        path.with_suffix('.json').write_text(json.dumps(self.metadata,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

    def predict_patch(self,patch):
        torch = torch_runtime()
        with torch.inference_mode():
            heatmap,visible = self.network(torch.from_numpy(patch_features(patch)[None]))
            logits = heatmap[0].reshape(2,-1)
            # 最大点近傍だけのsoft-argmaxで、遠い弱い副峰への重心の偏りを避ける。
            peak = logits.argmax(dim=1)
            yy,xx = torch.meshgrid(torch.arange(64),torch.arange(64),indexing='ij')
            coordinates = torch.stack((xx.flatten(),yy.flatten()),dim=1)
            points = []
            spreads = []
            for keypoint in range(2):
                distance = ((coordinates-coordinates[peak[keypoint]])**2).sum(dim=1)
                selected = distance<=36
                weights = torch.softmax(logits[keypoint,selected],dim=0)
                point = (weights[:,None]*coordinates[selected]).sum(dim=0)
                spread = (weights*((coordinates[selected]-point)**2).sum(dim=1)).sum().sqrt()
                points.append(point.numpy())
                spreads.append(float(spread))
            return np.asarray(points,np.float32),torch.sigmoid(visible[0]).numpy(),np.asarray(spreads)

    def refine(self,frame,front_px,rear_px):
        from body_appearance_model import extract_canonical_patch
        patch,affine = extract_canonical_patch(frame,front_px,rear_px)
        points,visible,spread = self.predict_patch(patch)
        inverse = cv2.invertAffineTransform(affine)
        native = cv2.transform(points[None],inverse)[0]
        return native[0],native[1],dict(cnn_visibility=visible.tolist(),cnn_spread_canonical_px=spread.tolist())

    @classmethod
    def fit_from_frames(cls,samples,metadata,*,steps=400,seed=29,progress=None,robust=False):
        """色ペアの擬似正解を拡張して学習。独立動画での精度検証とは区別する。"""
        from body_appearance_model import extract_canonical_patch
        if len(samples)<3 or not 1<=steps<=10000:
            raise ValueError('学習は3フレーム以上、stepsは1..10000が必要です')
        torch = torch_runtime()
        torch.manual_seed(seed)
        rng = np.random.default_rng(seed)
        xx,yy = np.meshgrid(np.arange(64),np.arange(64))
        features,targets,visibility = [],[],[]
        count = max(192,min(512,len(samples)*8))
        for index in range(count):
            frame,front,rear = samples[index%len(samples)]
            true = np.array([front,rear],np.float32)
            perturbed = true+rng.normal(0,2.8,size=(2,2)).astype(np.float32)
            patch,affine = extract_canonical_patch(frame,*perturbed)
            points = cv2.transform(true[None],affine)[0]
            if robust:
                rotate=cv2.getRotationMatrix2D((32.,32.),float(rng.uniform(-40,40)),1.)
                patch=cv2.warpAffine(patch,rotate,(64,64))
                points=cv2.transform(points[None],rotate)[0]
            patch = np.clip(patch.astype(np.float32)*rng.uniform(.15 if robust else .7,1.4 if robust else 1.25)+rng.uniform(-12,12),0,255).astype(np.uint8)
            if rng.random()<.4:
                patch = cv2.GaussianBlur(patch,(3,3),float(rng.uniform(.3,1.0)))
            if rng.random()<.25:
                size=9 if robust else 5
                kernel = np.zeros((size,size),np.float32)
                angle=float(rng.uniform(0,np.pi)) if robust else (0. if rng.random()<.5 else np.pi/2)
                vector=np.array([np.cos(angle),np.sin(angle)])*(size-1)/2
                center=np.array([size//2,size//2])
                cv2.line(kernel,tuple(np.rint(center-vector).astype(int)),tuple(np.rint(center+vector).astype(int)),1.,1)
                kernel/=kernel.sum()
                patch = cv2.filter2D(patch,-1,kernel)
            visible = (rng.random(2)>.15).astype(np.float32)
            heat = []
            for point,present in zip(points,visible):
                if not present:
                    cv2.circle(patch,tuple(np.rint(point).astype(int)),5,(60,60,60),-1)
                gaussian = np.exp(-((xx-point[0])**2+(yy-point[1])**2)/(2*1.5**2))
                heat.append(gaussian/gaussian.sum())
            features.append(patch_features(patch))
            targets.append(heat)
            visibility.append(visible)
        x = torch.from_numpy(np.asarray(features,np.float32))
        target = torch.from_numpy(np.asarray(targets,np.float32)).flatten(2)
        visible = torch.from_numpy(np.asarray(visibility,np.float32))
        network = build_network().train()
        optimizer = torch.optim.Adam(network.parameters(),lr=.002)
        loss_value = None
        for step in range(steps):
            selected = torch.randint(0,count,(16,))
            logits,presence = network(x[selected])
            point_loss = -(target[selected]*torch.nn.functional.log_softmax(logits.flatten(2),dim=-1)).sum(dim=-1)
            point_weights=torch.ones_like(visible[selected]) if robust else visible[selected]
            point_loss = (point_loss*point_weights).sum()/point_weights.sum().clamp_min(1)
            loss = point_loss+.3*torch.nn.functional.binary_cross_entropy_with_logits(presence,visible[selected])
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            optimizer.step()
            loss_value = float(loss.detach())
            if progress and (step==0 or (step+1)%100==0):
                progress(step+1,loss_value)
        meta = dict(metadata,backend='mouse_tiny_heatmap_cnn',sample_count=len(samples),
            augmented_samples=count,steps=steps,seed=seed,training_loss=loss_value,
            torch_version=str(torch.__version__),label_type='high_confidence_colour_pair_pseudo_labels',
            independent_accuracy_verified=False)
        meta.update(robust_augmentation=robust,hidden_point_targets_supervised=robust)
        return cls(network,meta)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    inputs=parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--trial',type=Path,nargs='+')
    inputs.add_argument('--patches',type=Path,help='train_turn_appearance.pyの学習パッチフォルダー')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-frames-per-trial',type=int,default=60)
    parser.add_argument('--steps',type=int,default=400)
    parser.add_argument('--robust',action='store_true')
    args = parser.parse_args()
    from body_appearance_model import harvest_training_samples_from_trial
    samples,sources = [],[]
    if args.patches:
        from body_appearance_model import FRONT_CANON_XY,REAR_CANON_XY
        saved=json.loads((args.patches/'training.json').read_text(encoding='utf-8'))
        meta=saved['metadata']
        sources=meta['sources']
        for label in saved['labels']:
            patch=cv2.imread(str(args.patches/label['patch']))
            if patch is None or patch.shape!=(64,64,3):raise ValueError('学習パッチが不正です')
            samples.append((patch,FRONT_CANON_XY,REAR_CANON_XY))
        mouse,front,rear=meta['mouse'],meta['front_colour'],meta['rear_colour']
    else:
        for path in args.trial:
            found,meta = harvest_training_samples_from_trial(path,args.max_frames_per_trial)
            samples.extend(found)
            sources.append(meta)
        identities = {(meta.get('mouse'),meta.get('front_colour'),meta.get('rear_colour')) for meta in sources}
        if len(identities)!=1 or None in next(iter(identities)):
            raise ValueError('機体と前後色が同じ試行だけを学習してください')
        mouse,front,rear = next(iter(identities))
    model = BodyKeypointCNN.fit_from_frames(samples,dict(mouse=mouse,front_colour=front,rear_colour=rear,sources=sources),
        steps=args.steps,robust=args.robust,progress=lambda step,loss:print(f'train step {step}: loss={loss:.4f}',flush=True))
    model.save(args.output)
    print(json.dumps(dict(saved=str(args.output),metadata=model.metadata),ensure_ascii=False))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
