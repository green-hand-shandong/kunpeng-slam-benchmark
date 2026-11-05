#!/usr/bin/env python3
# SuperPoint 双目匹配验证：左右目各自提取特征 → 描述子最近邻 → 视差直方图
# 证明：SuperPoint NPU 前端的输出可直接支撑双目几何（模态 E 接入 B/C 管线的证据）
import sys, time, argparse

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', default='/tmp/sp_test/superpoint_v1.pth')
    ap.add_argument('--left', required=True)
    ap.add_argument('--right', required=True)
    ap.add_argument('--H', type=int, default=46)   # 368/8
    ap.add_argument('--W', type=int, default=76)   # 608/8 ≈ 1226/16
    args = ap.parse_args()

    import torch, torch_npu
    import cv2
    import numpy as np

    device = 'npu' if torch.npu.is_available() else 'cpu'
    print(f'device: {device}')

    class SuperPointNet(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.relu = torch.nn.ReLU(inplace=True)
            self.pool = torch.nn.MaxPool2d(kernel_size=2, stride=2)
            c1, c2, c3, c4, d = 64, 64, 128, 128, 256
            self.conv1a = torch.nn.Conv2d(1, c1, 3, padding=1)
            self.conv1b = torch.nn.Conv2d(c1, c1, 3, padding=1)
            self.conv2a = torch.nn.Conv2d(c1, c2, 3, padding=1)
            self.conv2b = torch.nn.Conv2d(c2, c2, 3, padding=1)
            self.conv3a = torch.nn.Conv2d(c2, c3, 3, padding=1)
            self.conv3b = torch.nn.Conv2d(c3, c3, 3, padding=1)
            self.conv4a = torch.nn.Conv2d(c3, c4, 3, padding=1)
            self.conv4b = torch.nn.Conv2d(c4, c4, 3, padding=1)
            self.convPa = torch.nn.Conv2d(c4, 256, 3, padding=1)
            self.convPb = torch.nn.Conv2d(256, 65, 1)
            self.convDa = torch.nn.Conv2d(c4, 256, 3, padding=1)
            self.convDb = torch.nn.Conv2d(256, d, 1)
        def forward(self, x):
            x = self.relu(self.conv1a(x)); x = self.relu(self.conv1b(x)); x = self.pool(x)
            x = self.relu(self.conv2a(x)); x = self.relu(self.conv2b(x)); x = self.pool(x)
            x = self.relu(self.conv3a(x)); x = self.relu(self.conv3b(x)); x = self.pool(x)
            x = self.relu(self.conv4a(x)); x = self.relu(self.conv4b(x))
            cPa = self.relu(self.convPa(x))
            semi = self.convPb(cPa)
            cDa = self.relu(self.convDa(x))
            desc = self.convDb(cDa)
            desc = desc / torch.norm(desc, p=2, dim=1, keepdim=True)
            return semi, desc, x

    net = SuperPointNet()
    ckpt = torch.load(args.weights, map_location='cpu', weights_only=False)
    net.load_state_dict(ckpt)
    net = net.to(device).eval()

    def extract(path):
        img = cv2.imread(path, 0)
        assert img is not None, path
        hh, ww = args.H * 8, args.W * 8
        ch, cw = img.shape[0]//2, img.shape[1]//2
        crop = img[ch-hh//2:ch+hh//2, cw-ww//2:cw+ww//2]
        inp = torch.from_numpy(crop.astype(np.float32)/255.0)[None, None].to(device)
        with torch.no_grad():
            semi, desc, _ = net(inp)
        semi_np = semi.cpu().numpy()[0]
        heat = np.exp(semi_np[:-1])                # [64,h,w]
        h, w = heat.shape[1], heat.shape[2]
        # 逐 cell 取最大值 > 阈值的作为关键点
        # 向量化：找所有超阈值 cell
        cells = np.argwhere(heat.max(axis=0) > 0.015)   # [N,2] (y,x)
        kps = [(int(x)*8, int(y)*8) for y, x in cells]
        dn = desc.cpu().numpy()[0]                      # [256,h,w]
        descs = np.stack([dn[:, y, x] for y, x in cells]) if len(cells) else np.zeros((0,256), np.float32)
        return kps, descs, crop

    t0 = time.time()
    kpsL, desL, cropL = extract(args.left)
    kpsR, desR, _ = extract(args.right)
    t_extract = time.time() - t0

    # 最近邻匹配（L2）
    t1 = time.time()
    if len(kpsL) and len(kpsR):
        dL = torch.from_numpy(desL).to(device)
        dR = torch.from_numpy(desR).to(device)
        dist = torch.cdist(dL, dR)
        nn_idx = dist.argmin(dim=1).cpu().numpy()
        nn_val = dist.min(dim=1).values.cpu().numpy()
        matches = [(i, int(j), float(v)) for i, (j, v) in enumerate(zip(nn_idx, nn_val)) if v < 0.7]
    t_match = time.time() - t1

    # 视差统计（同一 y 行上的 x 差）
    dispar = []
    for i, j, v in matches:
        dy = abs(kpsL[i][1] - kpsR[j][1])
        dx = kpsL[i][0] - kpsR[j][0]
        if dy <= 2 and dx > 0:
            dispar.append(dx)
    dispar.sort()
    n_valid = len(dispar)
    med = dispar[n_valid//2] if n_valid else 0
    print(f'kpL={len(kpsL)} kpR={len(kpsR)} matches(|v|<0.7)={len(matches)}')
    print(f'same-row positive disparity: {n_valid}  median disparity: {med:.1f} px')
    if med > 0:
        # KITTI 09_30: fx=707.09, baseline=0.537m → depth = fx*B/d
        depth = 707.09 * 0.537150653267924 / med
        print(f'implied median depth: {depth:.1f} m  (期望 10~50m, 场景为居民区街道)')
    print(f'NPU extract: {t_extract*1000:.0f} ms (2 frames)  match: {t_match*1000:.0f} ms')
    ok = n_valid >= 50 and med > 3
    print('STEREO MATCHING VERIFIED' if ok else 'MATCHING WEAK — check calibration/params')

if __name__ == '__main__':
    main()
