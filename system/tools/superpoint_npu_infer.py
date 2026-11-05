#!/usr/bin/env python3
# SuperPoint 特征前端 — 昇腾 310P (torch_npu) 推理验证
# 输入：任一图像目录或单张图；输出：关键点/描述子数量统计（证明 NPU 推理可用）
import sys, time, argparse

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', default='/data0/slam-datasets/modality_E/superpoint_v1.pth')
    ap.add_argument('--image', required=True)
    ap.add_argument('--H', type=int, default=120)
    ap.add_argument('--W', type=int, default=160)
    ap.add_argument('--nms_dist', type=int, default=4)
    args = ap.parse_args()

    import torch, torch_npu
    import cv2
    import numpy as np

    device = 'npu' if torch.npu.is_available() else 'cpu'
    print(f'device: {device}')

    # 简化版 SuperPoint 网络（官方结构：shared encoder + 两个 head）
    # 权重来自 magicleap/SuperPointPretrainedNetwork (superpoint_v1.pth)
    class SuperPointNet(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.relu = torch.nn.ReLU(inplace=True)
            self.pool = torch.nn.MaxPool2d(kernel_size=2, stride=2)
            c1, c2, c3, c4, d = 64, 64, 128, 128, 256  # c4=encoder out=128; d=256
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
            semi = self.convPb(cPa)          # [B,65,H/8,W/8]
            cDa = self.relu(self.convDa(x))
            desc = self.convDb(cDa)          # [B,256,H/8,W/8]
            desc = desc / torch.norm(desc, p=2, dim=1, keepdim=True)
            return semi, desc, x

    net = SuperPointNet()
    ckpt = torch.load(args.weights, map_location='cpu', weights_only=False)
    if isinstance(ckpt, dict) and 'model_state_dict' in ckpt:
        ckpt = ckpt['model_state_dict']
    net.load_state_dict(ckpt)
    net = net.to(device).eval()
    print('model loaded to', device)

    img = cv2.imread(args.image, 0)
    if img is None:
        print('cannot read image:', args.image); sys.exit(1)
    # KITTI 图太大，中心裁剪到 H*8 x W*8 的倍数（SuperPoint 需要 8 的倍数）
    hh, ww = args.H * 8, args.W * 8
    ch, cw = img.shape[0]//2, img.shape[1]//2
    crop = img[ch-hh//2:ch+hh//2, cw-ww//2:cw+ww//2]
    inp = torch.from_numpy(crop.astype(np.float32) / 255.0)[None, None]
    inp = inp.to(device)

    with torch.no_grad():
        # warmup
        for _ in range(3):
            semi, desc, _ = net(inp)
        if device == 'npu':
            torch.npu.synchronize()
        t0 = time.time()
        for _ in range(10):
            semi, desc, _ = net(inp)
        if device == 'npu':
            torch.npu.synchronize()
        dt = (time.time() - t0) / 10

    # 取关键点（简单 argtopk）
    semi_np = semi.cpu().numpy()[0]          # [65,h,w]
    heatmap = np.exp(semi_np[:-1])           # 去 dustbin
    n_kp = int((heatmap > 0.015).sum())
    print(f'inference: {dt*1000:.1f} ms/frame on {device}')
    print(f'detected keypoints (raw>0.015): {n_kp}')
    print(f'desc shape: {desc.shape}')
    print('NPU inference verified' if device == 'npu' else 'NPU unavailable, ran on CPU')

if __name__ == '__main__':
    main()
