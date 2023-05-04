# YOLOv5 YOLO-specific modules

import argparse
import logging
import sys
from copy import deepcopy

sys.path.append('./')  # to run '$ python *.py' files in subdirectories
logger = logging.getLogger(__name__)

from models.common import *
from models.experimental import *
from utils.autoanchor import check_anchor_order
from utils.general import make_divisible, check_file, set_logging
from utils.torch_utils import time_synchronized, fuse_conv_and_bn, model_info, scale_img, initialize_weights, \
    select_device, copy_attr

try:
    import thop  # for FLOPS computation
except ImportError:
    thop = None



class SegMaskBiSe(nn.Module):  # 配置文件输入[16, 19, 22]通道无效
    def __init__(self, n_segcls=19, n=1, c_hid=256, shortcut=False, ch=()):  # n是C3的, c_hid是C3的输出通道数(接口保留了,没有使用,可用子模块控制s,m,l加深加宽)
        super(SegMaskBiSe, self).__init__()
        self.c_in8 = ch[0]  # 16 L
        self.c_in16 = ch[1]  # 19
        self.c_in32 = ch[2]  # 22
        self.c_out = n_segcls

        self.m8 = nn.Sequential(  # 未采用双流结构
                               Conv(self.c_in8, 128, k=1, s=1), 
                               )
        self.m16 = nn.Sequential(
                               RFB2(self.c_in16, 128, map_reduce=4, d=[2,3], has_globel=False),
                               )
        self.m32 = nn.Sequential(
                               RFB2(self.c_in32, 128, map_reduce=8, d=[2,3], has_globel=True),  # 舍弃原GP，在1/32(和1/16，可选)处加全局特征
                               )

        self.up16 = nn.Sequential(
                               Conv(128, 128, 3),  # refine论文源码每次up后一个3*3refine，降低计算量放在前
                               nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True),
        )
        self.up32 = nn.Sequential(
                               Conv(128, 128, 3),  # refine
                               nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True),
        )
        self.out = nn.Sequential(
                               FFM(256, 256, k=3),  
                               nn.Dropout(0.1),  # 最后一层改用3*3，我认为用dropout不合适（dropout对3*3响应空间维度形成遮挡），改为dropout2d（随机整个通道置０增强特征图独立性，空间上不遮挡）
                               nn.Conv2d(256, self.c_out, kernel_size=1, padding=0),
                               nn.Upsample(scale_factor=8, mode='bilinear', align_corners=True),
        )
        # 辅助分割头，训练用，推理丢弃
        self.aux16 = nn.Sequential(
                               Conv(128, 128, 3),
                               nn.Conv2d(128, self.c_out, kernel_size=1),
                               nn.Upsample(scale_factor=8, mode='bilinear', align_corners=True),
        )
        self.aux32 = nn.Sequential(
                               Conv(128, 128, 3),
                               nn.Conv2d(128, self.c_out, kernel_size=1),
                               nn.Upsample(scale_factor=16, mode='bilinear', align_corners=True),
        )

    def forward(self, x):
        feat3 = self.up32(self.m32(x[2]))  #  + GP) 
        feat2 = self.up16(self.m16(x[1]) + feat3)
        feat1 = [self.m8(x[0]), feat2]
        return self.out(feat1) if not self.training else [self.out(feat1), self.aux16(feat2), self.aux32(feat3)]


class Detect(nn.Module):  # 检测头
    stride = None  # strides computed during build
    export = False  # onnx export

    def __init__(self, nc=80, anchors=(), ch=()):  # detection layer
        super(Detect, self).__init__()
        self.nc = nc  # number of classes
        self.no = nc + 5  # number of outputs per anchor 每个anchor输出通道=nc类别+1是否有目标+4放缩偏移量
        self.nl = len(anchors)  # number of detection layers anchors是列表的列表,外层几个列表表示有几个层用于输出
        self.na = len(anchors[0]) // 2  # number of anchors  内层列表表示该层anchor形状尺寸,//即该层anchor数
        self.grid = [torch.zeros(1)] * self.nl  # init grid
        a = torch.tensor(anchors).float().view(self.nl, -1, 2)
        self.register_buffer('anchors', a)  # shape(nl,na,2) anchor参数是模型非计算图参数,用register_buffer保存(buffer parameter)
        self.register_buffer('anchor_grid', a.clone().view(self.nl, 1, -1, 1, 1, 2))  # shape(nl,1,na,1,1,2)
        self.m = nn.ModuleList(nn.Conv2d(x, self.no * self.na, 1) for x in ch)  # output conv 三个输出层输入通道不一样

    # gird和输出特征图一样大,值对应此anchor中心, anchor_grid张量也同尺寸, 两个值对应了此anchor的尺寸
    def forward(self, x):
        # x = x.copy()  # for profiling
        z = []  # inference output
        self.training |= self.export
        for i in range(self.nl):  # 分别对三个输出层处理
            x[i] = self.m[i](x[i])  # conv
            bs, _, ny, nx = x[i].shape  # x(bs,255,20,20) to x(bs,3,20,20,85)
            # 输出x[i]变形BCHW(C=na*no) --> B,na,H,W,no(由第二维区分三个anchor),  no=nc+5,  x是3个张量的列表, 一个张量表一个输出层
            x[i] = x[i].view(bs, self.na, self.no, ny, nx).permute(0, 1, 3, 4, 2).contiguous()

            if not self.training:  # inference
                if self.grid[i].shape[2:4] != x[i].shape[2:4]:
                    self.grid[i] = self._make_grid(nx, ny).to(x[i].device)

                y = x[i].sigmoid()  # 所有通道输出sigmoid, 后1+类别数通道自然表示有无目标和目标种类, 前4个通道按公式偏移放缩anchor
                y[..., 0:2] = (y[..., 0:2] * 2. - 0.5 + self.grid[i]) * self.stride[i]  # xy 中心偏移公式见issue
                y[..., 2:4] = (y[..., 2:4] * 2) ** 2 * self.anchor_grid[i]  # wh 大小放缩公式见issue
                z.append(y.view(bs, -1, self.no))                           # 0输入时保证0偏移, 中心0输入0.5输出,偏到grid中心(yolo anchor从左上角算起))
        # 训练直接返回变形后的x去求损失, 推理对                                    # 大小0输入1输出,乘以anchor尺寸不变, 公式限制最大放大倍数为4倍
        return x if self.training else (torch.cat(z, 1), x)  # 注意训练模式和测试(以及推理)模式不同, 训练模式仅返回变形后的x, 测试推理返回放缩偏移后的box(即z)和变形后x

    @staticmethod
    def _make_grid(nx=20, ny=20):  # 用来生成anchor中心(特征图每个像素下标即其anchor中心)的函数
        yv, xv = torch.meshgrid([torch.arange(ny), torch.arange(nx)])
        return torch.stack((xv, yv), 2).view((1, 1, ny, nx, 2)).float()


class Model(nn.Module):  # 核心模型
    def __init__(self, cfg='yolov5s.yaml', ch=3, nc=None, anchors=None):  # model, input channels, number of classes
        super(Model, self).__init__()
        if isinstance(cfg, dict):  # 配置可直接接收字典
            self.yaml = cfg  # model dict
        else:  # is *.yaml  更多是用yaml解析配置
            import yaml  # for torch hub
            self.yaml_file = Path(cfg).name
            with open(cfg) as f:
                self.yaml = yaml.load(f, Loader=yaml.SafeLoader)  # model dict

        # Define model
        ch = self.yaml['ch'] = self.yaml.get('ch', ch)  # input channels 字典的get方法,配置文件有ch就把模型输入通道配成ch,没有就按默认值ch=3
        if nc and nc != self.yaml['nc']:  # 若Model类初始化指定了nc(非None)且和配置文件不等,以Model类初始化值为准,并修改字典值
            logger.info(f"Overriding model.yaml nc={self.yaml['nc']} with nc={nc}")
            self.yaml['nc'] = nc  # override yaml value
        if anchors:  # 若Model类初始化指定了anchor值,以Model类初始化为准,并修改字典值
            logger.info(f'Overriding model.yaml anchors with anchors={anchors}')
            self.yaml['anchors'] = round(anchors)  # override yaml value
        self.model, self.save = parse_model(deepcopy(self.yaml), ch=[ch])  # model, savelist 解析配置文件
        self.save.append(24)  # 增加记录分割层
        self.names = [str(i) for i in range(self.yaml['nc'])]  # default names
        # print([x.shape for x in self.forward(torch.zeros(1, ch, 64, 64))])

        # Build strides, anchors
        m = self.model[-1]  # Detect()  Detect头
        if isinstance(m, Detect):
            s = 256  # 2x min stride
            m.stride = torch.tensor([s / x.shape[-2] for x in self.forward(torch.zeros(2, ch, s, s))[0]])  # forward
            m.anchors /= m.stride.view(-1, 1, 1)
            check_anchor_order(m)
            self.stride = m.stride
            self._initialize_biases()  # only run once
            # print('Strides: %s' % m.stride.tolist())

        # Init weights, biases
        initialize_weights(self)  # 初始化, 看代码只初始化了BN和激活函数,跳过了卷积层
        self.info()
        logger.info('')

    def forward(self, x, augment=False, profile=False):
        if augment:
            img_size = x.shape[-2:]  # height, width
            s = [1, 0.83, 0.67]  # scales
            f = [None, 3, None]  # flips (2-ud, 3-lr)
            y = []  # outputs
            for si, fi in zip(s, f):
                xi = scale_img(x.flip(fi) if fi else x, si, gs=int(self.stride.max()))
                yi = self.forward_once(xi)[0]  # forward
                # cv2.imwrite(f'img_{si}.jpg', 255 * xi[0].cpu().numpy().transpose((1, 2, 0))[:, :, ::-1])  # save
                yi[..., :4] /= si  # de-scale
                if fi == 2:
                    yi[..., 1] = img_size[0] - yi[..., 1]  # de-flip ud
                elif fi == 3:
                    yi[..., 0] = img_size[1] - yi[..., 0]  # de-flip lr
                y.append(yi)
            return torch.cat(y, 1), None  # augmented inference, train
        else:
            return self.forward_once(x, profile)  # single-scale inference, train

    def forward_once(self, x, profile=False):
        y, dt = [], []  # outputs  用于记录中间输出的y, profile时间的dt
        out = []  # 用于保存改版后的分割+检测输出
        for m in self.model:
            if m.f != -1:  # if not from previous layer 非单纯上一层则需要调整此层输入
                x = y[m.f] if isinstance(m.f, int) else [x if j == -1 else y[j] for j in m.f]  # from earlier layers
                    # 输入来自单层, 直接取那层输出           来自多层, 其中-1取输入x, 非-1取那层输出

            if profile:
                o = thop.profile(m, inputs=(x,), verbose=False)[0] / 1E9 * 2 if thop else 0  # FLOPS
                t = time_synchronized()
                for _ in range(10):
                    _ = m(x)
                dt.append((time_synchronized() - t) * 100)
                print('%10.1f%10.0f%10.1fms %-40s' % (o, m.np, dt[-1], m.type))
            # 调好输入每层都是直接跑, detect是最后一层, for循环最后一个自然是detect结果
            x = m(x)  # run
            # print(m.i, m.type, x.shape if m.f !=-1 else [a.shape for a in x])
            y.append(x if m.i in self.save else None)  # save output 解析时self.save记录了需要保存的那些层(后续层输入用到),仅保存这些层输出即可(改版代码新增记录分割层24)

        if profile:
            print('%.1fms total' % sum(dt))

        return [x, y[-2]]  # 检测, 分割

    def _initialize_biases(self, cf=None):  # initialize biases into Detect(), cf is class frequency
        # https://arxiv.org/abs/1708.02002 section 3.3
        # cf = torch.bincount(torch.tensor(np.concatenate(dataset.labels, 0)[:, 0]).long(), minlength=nc) + 1.
        m = self.model[-1]  # Detect() module
        for mi, s in zip(m.m, m.stride):  # from
            b = mi.bias.view(m.na, -1)  # conv.bias(255) to (3,85)
            b.data[:, 4] += math.log(8 / (640 / s) ** 2)  # obj (8 objects per 640 image)
            b.data[:, 5:] += math.log(0.6 / (m.nc - 0.99)) if cf is None else torch.log(cf / cf.sum())  # cls
            mi.bias = torch.nn.Parameter(b.view(-1), requires_grad=True)

    def _print_biases(self):
        m = self.model[-1]  # Detect() module
        for mi in m.m:  # from
            b = mi.bias.detach().view(m.na, -1).T  # conv.bias(255) to (3,85)
            print(('%6g Conv2d.bias:' + '%10.3g' * 6) % (mi.weight.shape[1], *b[:5].mean(1).tolist(), b[5:].mean()))

    # def _print_weights(self):
    #     for m in self.model.modules():
    #         if type(m) is Bottleneck:
    #             print('%10.3g' % (m.w.detach().sigmoid() * 2))  # shortcut weights

    def fuse(self):  # fuse model Conv2d() + BatchNorm2d() layers
        print('Fusing layers... ')
        for m in self.model.modules():
            if type(m) is Conv and hasattr(m, 'bn'):
                m.conv = fuse_conv_and_bn(m.conv, m.bn)  # update conv
                delattr(m, 'bn')  # remove batchnorm
                m.forward = m.fuseforward  # update forward
        self.info()
        return self

    def nms(self, mode=True):  # add or remove NMS module
        present = type(self.model[-1]) is NMS  # last layer is NMS
        if mode and not present:
            print('Adding NMS... ')
            m = NMS()  # module
            m.f = -1  # from
            m.i = self.model[-1].i + 1  # index
            self.model.add_module(name='%s' % m.i, module=m)  # add
            self.eval()
        elif not mode and present:
            print('Removing NMS... ')
            self.model = self.model[:-1]  # remove
        return self

    def autoshape(self):  # add autoShape module
        print('Adding autoShape... ')
        m = autoShape(self)  # wrap model
        copy_attr(m, self, include=('yaml', 'nc', 'hyp', 'names', 'stride'), exclude=())  # copy attributes
        return m

    def info(self, verbose=False, img_size=640):  # print model information
        model_info(self, verbose, img_size)


def parse_model(d, ch):  # model_dict, input_channels(3)
    logger.info('\n%3s%18s%3s%10s  %-40s%-30s' % ('', 'from', 'n', 'params', 'module', 'arguments'))
    anchors, nc, gd, gw, n_segcls = d['anchors'], d['nc'], d['depth_multiple'], d['width_multiple'], d['n_segcls']
    na = (len(anchors[0]) // 2) if isinstance(anchors, list) else anchors  # number of anchors
    no = na * (nc + 5)  # number of outputs = anchors * (classes + 5) yolo输出通道数 = anchor数 * (类别+1个是否有目标+4个偏移放缩量)

    layers, save, c2 = [], [], ch[-1]  # layers, savelist, ch out
    for i, (f, n, m, args) in enumerate(d['backbone'] + d['head']):  # from, number, module, args
        m = eval(m) if isinstance(m, str) else m  # eval strings 执行字符串表达式,block名转函数/类,字符数字转数字
        for j, a in enumerate(args):
            try:
                args[j] = eval(a) if isinstance(a, str) else a  # eval strings 同上,
            except:
                pass
        # n控制深度, yaml配置文件中num为1就1次,num>1就 num*depth_multiple次, 即此block本身以及block子结构重复次数
        n = max(round(n * gd), 1) if n > 1 else n  # depth gain
        if m in [Conv, GhostConv, Bottleneck, GhostBottleneck, SPP, DWConv, MixConv2d, Focus, CrossConv, BottleneckCSP,
                 C3, C3TR, ASPP]:
            c1, c2 = ch[f], args[0]  # 指定层输入(c1)输出(c2)通道数(ch记录各层输出通道,f表输入层下标,输入层的输出通道就是本层输入通道)
            if c2 != no:  # if not output 对非输出层, 原作者此处代码有风险
                c2 = make_divisible(c2 * gw, 8)  # 实际输出通道数是 配置文件的c2 * width_multiple 并向上取到可被8整除

            args = [c1, c2, *args[1:]]
            if m in [BottleneckCSP, C3, C3TR]:
                args.insert(2, n)  # number of repeats 对C3和BottleneckCSP来说深度n代表残差模块的个数, C3TR的n表transformer的head数
                n = 1  # 置1表示深度对这三个模块是控制子结构重复, 而不是本身重复
        elif m is nn.BatchNorm2d:
            args = [ch[f]]  # 对BN层, 参数就是输入层的通道数
        elif m is Concat:
            c2 = sum([ch[x] for x in f])  # Concat层, 输出通道就是几个输入层通道数相加
        elif m is Detect:
            args.append([ch[x] for x in f])  # 检测层, 把来源下标列表f中的层输出通道数加入args中, 用于构建Detect的卷积输入通道数
            if isinstance(args[1], int):  # number of anchors 一般跑不进这句, args[1]是anchors在配置文件中已用列表写好, 非int
                args[1] = [list(range(args[1] * 2))] * len(f)
        elif m in [SegMaskBiSe]:  # 语义分割头
            args[1] = max(round(args[1] * gd), 1) if args[1] > 1 else args[1]  # SegMask 中 C3 的n(Lab里用来控制ASPP砍多少通道)
            args[2] = make_divisible(args[2] * gw, 8)  # SegMask C3(或其他可放缩子结构) 的输出通道数
            args.append([ch[x] for x in f])
            # n = 1 不用设1了, SegMask自己配置文件的n永远1
        elif m is Contract:
            c2 = ch[f] * args[0] ** 2
        elif m is Expand:
            c2 = ch[f] // args[0] ** 2
        else:
            c2 = ch[f]

        m_ = nn.Sequential(*[m(*args) for _ in range(n)]) if n > 1 else m(*args)  # module 深度控制C3等的block子结构重复次数(见上if中n置为1), 对Conv等则是其本身重复次数
        t = str(m)[8:-2].replace('__main__.', '')  # module type
        np = sum([x.numel() for x in m_.parameters()])  # number params
        m_.i, m_.f, m_.type, m_.np = i, f, t, np  # attach index, 'from' index, type, number params
        logger.info('%3s%18s%3s%10.0f  %-40s%-30s' % (i, f, n, np, t, args))  # print
        save.extend(x % i for x in ([f] if isinstance(f, int) else f) if x != -1)  # append to savelist 由来源记哪些层的结果保存
        layers.append(m_)  # 解析结果加到layers列表
        if i == 0:
            ch = []  # 如果第一层,新建ch列表保存输出通道数
        ch.append(c2)  # 保存此层输出通道数, 下一层输入通道
    return nn.Sequential(*layers), sorted(save)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--cfg', type=str, default='yolov5s_city_seg.yaml', help='model.yaml')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    opt = parser.parse_args()
    opt.cfg = check_file(opt.cfg)  # check file
    set_logging()
    device = select_device(opt.device)

    # Create model
    model = Model(opt.cfg).to(device)
    model.train()
  #  model.eval()
    pass
    # a = torch.randn((1, 3, 1024, 2048), device=device)
    # model(a)
    # Profile
    # img = torch.rand(8 if torch.cuda.is_available() else 1, 3, 640, 640).to(device)
    # y = model(img, profile=True)

    # Tensorboard
    # from torch.utils.tensorboard import SummaryWriter
    # tb_writer = SummaryWriter()
    # print("Run 'tensorboard --logdir=models/runs' to view tensorboard at http://localhost:6006/")
    # tb_writer.add_graph(model.model, img)  # add model to tensorboard
    # tb_writer.add_image('test', img[0], dataformats='CWH')  # add model to tensorboard
