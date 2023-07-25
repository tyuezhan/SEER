
import math
import torch
import torch.nn as nn
import torch.nn.functional as F


class _ASPPModule(nn.Module):
    def __init__(self, inplanes, planes, kernel_size, padding, dilation):
        super(_ASPPModule, self).__init__()
        self.atrous_conv = nn.Conv3d(inplanes, planes, kernel_size=kernel_size,
                                            stride=1, padding=padding, dilation=dilation, bias=False)
        self.bn = nn.BatchNorm3d(planes)
        self.relu = nn.LeakyReLU(0.2, inplace=True)

        self._init_weight()

    def forward(self, x):
        x = self.atrous_conv(x)
        x = self.bn(x)

        return self.relu(x)

    def _init_weight(self):
        for m in self.modules():
            if isinstance(m, nn.Conv3d):
                torch.nn.init.kaiming_normal_(m.weight)
            elif isinstance(m, nn.BatchNorm3d):
                m.weight.data.fill_(1)
                m.bias.data.zero_()

class ASPP(nn.Module):
    def __init__(self, inplanes, output_stride):

        super(ASPP, self).__init__()
        self.inplanes = inplanes
        self.outplanes = output_stride
        mid_planes = 16
        dilations = [1, 2, 6]
        self.aspp1 = _ASPPModule(inplanes, mid_planes, 1, padding=0, dilation=dilations[0])
        self.aspp2 = _ASPPModule(inplanes, mid_planes, 3, padding=dilations[1], dilation=dilations[1])
        self.aspp3 = _ASPPModule(inplanes, mid_planes, 3, padding=dilations[2], dilation=dilations[2])

        self.global_avg_pool = nn.Sequential(nn.AdaptiveAvgPool3d((1, 1, 1)),
                                             nn.Conv3d(inplanes, mid_planes, 1, stride=1, bias=False),
                                             nn.BatchNorm3d(mid_planes),
                                             nn.ReLU()
                                            )

        self.conv1 = nn.Conv3d(mid_planes * 4, inplanes, 1, bias=False)
        self.bn1 = nn.BatchNorm3d(output_stride)
        self.relu = nn.ReLU()

        self._init_weight()

    def forward(self, x):
        input = x
        x1 = self.aspp1(x)
        x2 = self.aspp2(x)
        x3 = self.aspp3(x)

        x4 = self.global_avg_pool(x)
        # print("global pool:", x4.shape)
        dimx, dimy, dimz = x3.size()[2], x3.size()[3], x3.size()[4]
        # x4 = F.interpolate(x4, size=(dimx, dimy, dimz), mode='nearest') # , align_corners=True
        x4 = x4.repeat([1, 1, dimx, dimy, dimz])
        # print(" ...", x4.shape)

        x = torch.cat((x1, x2, x3, x4), dim=1)
        x = self.conv1(x)
        if self.inplanes == self.outplanes:
            x = x + input
        x = self.bn1(x)
        x = self.relu(x)

        return x #self.dropout(x)

    def _init_weight(self):
        for m in self.modules():
            if isinstance(m, nn.Conv3d):
                torch.nn.init.kaiming_normal_(m.weight)
            elif isinstance(m, nn.BatchNorm3d):
                m.weight.data.fill_(1)
                m.bias.data.zero_()

