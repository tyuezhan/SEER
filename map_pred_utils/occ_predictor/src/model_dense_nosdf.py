
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from IPython import embed
from aspp3d import ASPP
import time
from common import DEBUG_VERBOSE

def upsample(x, scale):
    ''' 3d upsample (trt need it)'''
    x_shape = torch.tensor(x.shape)
    return F.interpolate(x, size=(int(x_shape[2] * scale), int(x_shape[3] * scale), int(x_shape[4] * scale)), mode='nearest')

def count_num_model_params(model):
    num = 0
    for p in list(model.parameters()):
        cur = 1
        for s in list(p.size()):
            cur = cur * s
        num += cur
    return num

FSIZE0 = 3
FSIZE1 = 2
# ACTIVATE = nn.LeakyReLU(0.2, inplace=True)
ACTIVATE = nn.ReLU(inplace=True)
FIX = False

def bridge_block(in_dim, out_dim):
    return nn.Sequential(
        nn.Conv3d(in_dim, out_dim, kernel_size=3, stride=1, padding=1),
        nn.BatchNorm3d(out_dim),
        # nn.LeakyReLU(0.2, inplace=True),
        nn.ReLU(inplace=True),
        nn.Conv3d(out_dim, out_dim, kernel_size=3, stride=1, padding=1),
        nn.BatchNorm3d(out_dim),)

class conv_block(nn.Module):
    ''' down sample bloc'''
    def __init__(self, in_dim, out_dim, use_bias=False):
        super(conv_block, self).__init__()
        self.block = nn.Sequential(
            nn.Conv3d(in_dim, out_dim, kernel_size=2, stride=2, padding=0, bias=True),
            # nn.Conv3d(in_dim, out_dim, kernel_size=4, stride=2, padding=1, bias=True),
            nn.BatchNorm3d(out_dim),
            nn.ReLU(inplace=True),
        )

    def forward(self, x):
        x = self.block(x)
        return x

class upconv_block(nn.Module):
    ''' up sample bloc'''

    def __init__(self, in_dim, out_dim, stride=1, use_bias=False):
        super(upconv_block, self).__init__()
        self.block = nn.Sequential(
            nn.ConvTranspose3d(in_dim, out_dim, kernel_size=4, stride=2, padding=1, output_padding=1),
            nn.BatchNorm3d(out_dim),
            nn.ReLU(inplace=True)
        )

    def forward(self, x):
        x = self.block(x)
        return x

 
# def vgg_block(self, inplanes, planes, stride=1, use_bias=False):
#     block = nn.Sequential(
#         nn.Conv3d(planes, planes * self.expansion, kernel_size=1, bias=use_bias),
#         nn.BatchNorm3d(planes),
#         nn.ReLU(inplace=True),
#     )
#     return block


class UNet(nn.Module):
    '''
    Suppose reps = [3,3,3]
    '''
    def __init__(self, nPlanes, reps):
        super(UNet, self).__init__()
        assert reps == 1
        assert len(nPlanes) == 3

        # self.up = nn.Upsample(scale_factor=2)

        self.res1 = conv_block(nPlanes[0], nPlanes[1])
        # self.bn1 = nn.BatchNorm3d(nPlanes[0])
        self.res2 = conv_block(nPlanes[1], nPlanes[2])
        # self.bn2 = nn.BatchNorm3d(nPlanes[1])
        self.bridge = bridge_block(nPlanes[2], nPlanes[2])

        self.bn_relu = nn.Sequential(
            nn.BatchNorm3d(nPlanes[2]+nPlanes[1]+nPlanes[0]),
            nn.ReLU(inplace=True)           
        )

        # self.up1 = upconv_block(nPlanes[2], nPlanes[2])
        # self.up2 = upconv_block(nPlanes[2]+nPlanes[1], nPlanes[2]+nPlanes[1])
        # self.up2 = upconv_block(nPlanes[2]+nPlanes[1]+nPlanes[0], nPlanes[2]+nPlanes[1]+nPlanes[0])

    def forward(self, x):
        x0 = x
        x1 = self.res1(x0)
        x = self.res2(x1)
        x = self.bridge(x)

        # x = self.up(x)
        x = upsample(x, 2)
        x = torch.cat((x, x1), dim=1)

        # x = self.up(x)
        x = upsample(x, 2)

        x = torch.cat((x, x0), dim=1)

        return self.bn_relu(x)

class PreEncoderLayer(nn.Module):
    ''' sparse encoder layer:
    p0: dense input to sparse
    p1: conv, input channels to output channels, f=3
    p2: conv, save output as feature2
    p3: conv, downsample by 2, save output as feature3 (if p4)
    p4: sparse to dense 
    TODO: detail of scn.InputLayer & scn.SparseToDense
    '''
    def __init__(self, nf_in, nf):
        nn.Module.__init__(self)
        data_dim = 3
        self.nf_in = nf_in
        self.nf = nf
        self.use_bias = False
        # (8, 3, 1, 1)
        self.p1 = nn.Conv3d(nf_in, nf, kernel_size=FSIZE0, stride=1, padding=1, bias=self.use_bias)
        # (8, 3, 1, 1) * 2
        self.p2 = nn.Sequential(
            nn.BatchNorm3d(nf),
            nn.ReLU(True),
            nn.Conv3d(nf, nf, kernel_size=FSIZE0, stride=1, padding=1, bias=self.use_bias),
            nn.BatchNorm3d(nf),
            nn.ReLU(True),
            nn.Conv3d(nf, nf, kernel_size=FSIZE0, stride=1, padding=1, bias=self.use_bias),
        )

        self.p2_2 = nn.Sequential(
            nn.BatchNorm3d(nf),
            nn.ReLU(True),
        )

        # (8, 2, 2, 0)
        self.p3 = nn.Sequential(
            nn.Conv3d(nf, nf, kernel_size=FSIZE1, stride=2, padding=0, bias=self.use_bias),
            nn.BatchNorm3d(nf),
            nn.ReLU(True)
        )

    def forward(self,x):
        x0 = self.p1(x)
        x = self.p2(x0) + x0
        x = self.p2_2(x)
        ft2 = x
        x = self.p3(x)
        return x, [ft2]


class TSDFEncoder(nn.Module):
    '''
    sparse conv layers: 4, decrease resolution by 2 for 3 times, input is sparse, last layer's output is dense
    dense encoder-decoder: 16-8-4-4-8-16, with skip contacts, output is dense
    occ + sdf pred: take the output of dense-decoder, use 1*1*1 conv to output occ / sdf prediction
    '''
    def __init__(self, nf_in, nf_per_level, nf_out, use_skip_sparse=True, use_skip_dense=True):
        nn.Module.__init__(self)
        assert (type(nf_per_level) is list)
        data_dim = 3
        self.use_skip_sparse = use_skip_sparse
        self.use_skip_dense = use_skip_dense
        self.use_aspp = False
        #self.use_bias = True
        self.use_bias = False
        modules = []

        for level in range(len(nf_per_level)):
            nf_in = nf_in if level == 0 else nf_per_level[level-1]
            modules.append(PreEncoderLayer(nf_in, nf_per_level[level]))
        self.process_sparse = nn.Sequential(*modules)
        # print("TSDF SPARSE: ", self.process_sparse)
        nf = nf_per_level[-1]
        # 16 -> 8
        nf0 = nf*3 // 2
        self.encode_dense0 = nn.Sequential(
            nn.Conv3d(nf, nf0, kernel_size=4, stride=2, padding=1, bias=self.use_bias),
            nn.BatchNorm3d(nf0),
            nn.ReLU(True)
        )

        nf1 = nf*2
        if self.use_aspp:
            self.aspp = ASPP(nf0, nf1)
            # 8 -> 4
            self.encode_dense1 = nn.Sequential(
                nn.Conv3d(nf1, nf1, kernel_size=4, stride=2, padding=1, bias=self.use_bias),
                nn.BatchNorm3d(nf1),
                nn.ReLU(True)
            )
        else:
            # Note: Here, use_aspp = False
            # 8 -> 4
            self.encode_dense1 = nn.Sequential(
                nn.Conv3d(nf0, nf1, kernel_size=4, stride=2, padding=1, bias=self.use_bias),
                nn.BatchNorm3d(nf1),
                nn.ReLU(True)
            )

        # 4 -> 4
        nf2 = nf1
        self.bottleneck_dense2 = nn.Sequential(
            nn.Conv3d(nf1, nf2, kernel_size=1, bias=self.use_bias),
            nn.BatchNorm3d(nf2),
            nn.ReLU(True)
        )
        # 4 -> 8
        nf3 = nf2 if not self.use_skip_dense else nf1+nf2
        nf4 = nf3 // 2
        self.decode_dense3 = nn.Sequential(
            nn.ConvTranspose3d(nf3, nf4, kernel_size=4, stride=2, padding=1, bias=self.use_bias),
            nn.BatchNorm3d(nf4),
            nn.ReLU(True)
        )
        # 8 -> 16
        if self.use_skip_dense:
            nf4 += nf0
        nf5 = nf4 // 2
        self.decode_dense4 = nn.Sequential(
            nn.ConvTranspose3d(nf4, nf5, kernel_size=4, stride=2, padding=1, bias=self.use_bias),
            nn.BatchNorm3d(nf5),
            nn.ReLU(True)
        )
        self.final = nn.Sequential(
            nn.Conv3d(nf5, nf_out, kernel_size=1, bias=self.use_bias),
            nn.BatchNorm3d(nf_out),
            nn.ReLU(True)
        )
        # occ prediction
        self.occpred = nn.Sequential(
            nn.Conv3d(nf_out, 1, kernel_size=1, bias=self.use_bias)
        )
        # self.sdfpred = nn.Sequential(
        #     nn.Conv3d(nf_out, 1, kernel_size=1, bias=self.use_bias)
        # )
        # debug stats
        # params_encodesparse = count_num_model_params(self.process_sparse)
        # params_encodedense = count_num_model_params(self.encode_dense0) + count_num_model_params(self.encode_dense0) + count_num_model_params(self.encode_dense1) + count_num_model_params(self.bottleneck_dense2)
        # params_decodedense = count_num_model_params(self.decode_dense3) + count_num_model_params(self.decode_dense4) + count_num_model_params(self.final) + count_num_model_params(self.occpred)
        # print('[TSDFEncoder] params encode sparse', params_encodesparse)
        # print('[TSDFEncoder] params encode dense', params_encodedense)
        # print('[TSDFEncoder] params decode dense', params_decodedense)

    def forward(self,x):
        feats_sparse = []
        for k in range(len(self.process_sparse)):
            x, ft = self.process_sparse[k](x)
            if self.use_skip_sparse:
                feats_sparse.extend(ft)
        feats_sparse.append(x)
        if DEBUG_VERBOSE:
            print("before encode: ", x.shape)


        enc0 = self.encode_dense0(x)
        if self.use_aspp:
            enc_aspp = self.aspp(enc0)
            enc1 = self.encode_dense1(enc_aspp)
        else:
            enc1 = self.encode_dense1(enc0)
        if DEBUG_VERBOSE:
            print("enc1: ", enc1.shape)

        bottleneck = self.bottleneck_dense2(enc1)
        if self.use_skip_dense:
            dec0 = self.decode_dense3(torch.cat([bottleneck, enc1], 1))
        else:
            dec0 = self.decode_dense3(bottleneck)
        if DEBUG_VERBOSE:
            print("dec0: ", dec0.shape)

        if self.use_skip_dense:
            x = self.decode_dense4(torch.cat([dec0, enc0], 1))
        else:
            x = self.decode_dense4(dec0)
        x = self.final(x)
        occ = self.occpred(x)
        if DEBUG_VERBOSE:
            print("occ shape: ", occ.shape)
        # sdf = self.sdfpred(x)
        # out = torch.cat([occ, sdf],1)

        # return x, out, feats_sparse
        return x, occ.repeat([1,2,1,1,1]), feats_sparse

class Refinement(nn.Module):
    '''
    to_next_level_locs: up sample, locs: x,y,z *= 2, extend 1 grid into 8 grids 
    geo filter: mast = Sigmoid(occ) > 0.5
    occ, sdf: use a nn.Linear to go trough 16 channels (into 1 channel)
    TODO: why p and n? why FullyConvolutionalNet? why Input / Output layers(for upsampling & Linear?)?
    '''
    def __init__(self, nf_in, nf, max_data_size, truncation=3, pass_occ=True, pass_feats=True):
        nn.Module.__init__(self)
        data_dim = 3
        self.pass_occ = pass_occ
        self.pass_feats = pass_feats
        self.nf_in = nf_in
        self.nf = nf
        self.truncation = truncation
        self.use_aspp = True

        self.p1 = nn.Sequential(
            nn.ConvTranspose3d(nf_in, nf, kernel_size=FSIZE0, stride=1, padding=1, bias=False),
            nn.BatchNorm3d(nf),
            nn.ReLU(True)
        )

        if self.use_aspp:
            self.p2 = ASPP(nf, nf) 
            self.p3 = nn.Sequential(
            nn.Conv3d(nf, nf, kernel_size=3, stride=1, padding=1, bias=False),
            nn.BatchNorm3d(nf),
            nn.ReLU(inplace=True)
        )
        else:
            self.p2 = UNet(nPlanes=[nf, nf, nf], reps=1)

        # self.up = self.up = nn.Upsample(scale_factor=2)

            self.p3 = nn.Sequential(
                nn.Conv3d(nf*3, nf, kernel_size=3, stride=1, padding=1, bias=False),
                nn.BatchNorm3d(nf),
                nn.ReLU(inplace=True)
            )

        # self.p3 = nn.Sequential(
        #     nn.Conv3d(nf, nf, kernel_size=3, stride=1, padding=1, bias=False),
        #     nn.BatchNorm3d(nf),
        #     nn.ReLU(inplace=True)
        # )        

        self.occ = nn.Conv3d(nf, 1, kernel_size=1, stride=1, padding=0, bias=False)
        # self.occ = nn.Conv3d(nf, 1, kernel_size=3, stride=2, padding=1, bias=self.use_bias)
        # self.sdf = nn.Conv3d(nf, 1, kernel_size=1, stride=1, padding=0, bias=False)

    def forward(self,x):
        if DEBUG_VERBOSE:
            print("Before convtrans:", x.shape)
        x = self.p1(x)
        if DEBUG_VERBOSE:
            print("after convtrans:", x.shape)
        x = self.p2(x)
        if DEBUG_VERBOSE:
            print("after ASPP:", x.shape)
        # x = self.up(x)
        x = upsample(x, 2)
        if DEBUG_VERBOSE:
            print("after upsample:", x.shape)

        x = self.p3(x)
        if DEBUG_VERBOSE:
            print("after conv3d (16, 3):", x.shape)
        occ = self.occ(x)
        if DEBUG_VERBOSE:
            print("after occ conv3d:", x.shape)
        # sdf = self.sdf(x)
        # mask out for next level processing
        # mask = (nn.Sigmoid()(occ) < 0.5)
        # #print('x', x.type(), x.shape, torch.min(x).item(), torch.max(x).item())
        # x[mask.repeat((1,x.shape[1],1,1,1))] *= 0
        # sdf[mask] *= 0

        # out = torch.cat([occ, sdf],1)
        # if self.pass_feats and self.pass_occ:
        #     feats = torch.cat((x, out), 1)
        # elif self.pass_feats:
        #     feats = x
        # elif self.pass_occ:
        #     feats = out
        return None, occ
        # return feats, out

# ==== model ==== #
class GenModel(nn.Module):
    def __init__(self, encoder_dim, input_nf, nf_coarse, nf, num_hierarchy_levels, pass_occ, pass_feats, use_skip_sparse, use_skip_dense, truncation=3):
        nn.Module.__init__(self)
        self.truncation = truncation
        self.pass_occ = pass_occ
        self.pass_feats = pass_feats
        self.max_pool = nn.MaxPool3d(kernel_size=2)
        # encoder
    
        input_dim = [1]
        #self.nf_per_level = [encoder_dim*(k+1) for k in range(num_hierarchy_levels-1)]
        # print(num_hierarchy_levels)
        self.nf_per_level = [int(encoder_dim*(1+float(k)/(num_hierarchy_levels-2))) for k in range(num_hierarchy_levels-1)] if num_hierarchy_levels > 2 else [encoder_dim]*(num_hierarchy_levels-1)
        # print("nf_per_level: ", self.nf_per_level)
        self.use_skip_sparse = use_skip_sparse
        self.encoder = TSDFEncoder(input_nf, self.nf_per_level, nf_coarse, self.use_skip_sparse, use_skip_dense)

        self.refine_sizes = [(np.array(input_dim) // (pow(2,k))).tolist() for k in range(num_hierarchy_levels-1)][::-1]
        self.nf_per_level.append(self.nf_per_level[-1])
        print('#params encoder', count_num_model_params(self.encoder))

        # sparse prediction
        self.data_dim = 3
        self.refinement = []
        for h in range(1, num_hierarchy_levels):
            nf_in = 0 if not self.use_skip_sparse else self.nf_per_level[num_hierarchy_levels - h]
            if pass_occ:
                nf_in += 2
            if pass_feats:
                nf_in += (nf_coarse if h == 1 else nf)
            # nf_in = 12 + 2 + 16, nf = 16, refine_size = 32 (from [32, 64, 128])
            print("refine nf_in nf: %d"%h,nf_in, nf)
            self.refinement.append(Refinement(nf_in, nf, self.refine_sizes[h-1], truncation=self.truncation, pass_occ=pass_occ, pass_feats=pass_feats))
        self.refinement = nn.Sequential(*self.refinement)
        # print("REFINEMENT: ", self.refinement)
        print('#params refinement', count_num_model_params(self.refinement))
        self.PRED_SURF = True
        # if self.PRED_SURF:
        #     nf_in = 0 if not self.use_skip_sparse else self.nf_per_level[0]
        #     nf_out = 1
        #     if pass_occ:
        #         nf_in += 2
        #     if pass_feats:
        #         nf_in += nf
        #         print("suf refine nf_in nf: ",nf_in, nf, nf_out)
            # self.surfacepred = SurfacePrediction(nf_in, nf, nf_out, self.refine_sizes[-1])
            # print('#params surfacepred', count_num_model_params(self.surfacepred))


    def forward(self, x, loss_weights):

        if len(x.shape) == 4:
            x.unsqueeze_(1)
        init_input = x
        init_known_mask = (init_input >= 0).squeeze_(1)
        init_input_1 = self.max_pool(init_input)
        init_known_mask_1 = (init_input_1 >= 0).squeeze_(1)

        # x = torch.cat((x, init_known_mask.float()), 1)

        outputs = []
        #print('[model] x', x[0].shape, x[1].shape, torch.max(x[0][:,0]).item(), torch.max(x[0][:,1]).item(), torch.max(x[0][:,2]).item())
        # encode
        x, out, feats_dense = self.encoder(x)
        batch_size = x.shape[0]

        outputs.append(out)
        #print('locs, feats', locs.shape, locs.type(), feats.shape, feats.type(), x.shape)
        #raw_input('sdflkj')
        x = torch.cat((x, out), 1)
        # fix conflit in occ layer
        if FIX:
            x[:,-2][init_known_mask_1] = init_input_1[:, 0][init_known_mask_1]

        # Refinement (After UNet model)
        for h in range(len(self.refinement)):
            if DEBUG_VERBOSE:
                print("Loss weight for current layer:", loss_weights[h+1])
            if loss_weights[h+1] > 0:
                if self.use_skip_sparse:
                    x = torch.cat((feats_dense[len(self.refinement)-h], x), 1)
                #print('[model] refine(%d) x_sparse(input)' % h, x_sparse[0].shape, torch.min(x_sparse[0]).item(), torch.max(x_sparse[0]).item())
                x, out = self.refinement[h](x)
                outputs.append(out)
                #print('[model] refine(%d) x_sparse' % h, x_sparse[0].shape, torch.min(x_sparse[0]).item(), torch.max(x_sparse[0]).item())
            else:
                outputs.append([])
        # surface prediction

        # correct occ layer
        # if self.PRED_SURF and loss_weights[-1] > 0:
        #     if FIX:
        #         x[:,-2][init_known_mask] = init_input[:, 0][init_known_mask]
        #     if self.use_skip_sparse:
        #         x = torch.cat((feats_dense[0], x), 1)
        #         # x_sparse = self.concat_skip(feats_sparse[0][0], x_sparse, feats_sparse[0][1], batch_size)
        #     x = self.surfacepred(x)

        #     return x, outputs
        return None, outputs
 
if __name__ == '__main__':
    use_cuda = False

    model = GenModel(encoder_dim=8, input_nf=1, nf_coarse=16, nf=16, num_hierarchy_levels=2, pass_occ=True, pass_feats=True, use_skip_sparse=1, use_skip_dense=1)
    # model = GenModel_fix(input_dim=(128,128,128), input_nf=1, nf_coarse=16, nf=16, num_hierarchy_levels=2, pass_occ=True, pass_feats=True, use_skip_sparse=1, use_skip_dense=1)
    # exit()
    # batch size 10 -- all batches are identical
    input = torch.rand(1,1,80,80,80)
    print('input', input.shape, torch.min(input).item(), torch.max(input).item())
    if use_cuda:
        model = model.cuda()
        input = input.cuda()
    model.eval()
    print(model)
    times = []
    ts = time.time()
    with torch.no_grad():
        for i in range(100):
            # ts = time.time()
            output_sdf, output_occs = model(input, loss_weights=[1, 1, 1, 1])
            # inferen_time = time.time() - ts
            # times.append(inferen_time)
            # time.sleep(0.1)
    # print("average time: ", np.mean(times))
    print("average time: ", (time.time() - ts) / 100)

    print('output_occs', output_occs[0].shape, output_occs[1].shape)

